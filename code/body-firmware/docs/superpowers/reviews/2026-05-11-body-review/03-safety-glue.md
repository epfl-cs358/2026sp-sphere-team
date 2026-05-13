# Safety/Correctness × Glue & Lifecycle — Cell 3

Scope: `src/util/` (excl. `sync/CommandLatch.*`), `src/config/`, all `src/main_*.cpp`.
Anchor: f8ca95f. Tests inspected: `test_moving_average`, `test_passthrough_controller`.

## P0

```json
{
  "id": "C3-P0-01",
  "title": "Loose H-bridge pins between Wire.begin and motor.begin can float to PWM-high during boot",
  "file": "src/main_robot.cpp",
  "line_range": "215-219",
  "quote": "Wire.begin(I2C_SDA, I2C_SCL);\n\n    g_motor0.begin();\n    g_motor1.begin();\n    g_motor2.begin();",
  "risk": "The six L298N IN3/IN4 GPIOs (14, 27, 16, 17, 25, 26) are not driven LOW before `pinMode(..., OUTPUT)` runs inside each motor's `begin()`. On a power-on reset they default to floating inputs with weak pulls; on a soft `ESP.restart()` they can retain prior PWM state through the brief gap. `main_imu_test.cpp:22-32` explicitly contains `killMotorPins()` for exactly this reason, but `main_robot.cpp` skips it. If WiFi/IMU init aborted mid-run from a watchdog reset while a wheel was driving forward, the motor pin can briefly remain at its last LEDC duty until `g_motor0.begin()` overwrites it. Worst case is a runaway wheel for tens of ms after every brownout/restart.",
  "suggested_fix": "Mirror `main_imu_test.cpp`'s `killMotorPins()` and call it as the very first action in `setup()` before any other init. Also call `g_motor*.begin()` *before* `g_imu.begin()`/`connectWifi()` so a 30 s WiFi timeout doesn't leave motor pins floating for 30 s.",
  "test_gap": "No native test exercises boot-time pin state; no `test_l298n_driver_begin_leaves_pins_low` exists."
}
```

```json
{
  "id": "C3-P0-02",
  "title": "Motors armed for 30 s before WiFi/control task come up — staleness ramp not yet running",
  "file": "src/main_robot.cpp",
  "line_range": "217-247",
  "quote": "g_motor0.begin();\n    g_motor1.begin();\n    g_motor2.begin();\n\n    if (!g_imu.begin()) {",
  "risk": "Boot order is: motors begin (pins go OUTPUT) → IMU begin → connectWifi (blocks up to 30 s) → producer start → OTA → TWDT init → controlTask create. Between line 219 and line 240 the H-bridge IN pins are configured outputs (driven LOW by `pinMode`), but there is no task pulling commands or running the staleness watchdog. A glitch on the L298N's enable rail or a stuck PWM channel from before the soft reset has a 30 s window to express itself before the control task takes ownership. The 30 s WiFi timeout (`WIFI_TIMEOUT_MS=30000`) is the worst-case latency to first safe state.",
  "suggested_fix": "Construct latch + controller and start `controlTask` *before* `connectWifi()`. The task will run with `last_known={0,0,0}` and `have_seen_fresh=false` ⇒ `drive_cmd=0` every tick — actively zeroing motors via PID setpoint=0 instead of relying on `pinMode` defaults. Defer producer start until after WiFi.",
  "test_gap": "No test for `controlTask` behaviour with `have_seen_fresh=false`; the staleness-ramp logic in main_robot.cpp:160-183 lives inside the task body and has zero unit coverage."
}
```

```json
{
  "id": "C3-P0-03",
  "title": "TWDT init happens AFTER `g_producer->start()` — ws_prod's `esp_task_wdt_add(NULL)` runs against a 5 s default",
  "file": "src/main_robot.cpp",
  "line_range": "232-240",
  "quote": "g_producer->start();\n\n    initOta();\n\n    // Tighten the global Task Watchdog timeout. Affects IDLE tasks too, but\n    // 1s is comfortably above their normal slack.\n    esp_task_wdt_init(TWDT_TIMEOUT_S, true);",
  "risk": "`WebSocketCommandProducer::taskBody()` calls `esp_task_wdt_add(NULL)` at line 103, then loops. Because `start()` returns immediately after `xTaskCreatePinnedToCore`, that task can subscribe to the watchdog *before* `esp_task_wdt_init(1, true)` runs on the main task. ESP-IDF's behaviour on subscribing to an uninitialised TWDT differs by core release; on current arduino-esp32 the producer task ends up under the IDF-default ~5 s timeout, not the 1 s the rest of the system expects. Net effect: a hung WS task is undetected for 4 extra seconds and operators believe TWDT will save them in 1 s. Worst case path: half-open TCP + WS library spinlock ⇒ no commands ⇒ staleness ramp zeros motors at 200 ms (OK), but the producer never restarts and the ws_prod task is wedged for 5 s before panic. This deviates from the documented invariant in `00-CONTEXT.md §2` (\"TWDT timeout is 1 s\").",
  "suggested_fix": "Move `esp_task_wdt_init(TWDT_TIMEOUT_S, true)` to be the *first* action in `setup()` after `Serial.begin()`, before any `xTaskCreatePinnedToCore` call. The two tasks then both subscribe under a known 1 s timeout.",
  "test_gap": "TWDT timing is intrinsically untestable in `[env:native]`; no integration test verifies `esp_task_wdt_status()` after each task subscribes."
}
```

```json
{
  "id": "C3-P0-04",
  "title": "OTA flash starts even if `g_producer->stop()` times out — motors not actively driven to zero",
  "file": "src/main_robot.cpp",
  "line_range": "104-110",
  "quote": "ArduinoOTA.onStart([]() {\n        // Stop accepting new commands. Once the producer is gone, the staleness\n        // ramp in controlTask will drive motors to zero within 200 ms before\n        // the firmware actually overwrites flash.\n        Serial.println(\"[ota] update starting; halting teleop\");\n        if (g_producer) g_producer->stop();\n    });",
  "risk": "(a) The `onStart` handler does **not** call `g_drivetrain.stop()` or `g_controller->stop()`. It relies on the staleness ramp inside `controlTask` to zero motors *while* OTA is overwriting flash. The control task continues running PWM during the ~10–30 s flash window — and `OmniDrivetrain::update(dt)` keeps ticking the PIDs every 10 ms even after staleness scales the setpoint to zero (see `00-CONTEXT.md §6`, \"PIDs keep running, motors don't actively brake\"). If the OTA library suspends Core 1 tasks mid-flash, the last PWM duty latches and the wheels coast — usually fine, but if the staleness ramp had not yet completed (callback fires at t=0, flash starts immediately), motors are still driven at e.g. 80% for up to 200 ms before the ramp completes — but the OTA handler does not wait. (b) `g_producer->stop()` blocks up to 1 s on `_exitSemaphore`. This callback runs on the Arduino loopTask (Core 1, prio 1); during that 1 s, `ArduinoOTA.handle()` is not pumping subsequent OTA packets. Combined with the next packet retry path in `ArduinoOTA`, this risks an OTA session timeout.",
  "suggested_fix": "In `ArduinoOTA.onStart`: (1) `g_drivetrain.stop()` to actively brake every motor, (2) suspend `controlTask` via `vTaskSuspend(controlTaskHandle)` to prevent the PIDs from re-energising motors, (3) call `g_producer->stop()` with a shorter timeout or async. Store the controlTask handle (currently passed `nullptr` to `xTaskCreatePinnedToCore`).",
  "test_gap": "No integration test for the OTA shutdown path; `PassthroughDrivetrainController::stop()` is exercised by `test_passthrough_controller.cpp:67`, but the OTA wiring is not."
}
```

## P1

```json
{
  "id": "C3-P1-01",
  "title": "Real WiFi/OTA credentials in plaintext working tree (gitignored but present on dev disk)",
  "file": "src/config/wifi_credentials.h",
  "line_range": "8-10",
  "quote": "#define WIFI_SSID     ",
  "risk": "`.gitignore` correctly ignores `src/config/wifi_credentials.h` and `git ls-files` confirms it is NOT tracked at f8ca95f — so this is not a public secrets leak. However: the file does contain a live SSID/password and the OTA password (a 3-character string, see `00-CONTEXT.md §9`). Risks: (1) trivial OTA password lets anyone on the trusted LAN re-flash the robot, (2) the file can be added by `git add -A` or by an agent run by mistake — there is no pre-commit hook enforcing the gitignore, (3) it is committed to *this developer's* disk in plaintext where any other process can read it. Severity-wise this is P1 on a private repo and would jump to P0 the moment someone runs `git add -A`. The CONTEXT doc §9 incorrectly described this file as \"checked in\" — it is not at this SHA, but the .h file's own header comment (\"This file is gitignored. Do NOT commit.\") and `wifi_credentials.example.h` both signal the same risk.",
  "suggested_fix": "(a) Rotate the OTA password to ≥16 random chars and use the same env-var pattern already documented in `wifi_credentials.example.h`. (b) Move WIFI_PASSWORD to the same env-var mechanism (compile-time `-DWIFI_PASSWORD=\"...\"`) so the working-tree file no longer contains secrets. (c) Add a pre-commit hook that fails if `src/config/wifi_credentials.h` is staged.",
  "test_gap": "n/a — operational/process gap."
}
```

```json
{
  "id": "C3-P1-02",
  "title": "Arduino `loop()` (loopTask) is not subscribed to TWDT — silent hang of WiFi reboot watchdog possible",
  "file": "src/main_robot.cpp",
  "line_range": "252-264",
  "quote": "void loop() {\n    // Housekeeping: OTA poll + WiFi reboot watchdog. Control runs in dedicated\n    // tasks. Tick at 10 Hz so OTA initiate requests are answered promptly.\n    ArduinoOTA.handle();",
  "risk": "Only `controlTask` (main_robot.cpp:148) and ws_prod (WebSocketCommandProducer.cpp:103) call `esp_task_wdt_add(NULL)`. The Arduino loopTask runs `ArduinoOTA.handle()` and the WiFi-offline reboot watchdog (lines 257-262). If `ArduinoOTA.handle()` ever blocks indefinitely (e.g. malformed packet handling in the library), the 30 s WiFi-offline reboot watchdog also stops firing — the very mechanism CONTEXT §6 lists as the recovery path for mid-run WiFi loss. The control task keeps running on staleness ramp ⇒ motors are safe, but the box is permanently offline until a hard power cycle.",
  "suggested_fix": "Either subscribe loopTask to TWDT and reset inside `loop()` (the 100 ms `vTaskDelay` gives ample slack against a 1 s timeout), OR replace the WiFi reboot watchdog with an independent `esp_timer` that calls `ESP.restart()` based on `g_lastWifiConnectedMs`.",
  "test_gap": "No test for the WiFi-offline reboot path."
}
```

```json
{
  "id": "C3-P1-03",
  "title": "WiFi event callback writes `g_lastWifiConnectedMs` non-atomically across cores",
  "file": "src/main_robot.cpp",
  "line_range": "83-98",
  "quote": "volatile uint32_t g_lastWifiConnectedMs = 0;\n\nvoid onWifiEvent(WiFiEvent_t event) {",
  "risk": "`g_lastWifiConnectedMs` is `volatile uint32_t` (not `std::atomic`). It is written in two contexts: (a) `onWifiEvent` which arduino-esp32 dispatches on its own event task (Core 0), and (b) `loop()` (Core 1) at line 258. It is read in `loop()` at line 259. `volatile` on a 32-bit aligned word is *probably* atomic on xtensa, but the C++ standard doesn't guarantee it — and the read-modify-write at line 259 (`millis() - g_lastWifiConnectedMs > WIFI_OFFLINE_REBOOT_MS`) can see a torn or stale value. Worst case: the comparison wraps and triggers a spurious `ESP.restart()`. Probability is low but the consequence is a forced reboot mid-teleop.",
  "suggested_fix": "Declare `std::atomic<uint32_t> g_lastWifiConnectedMs{0}`; access with `.load(std::memory_order_relaxed)` and `.store(...)`.",
  "test_gap": "Cannot be tested in `[env:native]`; integration test would need WiFi event injection."
}
```

```json
{
  "id": "C3-P1-04",
  "title": "Pin map docs disagree with `pins.h` after the harness remap (f8ca95f)",
  "file": "docs/motor-spin-test-procedure.md",
  "line_range": "16-30",
  "quote": "| 2     | B             | GPIO 18   | GPIO 19   |",
  "risk": "`docs/motor-spin-test-procedure.md` line 18 lists motor 2 at GPIO 18/19; `src/config/pins.h:14` has `MOTOR2_PINS = {.fwd = 25, .rev = 26}`. Encoder 1 in docs (line 29) is `A=GPIO 4, B=GPIO 2`; `pins.h:17` has `A=2, B=4` (swapped). Encoder 2 in docs (line 30) is `A=GPIO 39, B=GPIO 36`; `pins.h:18` has `A=36, B=39` (also swapped). Separately, `docs/imu-test-procedure.md:15-16` lists IMU `SDA=GPIO 26, SCL=GPIO 25` but `main_robot.cpp:53-54` and `main_imu_test.cpp:37` use SDA=21, SCL=22. The motor 2/encoder swaps will cause anyone following the test procedure to wire the harness incorrectly and (a) blow IN3/IN4 of the H-bridge by tying GPIO 18 to a 5 V line that's now-orphaned (P1 hardware risk), or (b) get reversed-direction motors that the new closed-loop PID will run away from. `main_blink.cpp:3` still hard-codes `{14, 27, 16, 17, 18, 19}` — also stale for motor 2.",
  "suggested_fix": "Update all three docs (motor-spin, drivetrain-test, imu-test) to match `pins.h`. Replace the hardcoded pin array in `main_blink.cpp` with `MOTOR0_PINS`/`MOTOR1_PINS`/`MOTOR2_PINS` from `pins.h` so blink can't drift.",
  "test_gap": "n/a (doc drift)."
}
```

```json
{
  "id": "C3-P1-05",
  "title": "Asserts guard invariants that have NO runtime check on hardware (none of robot/motor_test/drivetrain_test set -DBB8_DEBUG)",
  "file": "src/config/debug.h",
  "line_range": "25-27",
  "quote": "#else\n#define BB8_ASSERT(cond, msg) ((void)0)\n#endif",
  "risk": "Only `[env:native]` sets `-DBB8_DEBUG` (`platformio.ini:83`). In `[env:robot]` and friends, `BB8_ASSERT` is `((void)0)`. The invariants thereby unprotected on production hardware include: (a) `L298NDriver::setOutput`'s `[-1,1]` range check — if any caller passes a NaN or out-of-range float, `static_cast<uint8_t>(std::abs(value) * 255.0f)` wraps modulo-256, so 1.001 → 0 PWM (silent stop), 2.0 → 254 PWM (silent near-full output). PID output clamping (PID.h:47-48) prevents NaN propagation in normal operation but the staleness scale at main_robot.cpp:177 multiplies by `last_known` — if `last_known.vx` is corrupted (e.g. partial write via non-cross-core-safe path, see C3-P1-06), the IK might emit NaN. (b) `PID::compute`'s finite/dt-non-negative checks — silently passes NaN through to `motor->setSpeed` ⇒ `L298NDriver::setOutput(NaN)`. (c) `BNO055IMU::read()` called before `begin()` — undefined Adafruit_BNO055 behaviour. (d) `BNO055IMU::begin()` called twice — leaks NVS handles.",
  "suggested_fix": "Either define a separate `BB8_RUNTIME_CHECK(cond, action)` that, on hardware, clamps inputs and increments a fault counter instead of asserting; OR enable `-DBB8_DEBUG` for `[env:robot]` (the abort handler would still be safer than a runaway motor). Specifically: in `L298NDriver::setOutput`, runtime-clamp `value` to `[-1,1]` and reject NaN before the static_cast.",
  "test_gap": "`test_moving_average.cpp:87` `test_nan_propagation` proves the filter passes NaN through, but no test confirms that downstream `L298NDriver::setOutput(NaN)` is rejected."
}
```

```json
{
  "id": "C3-P1-06",
  "title": "`MovingAverage<N>` is not safe for concurrent push/read across cores; underflow on first samples",
  "file": "src/util/MovingAverage.h",
  "line_range": "16-29",
  "quote": "void push(float value) {\n        _buffer[_index] = value;\n        _index = (_index + 1) % N;\n        if (_count < N) ++_count;\n    }\n\n    float average() const {",
  "risk": "Two issues. (1) Concurrency: `_buffer[_index] = value` then `_index = ...` then `_count = ...` is three independent writes on plain ints/floats. `average()` reads `_count` then iterates `_buffer[0..count)`. If a reader on Core A interleaves with a writer on Core B, it can read `_count==N` but a `_buffer[i]` that is in the middle of being written, returning a partial-write float — typically benign (NaN-free) on aligned 32-bit ops, but undefined behaviour per the C++ memory model. CONTEXT §2 and §9 note current usage is single-core (always touched by the control task), so this is *latent*, but the type is `public` and the class advertises no thread-safety constraint. (2) Underflow path: with `_count == 0`, `average()` returns 0.0f (line 23) — but `_count == 1` returns the single sample. Used as `getFilteredRPM()` measurement input to the PID at boot, the first call returns 0 RPM for any setpoint, so a step-response from rest immediately drives PWM up because the PID believes the wheel isn't moving (CONTEXT §1 specifies `MovingAverage<5>`). This is the existing behaviour but the test suite quietly endorses it (`test_partial_fill`, `test_single_value`).",
  "suggested_fix": "(1) Add a header comment + `static_assert` or runtime doc that `MovingAverage<N>` is single-threaded — and consider adding a `noexcept` + `portMUX_TYPE` variant for cross-core use. (2) Provide an explicit `seed(value)` or `prefill(value)` method so callers (FIT0186Motor::begin) can prime the filter with zero (already the default) or with the first measured sample to avoid the cold-start window where PID sees 0 RPM.",
  "test_gap": "`test_moving_average.cpp` has 12 cases but none exercise: concurrent push/read, push from ISR context, alignment guarantees, or prefill semantics. Quote: `test_partial_fill` (line 79) — covers the cold-start case but doesn't assert it's the *desired* behaviour."
}
```

```json
{
  "id": "C3-P1-07",
  "title": "BNO055 `begin()` failure path is an unbounded `ESP.restart()` boot loop with no backoff",
  "file": "src/main_robot.cpp",
  "line_range": "221-224",
  "quote": "if (!g_imu.begin()) {\n        Serial.println(\"FATAL: BNO055 init failed — restarting.\");\n        ESP.restart();\n    }",
  "risk": "Same applies to `connectWifi()`'s 30 s timeout → restart at line 134. If the IMU's I2C bus is permanently stuck (e.g. the BNO055 is unpowered or a wire is cut), the board enters a tight 200 ms reboot loop: `Serial.begin` + `delay(200)` + `Wire.begin` + 3× motor.begin + `g_imu.begin()` (probably a multi-second blocking timeout inside Adafruit_BNO055) + restart. Across hours this burns flash erase/program cycles on the Preferences partition every time the eventual NVS save runs, and the constant reboot prevents OTA recovery — once you bake a firmware that can't talk to its IMU you must physically re-flash. The boot-loop is also the worst possible operator UX: WiFi never comes up so there's no remote diagnostic path.",
  "suggested_fix": "Implement a degraded mode: if `g_imu.begin()` fails N times consecutively (track via RTC slow-mem counter), still bring up WiFi + WebSocket + OTA so the operator can re-flash. Run the control task with a zero IMUReading (passthrough controller ignores IMU anyway). Alternatively bound the boot-loop to 5 retries before entering deep-sleep with a recovery beacon.",
  "test_gap": "No test for boot-loop bounding; IMU begin failure path is hardware-only."
}
```

```json
{
  "id": "C3-P1-08",
  "title": "PassthroughDrivetrainController::stop() exists but is never wired to OTA, WS disconnect, or IMU loss",
  "file": "src/main_robot.cpp",
  "line_range": "104-110",
  "quote": "if (g_producer) g_producer->stop();",
  "risk": "CONTEXT §9 #11 already calls this out as an observation; for safety lens it bumps to P1. `PassthroughDrivetrainController::stop()` would call `drivetrain.stop()` which actively brakes motors and resets PIDs (CONTEXT §6). Today every shutdown path (OTA, staleness, WS disconnect) zeroes the *setpoint* and lets PIDs settle. At zero setpoint with `deadband=5 RPM`, PID returns 0 once measurement is also < 5 RPM — so the motors coast from current speed instead of braking. On a tilted floor this means continued rolling under OTA. Combined with C3-P0-04, OTA flash time + coast roll = uncontrolled motion.",
  "suggested_fix": "OTA `onStart`: call `g_controller->stop()` *before* `g_producer->stop()`. Add a `safetyStop()` entry point on the controller that also re-enables brake mode.",
  "test_gap": "`test_passthrough_controller.cpp:67-70` `test_stop_calls_drivetrain_stop` confirms the wiring of stop() but no integration test confirms any caller invokes it."
}
```

```json
{
  "id": "C3-P1-09",
  "title": "Dead entrypoints `main_original.cpp` + `main_blink.cpp` are present in src/ and rely on exclude-lists in every env",
  "file": "platformio.ini",
  "line_range": "7-47",
  "quote": "-<main_original.cpp> -<main_blink.cpp>",
  "risk": "Every env's `build_src_filter` explicitly excludes both files. The base env `[env:wemos_d1_uno32]` is the only one without all five test mains excluded — it has no `setup()`/`loop()` to link, so it would fail at link time and that fact is the only line of defense against accidental motor-spin builds. If a new contributor copies an env without `-<main_blink.cpp>` they get a board that runs `analogWrite(pins[i], 128)` on motor 0 (forward at ~50% PWM) immediately on boot. `main_blink.cpp:16` hard-codes the *old* pin map (`{14, 27, 16, 17, 18, 19}`), so motor 2's pins (18, 19) are now disconnected — but motor 0 (14, 27) and motor 1 (16, 17) would still spin at 50% on every boot until the operator notices.",
  "suggested_fix": "Move `main_blink.cpp` and `main_original.cpp` to `tools/` or `src/_archive/` outside the default `src/` filter scope. Or wrap their `setup()/loop()` in `#ifdef BB8_BLINK_TEST` so an accidental include is harmless.",
  "test_gap": "No build smoke test verifies that `[env:robot]` doesn't pull in `main_blink.cpp`."
}
```

```json
{
  "id": "C3-P1-10",
  "title": "Heap-allocated globals (`g_latch`, `g_producer`, `g_controller`) are nullable but dereferenced unconditionally in controlTask",
  "file": "src/main_robot.cpp",
  "line_range": "79-82",
  "quote": "CommandLatch<BodyVelocity>*      g_latch      = nullptr;\n    WebSocketCommandProducer*        g_producer   = nullptr;\n    PassthroughDrivetrainController* g_controller = nullptr;",
  "risk": "`controlTask` (line 162: `auto fresh = g_latch->read();`, line 187: `g_controller->update(...)`) dereferences these without a null check. They're set on the same task that creates `controlTask` *before* the task is created, so this is safe TODAY — but the pattern is fragile. If a future refactor moves task creation earlier (e.g. to fix C3-P0-02), the controlTask will null-deref on first iteration. There is also no memory barrier: ESP32 doesn't reorder these stores aggressively, but C++ doesn't guarantee that across a `xTaskCreatePinnedToCore` call. (Cf. CONTEXT §5 step 7-10.)",
  "suggested_fix": "Either make them static (not heap) and construct in `setup()` via placement-new, OR add `if (!g_latch || !g_controller) { vTaskDelay(...); continue; }` at the top of the loop, OR pass pointers as the `void* arg` to `xTaskCreatePinnedToCore`.",
  "test_gap": "n/a (lifecycle invariant)."
}
```

## P2

```json
{
  "id": "C3-P2-01",
  "title": "I2C pin documented in code comment is stale",
  "file": "src/main_robot.cpp",
  "line_range": "52-54",
  "quote": "// I2C pins for BNO055 (matches main_imu_test.cpp)\nconstexpr int I2C_SDA = 21;\nconstexpr int I2C_SCL = 22;",
  "risk": "Comment claims parity with `main_imu_test.cpp` — true (both use 21/22), but `docs/imu-test-procedure.md:15-16` documents SDA=26, SCL=25 for the same sensor. See C3-P1-04 above.",
  "suggested_fix": "Move I2C pins into `pins.h` alongside motor/encoder pins so all hardware mapping is in one place; reference from both `main_*` files.",
  "test_gap": "n/a."
}
```

```json
{
  "id": "C3-P2-02",
  "title": "Lifecycle interface has no started()/state query; double-start is a silent no-op, double-stop is benign",
  "file": "src/util/Lifecycle.h",
  "line_range": "13-24",
  "quote": "class Lifecycle {\npublic:\n    virtual ~Lifecycle() = default;\n\n    // Bring the component online: start any FreeRTOS task, open sockets, etc.\n    virtual void start() = 0;\n\n    // Bring the component offline. Must be synchronous: caller can rely on\n    // the task having exited and resources being released by the time\n    // stop() returns.\n    virtual void stop() = 0;",
  "risk": "No `bool running() const` and no explicit state machine. `WebSocketCommandProducer::start()` (`.cpp:42-43`) silently returns if already running; `stop()` (`.cpp:68-69`) silently returns if not running. The OTA path calls `stop()` and then OTA reboots, so we never re-`start()` — but a test or future caller that issues `stop(); start();` will hit a fresh task with the same `_latch` and may surprise the consumer with a duplicate state. The interface is sound (every state has a transition) but provides no introspection — a consumer like `controlTask` can't ask \"is my producer alive?\" without checking `connected()` which lies on half-open TCP (per main_robot.cpp:170-172 comment).",
  "suggested_fix": "Add `virtual bool running() const = 0;` to `Lifecycle`. Define explicit states {Stopped, Starting, Running, Stopping} or document the implicit state model in the interface header.",
  "test_gap": "No tests for `Lifecycle` invariants (double-start, restart-after-stop, stop-before-start)."
}
```

```json
{
  "id": "C3-P2-03",
  "title": "MovingAverage::reset() touches `_buffer = {}` which compiles to `memset` — fine, but `average()` after reset returns 0.0f regardless of any prior calibration",
  "file": "src/util/MovingAverage.h",
  "line_range": "31-35",
  "quote": "void reset() {\n        _buffer = {};\n        _index = 0;\n        _count = 0;\n    }",
  "risk": "Tested by `test_reset` (line 33) and `test_push_after_reset` (line 49). No reset() consumer exists in the production tree — `FIT0186Motor` never calls it after construction. The fact that reset() exists implies a use case (PID re-init?) that is currently absent. Low risk, just code surface.",
  "suggested_fix": "Either remove `reset()` if unused, or document the intended call site.",
  "test_gap": "n/a (covered)."
}
```

```json
{
  "id": "C3-P2-04",
  "title": "POD util types lack equality and stream ops — debug printing requires manual field-by-field printf",
  "file": "src/util/Vec3.h",
  "line_range": "8-12",
  "quote": "struct Vec3 {\n    float x = 0;\n    float y = 0;\n    float z = 0;\n};",
  "risk": "Same for `Quat`, `Euler`, `CalStatus`. None of these have `operator==` or `operator<<`. Test code (`test_passthrough_controller.cpp:62-64`) uses field-by-field `TEST_ASSERT_EQUAL_FLOAT`. Not a bug; just friction. CONTEXT §4 documents the conventions; the types lack docstring comments tying field names to those conventions.",
  "suggested_fix": "Add brief Doxygen-style comments to each field stating units and frame (e.g. `Vec3::x // m/s, body x-axis forward`). Optional: free-function `operator==` with epsilon tolerance for tests.",
  "test_gap": "Tests rely on per-field assertions, which is fine — flag is cosmetic."
}
```

```json
{
  "id": "C3-P2-05",
  "title": "Test coverage gap: no `controlTask` unit test for the staleness ramp, no `setup()` order test",
  "file": "test/test_passthrough_controller/test_passthrough_controller.cpp",
  "line_range": "57-70",
  "quote": "void test_update_forwards_command_to_drivetrain() {",
  "risk": "The two test cases (`test_update_forwards_command_to_drivetrain`, `test_stop_calls_drivetrain_stop`) cover the pure passthrough — they don't exercise: (a) staleness ramp math at age=0/100/199/200 ms, (b) `have_seen_fresh=false` cold-start, (c) staleness scale applied to `omega` and `vx` identically (per CONTEXT §9 #9), (d) what happens when `g_latch->read()` returns `nullopt` for N consecutive ticks. The ramp logic lives at main_robot.cpp:160-183 inline in `controlTask` — it cannot be unit-tested in its current location.",
  "suggested_fix": "Extract the staleness logic into a `StalenessRamp` helper class in `util/` with deterministic timestamps and exhaustive native tests. Then `controlTask` becomes a thin shell around it.",
  "test_gap": "Explicitly: `test_staleness_ramp_zero_to_full`, `test_staleness_ramp_hard_zero_after_200ms`, `test_staleness_ramp_before_first_fresh` — all missing."
}
```

## Observations

- `Vec3`, `Quat`, `Euler`, `CalStatus` are pure PODs with sensible default-init values (Quat defaults to identity). `Euler` orders fields `{heading, roll, pitch}` which differs from the more common `{roll, pitch, yaw}` convention — CONTEXT §4 documents this but it is an alignment trap for anyone copying readings to a different IMU library.
- `MovingAverage<N>` correctly `static_assert`s on `N > 0` and uses `std::array` for stack allocation. Window-size-one case is tested (`test_window_size_one`).
- `Lifecycle` is properly virtual-dtor'd; both `MockCommandProducer` and `WebSocketCommandProducer` inherit it cleanly. The interface separation from `CommandProducer` (transport-health half) is good design — polled producers don't pay the lifecycle cost.
- Boot sequence in `main_robot.cpp` is largely well-ordered: Serial first, Wire next, then motor pins, IMU, WiFi. The heap-allocation of `g_latch`/`g_producer`/`g_controller` after FreeRTOS is up (line 228-230) is correct — solves the static-init-vs-FreeRTOS-scheduler problem documented in CONTEXT §5 step 6.
- `WiFi.setAutoReconnect(true)` (line 125) combined with the loopTask 30 s reboot watchdog is the right policy — the control loop is *intentionally* decoupled from WiFi (commands come from the latch, which keeps last value indefinitely; staleness ramp owns the safety net at 200 ms).
- TWDT init at `esp_task_wdt_init(1, true)` with `panic=true` is the right call — silent watchdog reset would be worse than a panic. The 1 s window is comfortably above the 100 Hz control loop's 10 ms period (100× margin).
- `wifi_credentials.example.h` is committed and well-documented; pre-commit hook would be the natural next step (see C3-P1-01).
- `BB8_ASSERT` gated only by `-DBB8_DEBUG` (active only in `[env:native]`) is a documented decision but interacts with too many runtime invariants (see C3-P1-05).
- `g_lastWifiConnectedMs` is written from `setup()` flow before the WiFi event callback can fire, but the initial value (0) means `millis() - 0 > 30000` is false for the first 30 s, then true forever — i.e. there is a single window at exactly t=30 s where the device reboots if no IP has been obtained. This is fine but worth documenting.
- The OTA `onStart` callback's call to `g_producer->stop()` blocks up to 1 s; combined with the OTA library's expected acknowledgement timing, this is a tight budget but observed-working in practice.
- `IMUField operator|` returns `IMUField` but `operator&` returns `bool` (CONTEXT §9 #4) — Cell 2 or Cell 5 will likely cover, noted for cross-reference.
- `main_imu_test.cpp` is the only entrypoint that proactively kills motor pins on boot. This is the right pattern; `main_robot.cpp` should adopt it (see C3-P0-01).

## Top 3 P0 summary

- C3-P0-01: motor pins float / retain prior PWM through the ~30 s gap from boot to controlTask start; `main_imu_test.cpp`'s `killMotorPins()` is missing from `main_robot.cpp`.
- C3-P0-02: motors armed for up to 30 s during WiFi connect before the staleness-ramp control task exists — the staleness safety net only kicks in *after* `xTaskCreatePinnedToCore` at line 240.
- C3-P0-03: ws_prod subscribes to TWDT before `esp_task_wdt_init(1s)` runs, so its watchdog window is the IDF default (~5 s) rather than the documented 1 s.
