# Cell 9 — Code Quality & Architecture (Glue & Lifecycle)

**Scope:** `src/util/*.h` (excl. sync/), `src/config/*.h`, all `src/main_*.cpp`, `platformio.ini`
**Anchor commit:** `f8ca95f`

```json
{
  "cell": 9,
  "lens": "Code Quality & Architecture",
  "scope": "Glue & Lifecycle (util/ non-sync, config/, main_*.cpp, platformio.ini)",
  "findings": [
    {
      "id": "C9-P1-01",
      "severity": "P1",
      "title": "Dead source files (main_blink.cpp, main_original.cpp) are excluded from every env but still live in src/",
      "file": "code/body-firmware/src/main_original.cpp",
      "quote": " * BB-8 Body Firmware — Motor Spin Test\n * Serial input: \"<motor> <speed>\" e.g. \"0 0.5\" or \"1 -1.0\"\n * \"stop\" to brake all motors",
      "also_quote": "build_src_filter = +<*> -<motor/driver/BTS7960Driver.cpp> -<motor/FIT0186Motor.h> -<main_original.cpp> -<main_blink.cpp> -<main_motor_test.cpp> -<main_imu_test.cpp> -<main_robot.cpp> -<input/WebSocketCommandProducer.cpp>",
      "why": "main_original.cpp is a strict subset of main_motor_test.cpp (no encoders, no filtering) and is excluded from all 5 build envs. main_blink.cpp still references pin 18/19 which were removed in the harness remap (f8ca95f) and would mis-fire if anyone tried to build it. Both files are reachable to grep / IDE jump-to-symbol and they imply alternatives that no longer exist. Per global user instructions (no backwards-compat shims for unused things), they should be deleted. main_blink.cpp is additionally misleading: `const uint8_t pins[] = {14, 27, 16, 17, 18, 19};` references a pin layout that no longer exists in pins.h (motor 2 is now 25/26).",
      "evidence": "main_blink.cpp:3 `const uint8_t pins[] = {14, 27, 16, 17, 18, 19};` vs pins.h:14 `inline constexpr L298NPins MOTOR2_PINS = {.fwd = 25, .rev = 26};`. Every [env:*] in platformio.ini lists `-<main_original.cpp> -<main_blink.cpp>` in its src filter.",
      "recommendation": "Delete main_blink.cpp and main_original.cpp. Also drop the corresponding `-<main_original.cpp> -<main_blink.cpp>` exclusion tokens from all 5 envs in platformio.ini. If a smoke-test is wanted later, write a fresh one against current pins."
    },
    {
      "id": "C9-P1-02",
      "severity": "P1",
      "title": "CONTEXT.md §9.1 is factually wrong about wifi_credentials.h being committed — but the file's own header markers are misleading",
      "file": "code/body-firmware/src/config/wifi_credentials.h",
      "quote": " * BB-8 Body Firmware — local credentials.\n * This file is gitignored. Do NOT commit.",
      "why": "I verified: `git check-ignore -v` reports `code/body-firmware/.gitignore:8:src/config/wifi_credentials.h` and `git ls-files code/body-firmware/src/config/` returns only debug.h, pins.h, wifi_credentials.example.h. The file is *not* tracked. The structural setup is sound (gitignore + .example.h template). However, the CONTEXT.md note (`§9.1 ... is checked in (visible in git ls-files and the file's own header says \"gitignored. Do NOT commit.\")`) contradicts the working tree state and will mislead downstream cells. The risk is *future*: nothing in the source structure prevents an agent from `git add -f`-ing the file or from a tooling change ignoring the .gitignore. Two structural hardenings: (a) keep the existing comment marker as-is (good), (b) consider a CI grep for the literal SSID/OTA macro values landing in a tracked file.",
      "recommendation": "Tell synthesizer cell to correct CONTEXT.md §9.1 (file is correctly gitignored, untracked, current at f8ca95f). No source change needed beyond that. Do NOT relocate credentials into platformio.ini build_flags — the `${sysenv.OTA_PASSWORD}` pattern already used for OTA upload is the right precedent, and forcing WIFI_SSID/PASSWORD through env vars at compile time would be a larger workflow change worth a separate ticket."
    },
    {
      "id": "C9-P2-03",
      "severity": "P2",
      "title": "main_robot.cpp and main_drivetrain_test.cpp duplicate the entire motor/PID/drivetrain bring-up verbatim",
      "file": "code/body-firmware/src/main_robot.cpp",
      "quote": "L298NDriver g_driver0(MOTOR0_PINS);\nL298NDriver g_driver1(MOTOR1_PINS);\nL298NDriver g_driver2(MOTOR2_PINS);\n\nFIT0186Motor<5> g_motor0(g_driver0, MOTOR0_ENCODER);\nFIT0186Motor<5> g_motor1(g_driver1, MOTOR1_ENCODER);\nFIT0186Motor<5> g_motor2(g_driver2, MOTOR2_ENCODER);\n\nPID g_pid0(0.005f, 0.002f, 0.0005f, -1.0f, 1.0f, 5.0f);\nPID g_pid1(0.005f, 0.002f, 0.0005f, -1.0f, 1.0f, 5.0f);\nPID g_pid2(0.005f, 0.002f, 0.0005f, -1.0f, 1.0f, 5.0f);\n\nOmniDrivetrain g_drivetrain(g_motor0, g_motor1, g_motor2, g_config, g_pid0, g_pid1, g_pid2);",
      "also_quote": "static L298NDriver driver0(MOTOR0_PINS);\nstatic L298NDriver driver1(MOTOR1_PINS);\nstatic L298NDriver driver2(MOTOR2_PINS);\n\nstatic FIT0186Motor<5> motor0(driver0, MOTOR0_ENCODER);\nstatic FIT0186Motor<5> motor1(driver1, MOTOR1_ENCODER);\nstatic FIT0186Motor<5> motor2(driver2, MOTOR2_ENCODER);\n\nstatic PID pid0(0.005f, 0.002f, 0.0005f, -1.0f, 1.0f, 5.0f);\nstatic PID pid1(0.005f, 0.002f, 0.0005f, -1.0f, 1.0f, 5.0f);\nstatic PID pid2(0.005f, 0.002f, 0.0005f, -1.0f, 1.0f, 5.0f);",
      "why": "Three-motor + three-PID + drivetrain construction with identical PID gains (Kp=0.005, Ki=0.002, Kd=0.0005, deadband=5) is duplicated bit-for-bit between main_robot.cpp:58-70 and main_drivetrain_test.cpp:14-26. If a wheel index is mismapped (a real risk per Cell 8 territory) or a gain is retuned in one place and not the other, the test and prod diverge. This is the highest-yield DRY opportunity in scope: a header `src/drivetrain/RobotAssembly.h` could expose `makeOmniDrivetrain()` returning the bound object plus references to its sub-parts. Not a P1 because the duplication is currently consistent.",
      "recommendation": "Extract a header-only `RobotAssembly.h` that materializes the canonical 3-motor + 3-PID + OmniDrivetrain stack (as static locals or out-params). Both mains call it; PID gain table lives once in `RobotConstants.h`. Keep Serial init, Wire.begin, and main-specific globals (latch/producer/controller for prod; REPL state for test) inline — they differ legitimately."
    },
    {
      "id": "C9-P2-04",
      "severity": "P2",
      "title": "Test mains use ad-hoc String.indexOf parsing instead of CommandFrameParser",
      "file": "code/body-firmware/src/main_drivetrain_test.cpp",
      "quote": "    int firstSpace = line.indexOf(' ');\n    int secondSpace = line.indexOf(' ', firstSpace + 1);\n\n    if (firstSpace < 0 || secondSpace < 0) {\n        Serial.println(\"ERR: format is <vx> <vy> <omega_deg>\");\n        return;\n    }\n\n    float vx = line.substring(0, firstSpace).toFloat();\n    float vy = line.substring(firstSpace + 1, secondSpace).toFloat();\n    float omegaDeg = line.substring(secondSpace + 1).toFloat();",
      "why": "main_drivetrain_test.cpp takes `<vx> <vy> <omega_deg>` and parses it manually; the codebase already has `src/input/CommandFrameParser.h` that parses a `\"vx,vy,omega\"` text frame for the WS path. Different delimiter, but the validation logic (numeric-range, NaN handling) lives in only one of them. Not a quality fix worth contorting either for, but worth a one-line note: the test REPL's parser silently accepts garbage (`toFloat()` returns 0 on parse failure, which means typing `stop now` would silently issue zero command rather than reject).",
      "recommendation": "Either accept the divergence (test tools have looser specs — fine) or add a tiny `parseSpaceDelimitedTriple()` helper near CommandFrameParser. Low priority."
    },
    {
      "id": "C9-P2-05",
      "severity": "P2",
      "title": "main_imu_test.cpp owns motor-pin teardown logic that should live with the motor module",
      "file": "code/body-firmware/src/main_imu_test.cpp",
      "quote": "static void killMotorPins() {\n    constexpr uint8_t pins[] = {\n        MOTOR0_PINS.fwd, MOTOR0_PINS.rev,\n        MOTOR1_PINS.fwd, MOTOR1_PINS.rev,\n        MOTOR2_PINS.fwd, MOTOR2_PINS.rev,\n    };\n    for (auto p : pins) {\n        pinMode(p, OUTPUT);\n        digitalWrite(p, LOW);\n    }\n}",
      "why": "The IMU test main explicitly drives all H-bridge pins LOW at boot — sensible (the IMU is on the same board, hardware-IDed) — but the knowledge of \"how to put L298N driver pins in coast\" is leaking out of `src/motor/driver/` into a main. If the driver gains an Enable pin or pin layout changes, this list silently goes stale. A `L298NDriver::safeState(pins)` static (or a free function in pins.h) would localize the contract.",
      "recommendation": "Hoist into either `L298NDriver::idleAllPins(const L298NPins&)` static, or a `motors_idle_all_pins()` helper next to MOTOR{0..2}_PINS in pins.h. Keep the call site in main_imu_test.cpp."
    },
    {
      "id": "C9-P2-06",
      "severity": "P2",
      "title": "BB8_ASSERT is a no-op on every flashed env — silent runtime gap",
      "file": "code/body-firmware/src/config/debug.h",
      "quote": "#ifdef BB8_DEBUG\n\n#ifndef BB8_ASSERT_HANDLER\n#define BB8_ASSERT_HANDLER(msg, file, line) do { \\\n    Log.fatal(\"ASSERT FAILED: %s (%s:%d)\" CR, msg, file, line); \\\n    abort(); \\\n} while(0)\n#endif\n\n#define BB8_ASSERT(cond, msg) \\\n    do { \\\n        if (!(cond)) { \\\n            BB8_ASSERT_HANDLER(msg, __FILE__, __LINE__); \\\n        } \\\n    } while (0)\n#else\n#define BB8_ASSERT(cond, msg) ((void)0)\n#endif",
      "why": "Macro itself is well-formed (do/while wrapper, side-effect-once, file:line capture). The problem is that `-DBB8_DEBUG` is only set in `[env:native]` per CONTEXT.md §7, so every hardware env compiles asserts to `((void)0)`. Bond range checks like `Motor::setSpeed()` and `Driver::setOutput()` accepting `[-1.0,+1.0]` rely on this assert (per §4 invariants). On hardware, a NaN or out-of-range value from a future controller would propagate silently to `analogWrite`. The middle ground — log a warning without aborting on hardware — isn't expressible with this macro. Two options: (a) split into BB8_ASSERT (debug-only, abort) and BB8_CHECK (always-on, log + clamp), or (b) accept the gap and document that production runs without invariant checks.",
      "recommendation": "Add a parallel `BB8_CHECK(cond, msg)` that on hardware logs via ArduinoLog at warning level (no abort) and is no-op in `[env:native]`. Use it for hot-path range guards. Keep BB8_ASSERT for true invariants that should crash in tests."
    },
    {
      "id": "C9-P2-07",
      "severity": "P2",
      "title": "Lifecycle is a 2-method interface used by exactly one implementer — barely earning its keep",
      "file": "code/body-firmware/src/util/Lifecycle.h",
      "quote": "class Lifecycle {\npublic:\n    virtual ~Lifecycle() = default;\n\n    // Bring the component online: start any FreeRTOS task, open sockets, etc.\n    virtual void start() = 0;\n\n    // Bring the component offline. Must be synchronous: caller can rely on\n    // the task having exited and resources being released by the time\n    // stop() returns.\n    virtual void stop() = 0;\n};",
      "why": "The abstraction has exactly one implementation today (`WebSocketCommandProducer`) and exactly one polymorphic use site (`g_producer->stop()` in the OTA start callback, which already has a concrete pointer). The comment on `stop()` documents a non-trivial contract (synchronous, resources released) that the only implementer satisfies via 1 s `_exitSemaphore` timeout with `vTaskDelete` fallback — i.e. the contract is *almost* satisfied. The split from `CommandProducer` (described in CONTEXT.md §1) is reasonable as a design intent — polled producers shouldn't need empty stubs. But until a second `Lifecycle` arrives, the interface is paying a vtable + a separate header cost for no current dispatch benefit. Keep it as a documented convention (\"if you own a FreeRTOS task, name your bring-up `start()`\") rather than a virtual interface, *unless* a second consumer is on the near roadmap (camera stream? audio?). Not a defect — a forward-looking judgment call.",
      "recommendation": "Defer until a second Lifecycle-implementing component is added. At that point the abstraction is fully earned. For now, leave as-is — the cost is small and the doc comment is useful to readers."
    },
    {
      "id": "C9-P2-08",
      "severity": "P2",
      "title": "Vec3 / Quat / Euler / CalStatus are minimal but lack equality / printing — bound to be reinvented",
      "file": "code/body-firmware/src/util/Vec3.h",
      "quote": "struct Vec3 {\n    float x = 0;\n    float y = 0;\n    float z = 0;\n};",
      "why": "All four POD headers are correctly minimal — no premature math overloads, no operator vomit. But: every consumer that wants `==`, `!=`, or pretty-print writes ad-hoc code (e.g. main_imu_test.cpp:62-75 does field-by-field printf). For a 3-float POD, defaulted `operator==` + an inline `Print&` formatter would eliminate ~30 LOC across mains and tests without compromising minimality. Templates are absent (good — these are concrete PODs, not templates over float). MovingAverage<N> is properly templated with a `static_assert(N > 0)`. CalStatus uses `uint8_t` correctly for 0–3 channels. No single-use abstraction found here; all four PODs are used by both BNO055IMU.h and consumers.",
      "recommendation": "Add `inline bool operator==(const Vec3&, const Vec3&) = default;` (C++20) or hand-rolled (C++17) where the compile target is gnu++17. Same for Quat/Euler/CalStatus. Optionally an `operator<<` for Print. Both improvements pay back in test code more than runtime."
    },
    {
      "id": "C9-P2-09",
      "severity": "P2",
      "title": "platformio.ini env sprawl: 5 envs, 4 of them duplicate the same exclusion list with one toggle changed",
      "file": "code/body-firmware/platformio.ini",
      "quote": "[env:drivetrain_test]\nextends = env:wemos_d1_uno32\nbuild_src_filter = +<*> -<motor/driver/BTS7960Driver.cpp> -<main_original.cpp> -<main_blink.cpp> -<main_motor_test.cpp> -<main_imu_test.cpp> -<main_robot.cpp> -<input/WebSocketCommandProducer.cpp> -<util/sync/CommandLatch.cpp>",
      "also_quote": "[env:motor_test]\nextends = env:wemos_d1_uno32\nbuild_src_filter = +<*> -<motor/driver/BTS7960Driver.cpp> -<main_original.cpp> -<main_blink.cpp> -<main_drivetrain_test.cpp> -<main_imu_test.cpp> -<main_robot.cpp> -<input/WebSocketCommandProducer.cpp> -<util/sync/CommandLatch.cpp>",
      "why": "Every test env's filter is the same baseline list (BTS7960Driver, ws producer, CommandLatch.cpp, all unused mains, dead mains) with one `main_*.cpp` flipped on. PlatformIO's `extends` already gets used, but `build_src_filter` is *replaced* not *merged* — so each child env restates the full list. A cleaner pattern: define `[common_test_filter]` as `extra_configs` or just accept the duplication if PIO doesn't support filter inheritance natively (it doesn't, last I checked — that's a real constraint, not a quality issue). The structural problem is more that `-<main_original.cpp> -<main_blink.cpp>` are repeated five times *because the files still exist* — see C9-P1-01. Deleting them removes 5 redundant tokens. Also: `[env:wemos_d1_uno32]` is the base and excludes every main, so it never links — its sole purpose is `extends=`. A `[common]` (non-env) section would be clearer than a fake env, but PIO's syntax for that isn't free either.",
      "recommendation": "(1) Delete main_blink.cpp / main_original.cpp (C9-P1-01) — drops 2 tokens from every env. (2) Rename `[env:wemos_d1_uno32]` to `[base]` with `[base]` invocations changed to `extends = base` if PIO allows — verify with a `pio run -e <each>` smoke. If not, add a comment to wemos_d1_uno32 saying \"abstract base, do not flash directly\"."
    },
    {
      "id": "C9-P2-10",
      "severity": "P2",
      "title": "Inconsistent main file naming — main_original is the lone non-conforming entry",
      "file": "code/body-firmware/src/main_original.cpp",
      "quote": " * BB-8 Body Firmware — Motor Spin Test\n * Serial input: \"<motor> <speed>\" e.g. \"0 0.5\" or \"1 -1.0\"\n * \"stop\" to brake all motors",
      "why": "Naming convention across mains: `main_robot.cpp` (prod), `main_drivetrain_test.cpp`, `main_motor_test.cpp`, `main_imu_test.cpp`, `main_blink.cpp`, `main_original.cpp`. The pattern is `main_<subsystem>_test.cpp` for hardware bring-up checks and `main_<role>.cpp` for everything else. `main_original` doesn't say what it is — the file is a strict subset of `main_motor_test` (no encoders, no filter); it should either be renamed `main_motor_driver_test.cpp` (driver-only, no encoder) or, preferably, deleted (C9-P1-01). `main_blink` is fine as a name but the file is itself dead (P1).",
      "recommendation": "Resolved by C9-P1-01 (delete both). If `main_original` is kept against my recommendation, rename to `main_driver_only_test.cpp`."
    },
    {
      "id": "C9-P2-11",
      "severity": "P2",
      "title": "config/pins.h pulls in L298NDriver.h and FIT0186.h transitively — pins should not know drivers",
      "file": "code/body-firmware/src/config/pins.h",
      "quote": "#include \"L298NDriver.h\"\n#include \"MotorConfig.h\"\n#include \"FIT0186.h\"\n\ninline constexpr L298NPins MOTOR0_PINS = {.fwd = 14, .rev = 27};",
      "why": "`pins.h` lives in `config/` (project-wide constants) but #includes `L298NDriver.h` to get `L298NPins`, and `FIT0186.h` for ENCODER_CPR. That's a layering inversion: the lowest-level config layer now depends on the motor driver implementation. If we ever swap to BTS7960, pins.h has to know. Two cleaner shapes: (a) move `L298NPins` (a pure POD `{fwd, rev}`) into `config/pins.h` or a `motor/MotorPins.h` near `MotorConfig.h`, decoupling the driver class header from the pin type; (b) accept the coupling and rename `pins.h` → `robot_pins.h` and move it next to `RobotConstants.h` in drivetrain/. The current Config split (project-wide `config/`, subsystem `*Config.h`) is otherwise coherent: pins/debug/wifi are truly project-wide; MotorConfig / DrivetrainConfig / IMUReading are subsystem-scoped. Just this one include violates the layering.",
      "recommendation": "Move the `L298NPins {fwd, rev}` POD out of `L298NDriver.h` into either `pins.h` or `motor/MotorConfig.h`. L298NDriver.h then re-includes the POD or aliases it. ENCODER_CPR is okay to import — it's a motor spec constant, not a driver type."
    },
    {
      "id": "C9-P2-12",
      "severity": "P2",
      "title": "main_imu_test.cpp hard-codes I2C pins as 21,22 instead of importing from a shared constant",
      "file": "code/body-firmware/src/main_imu_test.cpp",
      "quote": "    Wire.begin(21, 22);",
      "also_quote": "// I2C pins for BNO055 (matches main_imu_test.cpp)\nconstexpr int I2C_SDA = 21;\nconstexpr int I2C_SCL = 22;",
      "why": "I2C_SDA/SCL are declared in main_robot.cpp's anonymous namespace as constexpr — but the comment explicitly says \"matches main_imu_test.cpp,\" which has the literal `21, 22` baked into a `Wire.begin()` call. Two sources of truth. Also: CONTEXT.md §5 step 2 calls out that `imu-test-procedure.md` lists 26/25, so we have a third source (the doc) drifting from both code copies. This is a spec-drift problem (Cell 8 territory) but also a quality issue because the comment lies about the relationship.",
      "recommendation": "Promote I2C_SDA / I2C_SCL into `src/config/pins.h` (project-wide constant — IMU is hardware-IDed on the same I2C bus as future sensors). Import in both mains. Update imu-test-procedure.md to reference the constant by name rather than magic numbers."
    },
    {
      "id": "C9-P2-13",
      "severity": "P2",
      "title": "main_robot.cpp boots heap objects without nullptr defense in controlTask",
      "file": "code/body-firmware/src/main_robot.cpp",
      "quote": "    while (true) {\n        // 1) Pull freshest command from latch.\n        auto fresh = g_latch->read();",
      "why": "`g_latch`, `g_producer`, `g_controller` are pointers initialized in `setup()` after FreeRTOS is up. `controlTask` is started at the *end* of `setup()` (line 240) so the ordering is safe in practice. But the deref pattern `g_latch->read()`, `g_controller->update(...)` is repeated 4× in controlTask without a single guard, and there's no `BB8_ASSERT(g_latch != nullptr)` to document the invariant. If anyone reorders setup() (a real risk when adding a sensor), the task fires before the pointers are set and we get a deterministic crash with no clue why. A single `BB8_ASSERT` near the top of controlTask, plus moving `xTaskCreatePinnedToCore` to be the very last call in setup() (already is — good), would document the contract.",
      "recommendation": "Add `BB8_ASSERT(g_latch && g_producer && g_controller, \"controlTask started before setup() finished\")` near the top of controlTask. Even though BB8_ASSERT is a no-op on hardware (C9-P2-06), it's a load-bearing comment for the next reader."
    }
  ]
}
```

## Summary

13 findings (2 P1, 11 P2). Top recommendations in priority order:

1. **Delete `main_blink.cpp` and `main_original.cpp`** — both excluded from every env, `main_blink` references the old pin map (pre-harness-remap), both are confusing to grep. Drops 10 redundant tokens from `platformio.ini`.
2. **Correct CONTEXT.md §9.1** — `wifi_credentials.h` is correctly gitignored and untracked. Structural setup is sound.
3. **Extract a `RobotAssembly` helper** — `main_robot.cpp` and `main_drivetrain_test.cpp` duplicate the 3-motor + 3-PID + drivetrain wiring with identical PID gains; risk of divergence on retune.
4. **Add a `BB8_CHECK`** companion to `BB8_ASSERT` for hot-path range guards that should fire on hardware (without aborting). Current `BB8_ASSERT` is `((void)0)` everywhere except `[env:native]`.
5. **Decouple `config/pins.h` from `L298NDriver.h`** — move the `L298NPins {fwd, rev}` POD into a config-layer header so pins.h doesn't depend on a driver class.
6. **Promote `I2C_SDA/SCL` into `pins.h`** — currently duplicated across `main_robot.cpp` (constexpr) and `main_imu_test.cpp` (magic number).

`Lifecycle` is a thin abstraction with one implementer today, but the cost is small and a second consumer would fully justify it — defer the decision. `Vec3/Quat/Euler/CalStatus` PODs are appropriately minimal; only missing comfort is `operator==` and a Print formatter. `MovingAverage<N>` is the only template in scope and is correctly constrained (`static_assert(N > 0)`).
