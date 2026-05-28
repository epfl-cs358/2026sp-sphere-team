# Cell 11 — Seam: Failure / Disconnect / Failsafe Paths

Scope: every path that can cost comms or sensors; whether each ends at *motors-off within a bounded time*; whether recovery is clean.

Anchored at f8ca95f. Files of record: `src/main_robot.cpp`, `src/input/WebSocketCommandProducer.{h,cpp}`, `src/motor/driver/L298NDriver.{h,cpp}`, `src/drivetrain/OmniDrivetrain.h`, `src/control/PID.h`, `src/imu/BNO055IMU.h`, `src/util/Lifecycle.h`, `test/`.

---

## Trigger → motors-off table

| # | Trigger | Path to "motors-off" | Upper bound | Enforced by | Active brake? |
|---|---------|----------------------|-------------|-------------|---------------|
| T1 | WiFi drops mid-run | Latch keeps last cmd; producer task still up; no fresh frames → staleness ramp → cmd=0 | **200 ms** (ramp end) | `main_robot.cpp:174-183` (ramp) | No — PIDs hold 0 RPM, L298N writes 0/0 (coast) |
| T2 | WS client clean disconnect (FIN) | `WStype_DISCONNECTED` clears `_connected`; cmds stop → ramp → cmd=0 | **200 ms** | same ramp | No |
| T3 | WS half-open TCP (no FIN, no commands) | Heartbeat 2 s/1 s/2 retries = ~6 s to detect; but ramp triggers at **200 ms** because no new frames hit latch | **200 ms** to motors-coast, **~6 s** to `_connected=false` | ramp (motors), heartbeat (flag only) | No |
| T4 | WS half-open with stale-cmd spam (e.g. client TCP buffered, OS resends old frames) | If frames keep arriving, latch keeps refreshing; ramp NEVER fires → motors hold last cmd indefinitely | **UNBOUNDED** until heartbeat (~6 s) — *and even then nothing in code consumes the flag* | nothing | No — see C11-P0-01 |
| T5 | Operator sends no commands >200 ms | `last_fresh_ms` ages past `STALENESS_TIMEOUT_MS` → ramp → cmd=0 | **200 ms** | ramp | No |
| T6 | BNO055 I²C error mid-run | `g_imu.read()` returns whatever Adafruit_BNO055 returns on bus failure; **no error check**; control loop continues with stale/garbage IMU. PIDs still drive motors. | **NEVER** — never trips staleness, TWDT, or any other failsafe | nothing | No — see C11-P0-02 |
| T7 | Motor driver fault (encoder stops reporting) | `getFilteredRPM()` returns 0; PID sees error = setpoint − 0 = setpoint; integrator winds up to saturation. PWM goes to max on that wheel. | **NEVER** — runaway on the un-faulted side | nothing | No — see C11-P0-03 |
| T8 | OTA flash starts | `ArduinoOTA.onStart` → `g_producer->stop()` (blocks ≤1 s). No new frames → ramp → cmd=0, *then* flash writes begin | **200 ms** (after `stop()` returns) | ramp | No — motors coast through flash; see C11-P0-04 |
| T9 | Power glitch / brownout | ESP32 brownout detector is at chip default (~2.43 V), no software policy. On reset, all GPIOs revert to high-Z. L298N inputs floating → driver output undefined. After boot, motors are coast until `g_motor*.begin()` (pin OUTPUT, default LOW). | reset → re-arm in **~hundreds of ms to multiple seconds** depending on WiFi-blocking `connectWifi()` (30 s timeout) | none specific; ESP32 brownout det. is enabled by default but no software callback | No |
| T10 | TWDT fires (1 s, panic=true) | `esp_task_wdt_init(1 s, true)` → panic → reset. Motors then coast (T9 path). | **1 s** to reset; full reboot cycle (WiFi reconnect 30 s budget) before re-armable | TWDT | No — coast during reset window |

> All "motors-coast" rows: at zero command the PIDs still run with setpoint=0. With deadband=5 RPM and measurement < 5 RPM, `PID::compute` calls `reset()` and returns `0.0f` (`PID.h:22-24`). L298N at 0.0 writes `analogWrite(fwd,0); analogWrite(rev,0)` (`L298NDriver.cpp:24-28`) — **coast, not brake**. On an incline the robot rolls.

```json
{
  "cell": 11,
  "scope": "seam-failure-failsafe-paths",
  "anchor_commit": "f8ca95f",
  "findings": [

    {
      "id": "C11-P0-01",
      "title": "Stale-but-arriving frames bypass the staleness ramp entirely",
      "severity": "P0",
      "lens": "safety",
      "file": "src/main_robot.cpp",
      "lines": "160-183",
      "quote": "auto fresh = g_latch->read();\n        const uint32_t now = millis();\n        if (fresh.has_value()) {\n            last_known      = *fresh;\n            last_fresh_ms   = now;\n            have_seen_fresh = true;\n        }\n\n        // 2) Drive command via staleness alone. The producer's connected()\n        //    flag can lie on half-open TCP; age < 200ms is the actual safety\n        //    net so we don't gate on connected() anymore.",
      "claim": "The staleness clock is reset by ANY frame in the latch, not by an operator-side timestamp. If a stuck client / proxy / OS retransmit replays a non-zero command, the ramp never fires; motors hold that command indefinitely. The comment above the code claims age<200 ms is 'the actual safety net' but the clock measures 'time since something landed in the latch', not 'time since operator intent'. There is no monotonic-counter or operator timestamp in `CommandFrameParser` to detect replays. Combined with T4, a wedged operator-side TCP stack can drive at full speed for >>6 s with zero new operator intent.",
      "impact": "Unbounded time-to-motors-off in the half-open-with-stale-frames scenario. This is the *entire* point of the staleness contract and it is bypassed.",
      "remediation": "Add a wire-protocol field `seq` (u32 monotonic) or `ts_ms` and reset `last_fresh_ms` only when the seq advances. Optionally, gate also on `_connected.load()` once the heartbeat-driven flag is honored; today line 170-171 explicitly chooses not to.",
      "verification": "Add a native test that pushes the SAME `BodyVelocity{1,0,0}` into the latch at 100 Hz for >200 ms and asserts the drive command decays — it currently won't."
    },

    {
      "id": "C11-P0-02",
      "title": "IMU read failure is silent — control loop keeps driving on stale/garbage data",
      "severity": "P0",
      "lens": "safety",
      "file": "src/imu/BNO055IMU.h",
      "lines": "33-80",
      "quote": "IMUReading read() override {\n        BB8_ASSERT(_initialized, \"BNO055: read() called before begin()\");\n        if (!_initialized) return IMUReading{};\n\n        IMUReading r;\n\n        if (_fields & IMUField::Quaternion) {\n            auto q = _bno.getQuat();",
      "claim": "`BNO055IMU::read()` never returns an error and never reports I2C failure. `Adafruit_BNO055::getQuat()` on bus error returns a zero quaternion; downstream `IMUReading` carries identity/zero defaults indistinguishable from valid silence. `main_robot.cpp:186-187` consumes the reading and passes it to `PassthroughDrivetrainController::update` without checking; for the current passthrough this is benign, but the moment any stabilizer controller is wired (see `docs/controller-discussion.md`), an undetected IMU failure will drive open-loop and likely runaway. Even today, the control task can block for a long time inside `_bno.getQuat()` if I2C stretches — see TWDT bound (1 s), so worst case is reset, not runaway. Still, no error signal to the rest of the system.",
      "impact": "Future stabilizer controllers will fail silently with garbage IMU. Today: undetected sensor degradation; T6 row in the trigger table = NEVER.",
      "remediation": "Return `std::optional<IMUReading>` or a `bool ok` from `IMU<T>::read()`. Adafruit_BNO055 exposes `getEvent()` returning bool and `verify()`; wrap with a consecutive-failure counter, and either zero the command or call `controller->stop()` after N misses.",
      "verification": "Add a stub `Adafruit_BNO055` mock that fails I2C and assert the control loop reacts."
    },

    {
      "id": "C11-P0-03",
      "title": "Encoder/motor fault causes PID windup and one-wheel runaway",
      "severity": "P0",
      "lens": "safety",
      "file": "src/drivetrain/OmniDrivetrain.h",
      "lines": "25-33",
      "quote": "void update(float dt) override {\n        for (auto* m : _motors) m->update();\n\n        float s0 = _pids[0]->compute(_targetRPMs[0], _motors[0]->getFilteredRPM(), dt);\n        float s1 = _pids[1]->compute(_targetRPMs[1], _motors[1]->getFilteredRPM(), dt);\n        float s2 = _pids[2]->compute(_targetRPMs[2], _motors[2]->getFilteredRPM(), dt);",
      "claim": "If an encoder disconnects or a motor stalls under load while the operator commands nonzero, `getFilteredRPM()` reports ~0 while `_targetRPMs[i]` stays at the command. Error = setpoint − 0 = full setpoint; PID integrator winds up to its anti-windup clamp (output/Ki), output saturates at +1.0; that wheel goes to full PWM. The other two wheels keep tracking — the result is a violent yaw or spin. There is no per-wheel fault detection (no |error| threshold + timer, no current sense, no encoder-timeout check in `FIT0186Motor::update`).",
      "impact": "Single-point sensor failure → runaway on the un-faulted axis. The omni-tilt geometry means even one bad wheel can launch the sphere.",
      "remediation": "Add a per-motor fault detector in `FIT0186Motor`: if commanded |PWM| > threshold for >N ms and |measured RPM| ~= 0, raise a fault flag. `OmniDrivetrain::update` then either calls `stop()` on the offending motor (passing fault upward) or reduces all commanded outputs to zero via `controller->stop()`.",
      "verification": "Native test: drive `OmniDrivetrain` with a `MockMotor` that ignores `setSpeed` (forces RPM=0) and assert fault flag fires within bound."
    },

    {
      "id": "C11-P0-04",
      "title": "OTA path does not actively brake motors; only stops the producer",
      "severity": "P0",
      "lens": "safety",
      "file": "src/main_robot.cpp",
      "lines": "104-110",
      "quote": "ArduinoOTA.onStart([]() {\n        // Stop accepting new commands. Once the producer is gone, the staleness\n        // ramp in controlTask will drive motors to zero within 200 ms before\n        // the firmware actually overwrites flash.\n        Serial.println(\"[ota] update starting; halting teleop\");\n        if (g_producer) g_producer->stop();\n    });",
      "claim": "(a) The onStart callback only stops the producer; it does NOT call `g_controller->stop()` or `g_drivetrain.stop()` or zero `_targetRPMs`. The control loop continues running through the ramp and reaches PWM=0 (coast) but the wheels are not actively braked. (b) `g_producer->stop()` blocks up to 1 s on `_exitSemaphore` (`WebSocketCommandProducer.cpp:74`) — that 1 s is added to the 200 ms ramp, so total deadline before flash overwrite = ~1.2 s, comfortably inside ArduinoOTA's own handshake window, but not documented. (c) There is no `onProgress` callback registered; the control task keeps running during flash and competes for flash/CPU with the OTA writer. (d) After `onEnd` the device reboots; on reboot `connectWifi()` blocks for up to 30 s before the control task even starts — during this window motors are coast (T9). (e) If `onError` fires, the producer has been destroyed (`stop()` calls `delete _server; _server = nullptr;`) and there is no `start()` to bring it back — operator must power-cycle.",
      "impact": "OTA-mid-flight coasts the sphere from whatever motion it was in; could roll off a table during the 200 ms + flash window. OTA failure leaves the device in a half-armed state (control loop alive, no producer) until manual reset.",
      "remediation": "In `onStart` also call `g_controller->stop()` (which calls `OmniDrivetrain::stop()` → `motor->brake()` on all three → L298N HIGH/HIGH = active short). In `onError` re-arm: `g_producer->start()` again. Optionally suspend the control task entirely during flash with `vTaskSuspend` and resume in `onEnd/onError`.",
      "verification": "Manual flash test: command a forward velocity, trigger OTA, observe whether wheels coast or brake (audible click on H-bridge brake)."
    },

    {
      "id": "C11-P0-05",
      "title": "OmniDrivetrain::stop() is dead code — no runtime trigger reaches active-brake",
      "severity": "P0",
      "lens": "safety",
      "file": "src/main_robot.cpp",
      "lines": "144-206",
      "quote": "void controlTask(void* /*arg*/) {\n    // Subscribe to the Task Watchdog. If this loop hangs (e.g. I2C bus\n    // stretch on the IMU), TWDT panics and resets — preferable to motors\n    // stuck at last PWM with no operator recovery.\n    esp_task_wdt_add(NULL);",
      "claim": "Grep across `src/` confirms `OmniDrivetrain::stop()` is invoked only by `PassthroughDrivetrainController::stop()` and the off-runtime `main_drivetrain_test.cpp`. `PassthroughDrivetrainController::stop()` has zero callers in `main_robot.cpp` (T1-T8 all end in PWM=0 coast, not `motor->brake()`). Net effect: `Motor::brake()` and the L298N HIGH/HIGH active-brake path are unreachable in production. The 'emergency stop' API exists but is never triggered.",
      "impact": "Across all 9 failure rows, motors-off means coast, never brake. On an inclined surface the robot will roll. The codebase has a functional brake but no wiring to invoke it.",
      "remediation": "Wire `g_controller->stop()` into: (1) OTA onStart (after producer.stop()), (2) post-ramp once `age >= STALENESS_TIMEOUT_MS` AND `have_seen_fresh` (latch the brake instead of coast), (3) a TWDT pretrigger handler via `esp_task_wdt_init_panic` / `esp_register_shutdown_handler`. Even if (3) only has milliseconds before reset, calling `motor->brake()` writes the H-bridge inputs before chip reset — better than coast.",
      "verification": "Native test: assert `mock_motor.brake_count > 0` after a simulated 200 ms staleness episode — currently 0."
    },

    {
      "id": "C11-P1-06",
      "title": "Boot races: WS server accepts commands before TWDT/control task arm",
      "severity": "P1",
      "lens": "safety",
      "file": "src/main_robot.cpp",
      "lines": "228-247",
      "quote": "g_latch      = new CommandLatch<BodyVelocity>();\n    g_producer   = new WebSocketCommandProducer(*g_latch, WS_PORT);\n    g_controller = new PassthroughDrivetrainController(g_drivetrain, g_imu);\n\n    g_producer->start();\n\n    initOta();\n\n    // Tighten the global Task Watchdog timeout. Affects IDLE tasks too, but\n    // 1s is comfortably above their normal slack.\n    esp_task_wdt_init(TWDT_TIMEOUT_S, true);\n\n    xTaskCreatePinnedToCore(\n        &controlTask,",
      "claim": "Order: producer starts → OTA inits → TWDT inits → control task is created last. Between `g_producer->start()` (line 232) and `xTaskCreatePinnedToCore(controlTask, ...)` (line 240) the WS server is live and writing to the latch but nothing reads it. If a client connects in this ~ms-scale window the first command is delivered, then the control task starts and the latch read returns a `last_known` from before-boot intent. Worse: control task subscribes to TWDT *after* `esp_task_wdt_init(1 s, panic=true)` — IDLE0/IDLE1 are TWDT-subscribed in Arduino-ESP32 by default; ws_prod ran briefly under panic=true watchdog already.",
      "impact": "Tiny but non-zero arming-window where the first operator command can leak into post-boot state. Boot is also slow (30 s WiFi timeout) so an impatient operator may already be banging on the WS.",
      "remediation": "Reorder: create control task first (with an initial 'safety-disarmed' flag that forces drive_cmd=0 regardless of latch). Only flip the flag once `setup()` completes. Or: do not call `g_producer->start()` until after the control task is alive."
    },

    {
      "id": "C11-P1-07",
      "title": "Half-open detection takes ~6 s but `_connected` is unused on hot path",
      "severity": "P1",
      "lens": "safety",
      "file": "src/input/WebSocketCommandProducer.cpp",
      "lines": "47-52",
      "quote": "_server->onEvent([this](uint8_t num, WStype_t type, uint8_t* payload, size_t length) {\n        this->onWsEvent(num, static_cast<uint8_t>(type), payload, length);\n    });\n    // Detect half-open TCP within ~6s: ping every 2s, fail after 1s + 2 retries.\n    _server->enableHeartbeat(2000, 1000, 2);",
      "claim": "Heartbeat config matches the comment (ping 2 s, pong-timeout 1 s, 2 retries → upper bound 2 s × 3 = 6 s before disconnect event). The control loop intentionally ignores `_connected` (`main_robot.cpp:170-171`). In the half-open-silent case (T3) ramp catches it at 200 ms — fine. In half-open-with-stale-frames (T4, see C11-P0-01) ramp doesn't catch it and `_connected` is ignored, so detection is effectively never. Even when heartbeat does fire the disconnect event, the producer task and control loop don't observe it as anything other than the absence of new frames — which the ramp already handles when frames stop.",
      "impact": "The 6 s heartbeat is currently observability-only. Pair with C11-P0-01 — the *combination* is the runaway.",
      "remediation": "Either (a) on `WStype_DISCONNECTED` for the active client, write a zero `BodyVelocity` into the latch immediately AND mark a 'disconnect-latched' flag the control loop reads to call `controller->stop()`; or (b) gate the staleness clock on operator-side sequence numbers (C11-P0-01) so half-open with replay is detected at 200 ms regardless."
    },

    {
      "id": "C11-P1-08",
      "title": "PID integrator persists across disconnect/reconnect cycles",
      "severity": "P1",
      "lens": "safety",
      "file": "src/main_robot.cpp",
      "lines": "144-206",
      "quote": "BodyVelocity last_known{};\n    uint32_t     last_fresh_ms = 0;\n    bool         have_seen_fresh = false;",
      "claim": "On WS disconnect → reconnect, the control task's locals reset only on a process reset, not on a producer-restart. `_targetRPMs[]` inside `OmniDrivetrain` and the three `PID` integrators are also never reset by any code path the runtime exercises (`PID::reset()` is only called from `OmniDrivetrain::stop()` which is dead code — see C11-P0-05). After a stale-disarm-then-reconnect, the new operator inherits whatever integrator state the previous session left behind, including possibly saturated integrators if the previous session ended under load.",
      "impact": "Reconnect surprise — first command after reconnect can produce a kick. The PID deadband zero-snap path (`PID.h:22-24`) resets only when *both* setpoint and measurement are near zero, so a snapshot at fast-decel may not snap.",
      "remediation": "On `WStype_DISCONNECTED` for the active client OR on staleness timeout, the control loop should call `g_controller->stop()` once (latched) — this resets PIDs and `_targetRPMs`. Re-arm on first fresh frame after reconnect."
    },

    {
      "id": "C11-P1-09",
      "title": "No brownout policy; no brake-on-power-loss",
      "severity": "P1",
      "lens": "safety",
      "file": "src/main_robot.cpp",
      "lines": "210-250",
      "quote": "void setup() {\n    Serial.begin(115200);\n    delay(200);\n    Serial.println(\"BB-8 main_robot starting.\");\n\n    Wire.begin(I2C_SDA, I2C_SCL);\n\n    g_motor0.begin();\n    g_motor1.begin();\n    g_motor2.begin();",
      "claim": "Grep across the body-firmware tree: zero references to `brownout`, `setBrownoutDetector`, `WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG...)`, or `esp_brownout_register`. ESP32-Arduino enables the brownout detector by default (~2.43 V) and brownout → reset is the chip-level behavior. The reset path is T9: GPIOs go high-Z (good — L298N inputs floating → 0 V → both LOW → coast), motors coast through the reset, then `g_motor*.begin()` sets pins OUTPUT/LOW (still coast), then WiFi blocks for up to 30 s. With L298N, this is acceptable (coast, not free-PWM). With BTS7960 in the future, the boot-time pin behavior must be re-validated — BTS7960 has explicit enable lines.",
      "impact": "On battery sag during high-current acceleration, the chip resets, motors coast for the duration of the reset+reconnect cycle (could be tens of seconds). On an incline this rolls. No active-brake-on-brownout path.",
      "remediation": "Add an `esp_register_shutdown_handler` that calls `g_drivetrain.stop()` to brake before reset. Add a brownout pretrigger via `esp_brownout_register_callback` (ESP-IDF), and consider raising brownout threshold for early warning. Also: at boot, set `g_lastWifiConnectedMs = millis()` before the 30 s `connectWifi` loop to avoid the post-boot reboot watchdog tripping during a slow WiFi join."
    },

    {
      "id": "C11-P1-10",
      "title": "loopTask (Arduino main) is not TWDT-subscribed — OTA hang is invisible",
      "severity": "P1",
      "lens": "realtime",
      "file": "src/main_robot.cpp",
      "lines": "252-264",
      "quote": "void loop() {\n    // Housekeeping: OTA poll + WiFi reboot watchdog. Control runs in dedicated\n    // tasks. Tick at 10 Hz so OTA initiate requests are answered promptly.\n    ArduinoOTA.handle();\n\n    if (WiFi.status() == WL_CONNECTED) {\n        g_lastWifiConnectedMs = millis();\n    } else if (millis() - g_lastWifiConnectedMs > WIFI_OFFLINE_REBOOT_MS) {\n        Serial.println(\"FATAL: WiFi offline >30s — restarting.\");\n        ESP.restart();\n    }",
      "claim": "`controlTask` (line 148) and `taskBody` (`WebSocketCommandProducer.cpp:103`) both subscribe to TWDT. Arduino's `loopTask` is created by the framework and is NOT subscribed in this code. If `ArduinoOTA.handle()` deadlocks or `WiFi.status()` blocks, the housekeeping task hangs silently — the control task keeps running, motors keep tracking commands (or staleness ramp). The 30 s WiFi offline reboot watchdog is itself in the unwatched task, so a hang of loopTask defeats the WiFi watchdog.",
      "impact": "A class of OTA/WiFi-stack bugs becomes silent: no TWDT panic, no reset. The control task happily runs without WiFi-recovery oversight.",
      "remediation": "Either subscribe loopTask to TWDT (`esp_task_wdt_add(NULL); esp_task_wdt_reset()` inside `loop`) — but the function is short and a 1 s budget is generous — or move the WiFi-reboot watchdog into a dedicated TWDT-subscribed task."
    },

    {
      "id": "C11-P1-11",
      "title": "Zero test coverage of failure paths",
      "severity": "P1",
      "lens": "quality",
      "file": "test/",
      "lines": "all",
      "quote": "test_omni_drivetrain/test_omni_drivetrain.cpp:    dt.stop();",
      "claim": "Native test inventory under `test/`: pid, omni_kinematics, omni_drivetrain (stop is tested in isolation), passthrough_controller (stop tested in isolation), command_latch, command_frame_parser, fit0186_motor, l298n_driver, bts7960_driver, moving_average, bno055_imu, mock_command_producer. NONE of them simulate: a staleness ramp, a WS disconnect, an IMU read failure, an encoder fault, an OTA hook firing, a TWDT-pretrigger, or a brownout shutdown. The seam between WS producer + latch + control task + ramp + drivetrain is untested as an integrated unit. C11-P0-01 (replay), C11-P0-02 (IMU silent), C11-P0-03 (encoder fault) are all easy to express as native tests with the existing mocks (`test_mock_motor.h`, `MockCommandProducer`) and would have caught these issues.",
      "impact": "All safety claims in this review are static-analysis claims; nothing prevents regressions.",
      "remediation": "Add `test_staleness_ramp/` (drive `controlTask`-equivalent function with controlled clock + latch), `test_disconnect_recovery/`, `test_imu_failure/` (mock IMU returning identity quat), `test_encoder_fault/` (mock motor returning RPM=0 under nonzero command)."
    },

    {
      "id": "C11-P2-12",
      "title": "Zero-command coast vs hold-position on an incline",
      "severity": "P2",
      "lens": "safety",
      "file": "src/control/PID.h",
      "lines": "22-26",
      "quote": "if (_deadband > 0.0f && std::abs(setpoint) < 1e-6f && std::abs(measurement) < _deadband) {\n            reset();\n            return 0.0f;\n        }",
      "claim": "Definition of 'motors off' in this codebase = setpoint=0 to PID, which under the deadband snap (setpoint≈0 AND |measured|<5 RPM) returns 0.0 and resets the integrator. If the robot is on an incline and rolling, |measured| > 5 RPM → no snap; PID actively fights with `error = -measurement`, output = `Kp * (-measurement)` = `-0.005 * measured_rpm`. At 50 RPM that's −0.25 → 25% reverse PWM. So 'zero command' is NOT 'PWM=0' on a moving robot — the PID brakes electrically (good, behavior-wise) but it's not what the table above calls 'coast'. On the *flat* with momentum the PIDs do hold position roughly. On an incline the PIDs fight the gravity-induced wheel rotation but can be overpowered. This is subtler than the context doc states.",
      "impact": "The 'staleness → motors off' wording in the context oversimplifies. Behavior is 'PIDs actively try to hold zero RPM' for as long as they can — which is *better* than coast for safety, but not the same as brake. And once the deadband snap fires (when the bot is nearly stopped), motors release.",
      "remediation": "Document this behavior explicitly. Decide whether the desired end-state is 'hold zero RPM' (current) or 'active brake' (`motor->brake()` short via L298N HIGH/HIGH). On an incline the latter is more conservative."
    },

    {
      "id": "C11-P2-13",
      "title": "Producer rejected client path can leak repeated connect attempts (no rate limit)",
      "severity": "P2",
      "lens": "safety",
      "file": "src/input/WebSocketCommandProducer.cpp",
      "lines": "121-132",
      "quote": "case WStype_CONNECTED: {\n            // Single-client policy: accept the first client, reject the rest.\n            // Two operators sending commands would oscillate the latch.\n            uint8_t expected = kNoClient;\n            if (_activeClient.compare_exchange_strong(expected, clientNum)) {\n                _connected = true;\n                Serial.printf(\"[ws] client %u connected\\n\", clientNum);\n            } else {\n                Serial.printf(\"[ws] client %u rejected (active=%u)\\n\", clientNum, expected);\n                if (_server) _server->disconnect(clientNum);\n            }\n            break;\n        }",
      "claim": "A second connecting client is disconnected, but each rejection runs `Serial.printf` synchronously on the ws_prod task. An attacker (or buggy client) hammering connect attempts can hold the WS task in Serial-flushing for extended periods, starving the WS heartbeat poll and indirectly degrading T3 detection time. Also, `Serial.printf` of `parse_fail` is rate-limited (`logParseFail` line 184) but the connect-reject log is not.",
      "impact": "Low — needs malicious/buggy LAN client. Worth a 1 Hz rate-limit on connect-reject logging and possibly an IP-level cooldown via `server->disconnect`.",
      "remediation": "Rate-limit connect-reject logs identically to `logParseFail`. Optionally maintain a small recent-rejected-IP set."
    }
  ],
  "seam_questions_answered": {
    "Q1_bounded_time_to_motors_off": "See trigger table. Bounded for T1/T2/T3/T5/T8 (~200 ms ramp). UNBOUNDED for T4 (stale-frames replay — C11-P0-01), T6 (IMU failure — C11-P0-02), T7 (encoder fault — C11-P0-03). For T10 (TWDT) bounded at 1 s but reset path is also coast.",
    "Q2_zero_command_vs_pwm_zero": "Not the same. Zero command → PID with setpoint=0; deadband snap returns 0.0 only when |measured|<5 RPM. Above 5 RPM the PID actively brakes electrically (Kp*-measured). On incline rolling fast the PID is being asked to brake but is rate-limited by Kp=0.005. See C11-P2-12.",
    "Q3_omni_drivetrain_stop_unreachable": "Confirmed dead code in main_robot.cpp. Only invoked by main_drivetrain_test.cpp and the unwired PassthroughDrivetrainController::stop(). Should be called from: OTA onStart (after producer.stop), post-staleness-timeout (latch a brake state), and ideally a TWDT shutdown handler. See C11-P0-05.",
    "Q4_ota_mid_flight": "onStart calls producer.stop() only — no controller.stop(), no motor.brake(). No onProgress. onEnd reboots. onError leaves producer destroyed and unrecoverable without power cycle. See C11-P0-04.",
    "Q5_half_open_tcp": "Heartbeat 2s/1s/2 retries = ~6s detection. _connected flag is intentionally not consulted on hot path (main_robot.cpp:170-171). Staleness ramp covers half-open-silent case at 200 ms; does NOT cover half-open-with-replay-frames. See C11-P0-01 + C11-P1-07.",
    "Q6_reconnect_path": "Not clean. Locals in controlTask (last_known, last_fresh_ms, have_seen_fresh) only reset on process restart. _targetRPMs and PID integrators are never reset by any runtime path (only by dead OmniDrivetrain::stop). See C11-P1-08.",
    "Q7_twdt_panic": "1 s, panic=true. Subscribed: controlTask + ws_prod task. NOT subscribed: Arduino loopTask (C11-P1-10). On reset motors coast — no brake-on-power-loss handler. See C11-P1-09.",
    "Q8_brownout": "Zero references in code. ESP32-Arduino default detector at ~2.43V → reset. Coast-through-reset with L298N. No software policy. See C11-P1-09.",
    "Q9_boot_armed_state": "Producer starts BEFORE control task is created (lines 232 vs 240). Tiny race window where WS server is live but no consumer of the latch. WiFi join blocks up to 30 s and reboot watchdog timer (g_lastWifiConnectedMs initialized to 0) could trip immediately post-boot if WiFi join is slow. See C11-P1-06.",
    "Q10_test_coverage": "Zero failure-path tests. All findings P0/P1 above are easy to express in the existing native test harness. See C11-P1-11."
  }
}
```
