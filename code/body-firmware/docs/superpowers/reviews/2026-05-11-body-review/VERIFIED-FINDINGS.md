# Verified Findings — Body Firmware Review 2026-05-11

**Anchored at commit:** f8ca95f (f8ca95ff7fd8117f732fb026e019d957cf209645)

## Verification summary

| Cell | Findings parsed | Verified | Misplaced (line fixed) | Unverified (dropped) |
|---|---|---|---|---|
| C1 | 15 | 15 | 0 | 0 |
| C2 | 14 | 14 | 0 | 0 |
| C3 | 19 | 19 | 0 | 0 |
| C4 | 13 | 13 | 0 | 0 |
| C5 | 11 | 11 | 0 | 0 |
| C6 | 14 | 14 | 0 | 0 |
| C7 | 12 | 12 | 0 | 0 |
| C8 | 12 | 12 | 0 | 0 |
| C9 | 13 | 13 | 0 | 0 |
| C10 | 12 | 12 | 0 | 0 |
| C11 | 13 | 12 | 0 | 1 |
| C12 | 21 | 21 | 0 | 0 |
| **Total** | 169 | 168 | 0 | 1 |

## P0 — verified

### Cell 1

```json
{
  "id": "C1-P0-01",
  "title": "NaN/Inf BodyVelocity propagates to analogWrite as undefined uint8_t",
  "file": "src/motor/driver/L298NDriver.cpp",
  "line_range": "20-22",
  "quote": "BB8_ASSERT(value >= -1.0f && value <= 1.0f, \"setOutput value out of range [-1,1]\");\n\n    uint8_t pwm = static_cast<uint8_t>(std::abs(value) * 255.0f);",
  "risk": "BB8_ASSERT is a no-op outside `[env:native]` (debug.h:26). `CommandFrameParser` uses `std::strtof` with no finite/bounds check (CommandFrameParser.h:46-54), so a client sending `'nan,0,0'` or `'1e30,0,0'` produces a BodyVelocity that propagates through `OmniKinematics::toWheelRPMs` (NaN poisons all 3 RPMs; saturation scaling does not catch NaN beca …[truncated]",
  "suggested_fix": "Add a runtime clamp in `Driver::setOutput` implementations (replace the BB8_ASSERT with `if (!std::isfinite(value)) value = 0.0f; value = std::clamp(value, -1.0f, 1.0f);`). Also reject non-finite fram …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C1-P0-02",
  "title": "Sub-threshold dt freezes saturated PID output indefinitely",
  "file": "src/control/PID.h",
  "line_range": "20-20",
  "quote": "if (dt < 1e-4f) return _lastOutput;",
  "risk": "If `dt < 100µs` is ever passed (e.g. caller drift, double-pump, or a future loop change), the PID returns the previous output verbatim. If `_lastOutput` was at `+1.0` saturation, the motor stays at full forward across an unbounded number of such ticks — there is no max-dwell or fallback. The control task's nominal dt is 10 ms (well above), but the …[truncated]",
  "suggested_fix": "Return 0 (not last_output) when dt is sub-threshold, OR keep last_output but clamp to a watchdog max-dwell (e.g. if the same dt-skip happens >N consecutive calls, reset to 0).",
  "verification_status": "verified"
}
```

```json
{
  "id": "C1-P0-03",
  "title": "L298N brake (255/255) is left latched if anything calls brake() then setOutput() never executes",
  "file": "src/drivetrain/OmniDrivetrain.h",
  "line_range": "39-43",
  "quote": "void stop() override {\n        for (auto* m : _motors) m->brake();\n        for (auto* p : _pids) p->reset();\n        for (int i = 0; i < 3; i++) _targetRPMs[i] = 0.0f;\n    }",
  "risk": "`stop()` brakes the motors (both pins HIGH on L298N — a hard short between motor terminals, which is the correct fast-stop but high-current). It is *never called* by the runtime control loop (see CONTEXT §9.11): the staleness path drives `BodyVelocity{0,0,0}` through the *PID*, not through stop(). However, if any future code path calls `stop()` and …[truncated]",
  "suggested_fix": "Add a `Motor::coast()` / `Driver::coast()` primitive that writes 0/0 on L298N and Disable on BTS7960; have `OmniDrivetrain::stop()` call `coast()` not `brake()` unless an explicit hard-brake intent is …[truncated]",
  "verification_status": "verified"
}
```

### Cell 2

```json
{
  "id": "C2-P0-01",
  "title": "Parser accepts NaN/Inf and propagates them through latch into PID/motors",
  "file": "src/input/CommandFrameParser.h",
  "line_range": "46-58",
  "quote": "float vx = std::strtof(p, &end);",
  "risk": "`std::strtof` parses 'nan', 'NaN', 'inf', '+inf', '-inf' (case-insensitive) as valid floats. Parser returns ok and `_latch.write(result.value)` (WebSocketCommandProducer.cpp:161) pushes `BodyVelocity{NaN,...}` to control. `last_known.vx * scale` (main_robot.cpp:179) yields NaN; `OmniKinematics::toWheelRPMs` propagates it (saturation scan `a > maxAb …[truncated]",
  "suggested_fix": "Reject after each strtof if `!std::isfinite(v)`; add `FrameParseError::NonFinite`. Defensive: assert finite in `OmniDrivetrain::drive`.",
  "verification_status": "verified"
}
```

```json
{
  "id": "C2-P0-02",
  "title": "No magnitude clamp on parsed velocity — any LAN client commands any value",
  "file": "src/input/CommandFrameParser.h",
  "line_range": "46-58",
  "quote": "return {BodyVelocity{vx, vy, omega}, FrameParseError::None};",
  "risk": "Nothing bounds `vx/vy/omega`. Protocol is unauthenticated `ws://` (WebSocketCommandProducer.h:22-23). A client sending `1e9,1e9,1e9` is accepted. IK saturates per wheel, but staleness then bleeds `last_known` over 200 ms as a max-thrust ramp rather than effectively-zero — runaway hazard on a slope.",
  "suggested_fix": "Define `kMaxVxMps/kMaxVyMps/kMaxOmegaRadps` in `RobotConstants.h`; clamp or reject in parser. Defensive clamp also in `OmniDrivetrain::drive`.",
  "verification_status": "verified"
}
```

```json
{
  "id": "C2-P0-03",
  "title": "Mid-run BNO055 I²C failure is silent — stale/garbage flows, no recovery",
  "file": "src/imu/BNO055IMU.h",
  "line_range": "33-80",
  "quote": "IMUReading read() override {",
  "risk": "`read()` has no error path. Adafruit_BNO055 returns zero/last-state silently on bus error. `controlTask` reads every 10 ms (main_robot.cpp:186). Today passthrough ignores IMU, bounded blast radius — but `docs/controller-discussion.md`'s tilt stabilizer will trust this data; `Quat{1,0,0,0}` is identical for 'no-begin' and 'real-upright', so a dead b …[truncated]",
  "suggested_fix": "Return `bool`/`std::optional<IMUReading>` or add `IMUReading::valid`. Track time-since-last-successful-read; control loop treats stale IMU like stale command (zero + brake).",
  "verification_status": "verified"
}
```

### Cell 3

```json
{
  "id": "C3-P0-01",
  "title": "Loose H-bridge pins between Wire.begin and motor.begin can float to PWM-high during boot",
  "file": "src/main_robot.cpp",
  "line_range": "215-219",
  "quote": "Wire.begin(I2C_SDA, I2C_SCL);\n\n    g_motor0.begin();\n    g_motor1.begin();\n    g_motor2.begin();",
  "risk": "The six L298N IN3/IN4 GPIOs (14, 27, 16, 17, 25, 26) are not driven LOW before `pinMode(..., OUTPUT)` runs inside each motor's `begin()`. On a power-on reset they default to floating inputs with weak pulls; on a soft `ESP.restart()` they can retain prior PWM state through the brief gap. `main_imu_test.cpp:22-32` explicitly contains `killMotorPins() …[truncated]",
  "suggested_fix": "Mirror `main_imu_test.cpp`'s `killMotorPins()` and call it as the very first action in `setup()` before any other init. Also call `g_motor*.begin()` *before* `g_imu.begin()`/`connectWifi()` so a 30 s …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C3-P0-02",
  "title": "Motors armed for 30 s before WiFi/control task come up — staleness ramp not yet running",
  "file": "src/main_robot.cpp",
  "line_range": "217-247",
  "quote": "g_motor0.begin();\n    g_motor1.begin();\n    g_motor2.begin();\n\n    if (!g_imu.begin()) {",
  "risk": "Boot order is: motors begin (pins go OUTPUT) → IMU begin → connectWifi (blocks up to 30 s) → producer start → OTA → TWDT init → controlTask create. Between line 219 and line 240 the H-bridge IN pins are configured outputs (driven LOW by `pinMode`), but there is no task pulling commands or running the staleness watchdog. A glitch on the L298N's enab …[truncated]",
  "suggested_fix": "Construct latch + controller and start `controlTask` *before* `connectWifi()`. The task will run with `last_known={0,0,0}` and `have_seen_fresh=false` ⇒ `drive_cmd=0` every tick — actively zeroing mot …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C3-P0-03",
  "title": "TWDT init happens AFTER `g_producer->start()` — ws_prod's `esp_task_wdt_add(NULL)` runs against a 5 s default",
  "file": "src/main_robot.cpp",
  "line_range": "232-240",
  "quote": "g_producer->start();\n\n    initOta();\n\n    // Tighten the global Task Watchdog timeout. Affects IDLE tasks too, but\n    // 1s is comfortably above their normal slack.\n    esp_task_wdt_init(TWDT_TIMEOUT_S, true);",
  "risk": "`WebSocketCommandProducer::taskBody()` calls `esp_task_wdt_add(NULL)` at line 103, then loops. Because `start()` returns immediately after `xTaskCreatePinnedToCore`, that task can subscribe to the watchdog *before* `esp_task_wdt_init(1, true)` runs on the main task. ESP-IDF's behaviour on subscribing to an uninitialised TWDT differs by core release …[truncated]",
  "suggested_fix": "Move `esp_task_wdt_init(TWDT_TIMEOUT_S, true)` to be the *first* action in `setup()` after `Serial.begin()`, before any `xTaskCreatePinnedToCore` call. The two tasks then both subscribe under a known …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C3-P0-04",
  "title": "OTA flash starts even if `g_producer->stop()` times out — motors not actively driven to zero",
  "file": "src/main_robot.cpp",
  "line_range": "104-110",
  "quote": "ArduinoOTA.onStart([]() {\n        // Stop accepting new commands. Once the producer is gone, the staleness\n        // ramp in controlTask will drive motors to zero within 200 ms before\n        // the firmware actually overwrites flash.\n        Serial.println(\"[ota] update starting; halting teleop\");\n        if (g_producer) g_producer->stop();\n    } …[truncated]",
  "risk": "(a) The `onStart` handler does **not** call `g_drivetrain.stop()` or `g_controller->stop()`. It relies on the staleness ramp inside `controlTask` to zero motors *while* OTA is overwriting flash. The control task continues running PWM during the ~10–30 s flash window — and `OmniDrivetrain::update(dt)` keeps ticking the PIDs every 10 ms even after st …[truncated]",
  "suggested_fix": "In `ArduinoOTA.onStart`: (1) `g_drivetrain.stop()` to actively brake every motor, (2) suspend `controlTask` via `vTaskSuspend(controlTaskHandle)` to prevent the PIDs from re-energising motors, (3) cal …[truncated]",
  "verification_status": "verified"
}
```

### Cell 4

```json
{
  "id": "C4-P0-01",
  "title": "PID dt is constant 0.010s, decoupled from real loop period — under jitter the integrator and derivative term lie",
  "file": "src/main_robot.cpp",
  "line_range": "190-191",
  "quote": "const float dt = static_cast<float>(CONTROL_PERIOD_MS) / 1000.0f;\n        g_drivetrain.update(dt);",
  "risk": "",
  "verification_status": "verified"
}
```

```json
{
  "id": "C4-P0-02",
  "title": "1 kHz LEDC PWM × 100 Hz PID = only ~10 carrier cycles per update; 8-bit resolution at low duty quantizes to ~25 RPM steps",
  "file": "src/motor/driver/L298NDriver.cpp",
  "line_range": "22-36",
  "quote": "uint8_t pwm = static_cast<uint8_t>(std::abs(value) * 255.0f);\n\n    if (pwm == 0) {\n        analogWrite(_pins.fwd, 0);\n        analogWrite(_pins.rev, 0);\n        return;\n    }",
  "risk": "",
  "verification_status": "verified"
}
```

```json
{
  "id": "C4-P0-03",
  "title": "OmniDrivetrain::stop() is unreachable from any runtime path — failsafe never engages active braking; staleness ramp only sets setpoint to ze …[truncated]",
  "file": "src/main_robot.cpp",
  "line_range": "173-191",
  "quote": "BodyVelocity drive_cmd{0.0f, 0.0f, 0.0f};\n        if (have_seen_fresh) {\n            const uint32_t age = now - last_fresh_ms;\n            if (age < STALENESS_TIMEOUT_MS) {\n                const float scale = 1.0f - static_cast<float>(age) /\n                                            static_cast<float>(STALENESS_TIMEOUT_MS);",
  "risk": "",
  "verification_status": "verified"
}
```

### Cell 5

```json
{
  "id": "C5-P0-04",
  "title": "WStype_TEXT parse + latch.write runs **on the ws_prod task** (Core 0), not on a separate WS callback thread — but no auth, no rate limit, an …[truncated]",
  "file": "src/input/WebSocketCommandProducer.cpp",
  "line_range": "",
  "quote": "case WStype_TEXT: {\n            // Drop frames from non-active clients (defense in depth: server\n            // disconnect on rejection isn't always immediate).\n            if (_activeClient.load() != clientNum) return;\n\n            auto result = parseCommandFrame(payload, length);",
  "risk": "On a flood (intentional or buggy operator), ws_prod's wake rate falls because each Serial.printf burns ~0.5-1 ms. Even rate-limited to 1/s the **periodic counter dump** (`Serial.printf(\"[ws] frames=%u ...`) at line 166 fires every 1000 frames and is unconditional. Worse, the periodic counter dump and the parse-fail printf can interleave with WS hea …[truncated]",
  "verification_status": "verified"
}
```

### Cell 6

```json
{
  "id": "C6-P0-01",
  "title": "Control loop assumes fixed dt=10ms regardless of actual elapsed time",
  "file": "src/main_robot.cpp",
  "line_range": "189-191",
  "quote": "// 4) Tick motor controllers (RPM PID).\n        const float dt = static_cast<float>(CONTROL_PERIOD_MS) / 1000.0f;\n        g_drivetrain.update(dt);",
  "risk": "",
  "verification_status": "verified"
}
```

```json
{
  "id": "C6-P0-02",
  "title": "Blocking Serial.printf inside the 100 Hz control loop",
  "file": "src/main_robot.cpp",
  "line_range": "195-202",
  "quote": "if (hwm_iter < 100) {\n            UBaseType_t hwm = uxTaskGetStackHighWaterMark(NULL);\n            if (hwm < stack_hwm_min) stack_hwm_min = hwm;\n            if (++hwm_iter == 100) {\n                Serial.printf(\"[ctrl] stack HWM min over 100 iters: %u words free\\n\",\n                              stack_hwm_min);\n            }\n        }",
  "risk": "",
  "verification_status": "verified"
}
```

```json
{
  "id": "C6-P0-03",
  "title": "ESP.restart() reboot paths do not stop motors first",
  "file": "src/main_robot.cpp",
  "line_range": "131-138, 221-224, 257-262",
  "quote": "} else if (millis() - g_lastWifiConnectedMs > WIFI_OFFLINE_REBOOT_MS) {\n        Serial.println(\"FATAL: WiFi offline >30s — restarting.\");\n        ESP.restart();\n    }",
  "risk": "",
  "verification_status": "verified"
}
```

### Cell 10

```json
{
  "id": "C10-P0-01",
  "title": "Reconnect serves stale pre-disconnect command for up to 200 ms — motors lurch on new connect",
  "file": "src/main_robot.cpp",
  "line_range": "150-183",
  "quote": "BodyVelocity last_known{};\n    uint32_t     last_fresh_ms = 0;\n    bool         have_seen_fresh = false;\n\n    TickType_t lastWake = xTaskGetTickCount();\n    const TickType_t period = pdMS_TO_TICKS(CONTROL_PERIOD_MS);\n\n    UBaseType_t stack_hwm_min = static_cast<UBaseType_t>(-1);\n    uint32_t    hwm_iter = 0;\n\n    while (true) {\n        // 1) Pull f …[truncated]",
  "risk": "On WS disconnect, `WebSocketCommandProducer::onWsEvent` (DISCONNECTED case, lines 134-142) **does not publish a zero command to the latch** — it only clears `_activeClient` / `_connected` atomics. The control task ignores `connected()` by design (comment at main_robot.cpp:170-172). The staleness ramp protects in steady state: if the previous client …[truncated]",
  "suggested_fix": "On `WStype_CONNECTED` for the first client (compare_exchange success) publish a zero `BodyVelocity` to the latch BEFORE the client's first frame can arrive. Equivalently: on `WStype_DISCONNECTED` for …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C10-P0-02",
  "title": "Disconnect does not propagate to motors-off — only staleness timer does (200 ms blind window)",
  "file": "src/input/WebSocketCommandProducer.cpp",
  "line_range": "134-142",
  "quote": "case WStype_DISCONNECTED:\n            // Only clear state if the disconnecting client is the active one.\n            // A rejected client's later disconnect must not unset _connected.\n            if (_activeClient.load() == clientNum) {\n                _activeClient = kNoClient;\n                _connected = false;\n                Serial.printf(\"[ws …[truncated]",
  "risk": "From operator-perspective end-to-end latency for 'I closed the tab, robot should stop': WS DISCONNECTED event fires immediately on a clean close, but on a half-open TCP (cable yanked, WiFi router pulled), arduinoWebSockets' heartbeat config is `enableHeartbeat(2000, 1000, 2)` — ping every 2s, fail after 1s, 2 retries — so detection is **~6 seconds* …[truncated]",
  "suggested_fix": "In `WebSocketCommandProducer::stop()` (line 68), before deleting the task and server, call `_latch.write(BodyVelocity{0,0,0})`. This makes both OTA-initiated halt and explicit shutdown immediately sta …[truncated]",
  "verification_status": "verified"
}
```

### Cell 11

```json
{
  "id": "C11-P0-01",
  "title": "Stale-but-arriving frames bypass the staleness ramp entirely",
  "file": "src/main_robot.cpp",
  "line_range": "160-183",
  "quote": "auto fresh = g_latch->read();\n        const uint32_t now = millis();\n        if (fresh.has_value()) {\n            last_known      = *fresh;\n            last_fresh_ms   = now;\n            have_seen_fresh = true;\n        }\n\n        // 2) Drive command via staleness alone. The producer's connected()\n        //    flag can lie on half-open TCP; age < 2 …[truncated]",
  "risk": "The staleness clock is reset by ANY frame in the latch, not by an operator-side timestamp. If a stuck client / proxy / OS retransmit replays a non-zero command, the ramp never fires; motors hold that command indefinitely. The comment above the code claims age<200 ms is 'the actual safety net' but the clock measures 'time since something landed in t …[truncated]",
  "suggested_fix": "Add a wire-protocol field `seq` (u32 monotonic) or `ts_ms` and reset `last_fresh_ms` only when the seq advances. Optionally, gate also on `_connected.load()` once the heartbeat-driven flag is honored; …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C11-P0-02",
  "title": "IMU read failure is silent — control loop keeps driving on stale/garbage data",
  "file": "src/imu/BNO055IMU.h",
  "line_range": "33-80",
  "quote": "IMUReading read() override {\n        BB8_ASSERT(_initialized, \"BNO055: read() called before begin()\");\n        if (!_initialized) return IMUReading{};\n\n        IMUReading r;\n\n        if (_fields & IMUField::Quaternion) {\n            auto q = _bno.getQuat();",
  "risk": "`BNO055IMU::read()` never returns an error and never reports I2C failure. `Adafruit_BNO055::getQuat()` on bus error returns a zero quaternion; downstream `IMUReading` carries identity/zero defaults indistinguishable from valid silence. `main_robot.cpp:186-187` consumes the reading and passes it to `PassthroughDrivetrainController::update` without c …[truncated]",
  "suggested_fix": "Return `std::optional<IMUReading>` or a `bool ok` from `IMU<T>::read()`. Adafruit_BNO055 exposes `getEvent()` returning bool and `verify()`; wrap with a consecutive-failure counter, and either zero th …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C11-P0-03",
  "title": "Encoder/motor fault causes PID windup and one-wheel runaway",
  "file": "src/drivetrain/OmniDrivetrain.h",
  "line_range": "25-33",
  "quote": "void update(float dt) override {\n        for (auto* m : _motors) m->update();\n\n        float s0 = _pids[0]->compute(_targetRPMs[0], _motors[0]->getFilteredRPM(), dt);\n        float s1 = _pids[1]->compute(_targetRPMs[1], _motors[1]->getFilteredRPM(), dt);\n        float s2 = _pids[2]->compute(_targetRPMs[2], _motors[2]->getFilteredRPM(), dt);",
  "risk": "If an encoder disconnects or a motor stalls under load while the operator commands nonzero, `getFilteredRPM()` reports ~0 while `_targetRPMs[i]` stays at the command. Error = setpoint − 0 = full setpoint; PID integrator winds up to its anti-windup clamp (output/Ki), output saturates at +1.0; that wheel goes to full PWM. The other two wheels keep tr …[truncated]",
  "suggested_fix": "Add a per-motor fault detector in `FIT0186Motor`: if commanded |PWM| > threshold for >N ms and |measured RPM| ~= 0, raise a fault flag. `OmniDrivetrain::update` then either calls `stop()` on the offen …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C11-P0-04",
  "title": "OTA path does not actively brake motors; only stops the producer",
  "file": "src/main_robot.cpp",
  "line_range": "104-110",
  "quote": "ArduinoOTA.onStart([]() {\n        // Stop accepting new commands. Once the producer is gone, the staleness\n        // ramp in controlTask will drive motors to zero within 200 ms before\n        // the firmware actually overwrites flash.\n        Serial.println(\"[ota] update starting; halting teleop\");\n        if (g_producer) g_producer->stop();\n    } …[truncated]",
  "risk": "(a) The onStart callback only stops the producer; it does NOT call `g_controller->stop()` or `g_drivetrain.stop()` or zero `_targetRPMs`. The control loop continues running through the ramp and reaches PWM=0 (coast) but the wheels are not actively braked. (b) `g_producer->stop()` blocks up to 1 s on `_exitSemaphore` (`WebSocketCommandProducer.cpp:7 …[truncated]",
  "suggested_fix": "In `onStart` also call `g_controller->stop()` (which calls `OmniDrivetrain::stop()` → `motor->brake()` on all three → L298N HIGH/HIGH = active short). In `onError` re-arm: `g_producer->start()` again. …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C11-P0-05",
  "title": "OmniDrivetrain::stop() is dead code — no runtime trigger reaches active-brake",
  "file": "src/main_robot.cpp",
  "line_range": "144-206",
  "quote": "void controlTask(void* /*arg*/) {\n    // Subscribe to the Task Watchdog. If this loop hangs (e.g. I2C bus\n    // stretch on the IMU), TWDT panics and resets — preferable to motors\n    // stuck at last PWM with no operator recovery.\n    esp_task_wdt_add(NULL);",
  "risk": "Grep across `src/` confirms `OmniDrivetrain::stop()` is invoked only by `PassthroughDrivetrainController::stop()` and the off-runtime `main_drivetrain_test.cpp`. `PassthroughDrivetrainController::stop()` has zero callers in `main_robot.cpp` (T1-T8 all end in PWM=0 coast, not `motor->brake()`). Net effect: `Motor::brake()` and the L298N HIGH/HIGH ac …[truncated]",
  "suggested_fix": "Wire `g_controller->stop()` into: (1) OTA onStart (after producer.stop()), (2) post-ramp once `age >= STALENESS_TIMEOUT_MS` AND `have_seen_fresh` (latch the brake instead of coast), (3) a TWDT pretrig …[truncated]",
  "verification_status": "verified"
}
```

### Cell 12

```json
{
  "id": "C12-P0-01",
  "title": "fit0186::ENCODER_CPR disagrees with motor design spec (2800 vs 700)",
  "file": "src/motor/constants/FIT0186.h",
  "line_range": "11-11",
  "quote": "inline constexpr uint16_t ENCODER_CPR = 2800;",
  "risk": "RPM = (deltaCount / encoderCPR) * 60/dt. If CPR is 4× the true value, computed RPM is 4× too low; if CPR is too low, computed RPM is too high. PID measurement is in RPM-space, so the loop is mistuned by a factor of 4. Saturation in OmniKinematics (capped at MAX_RPM=251) will allow demanded RPMs the PID never actually reaches (or vice versa). The ha …[truncated]",
  "suggested_fix": "Update the spec to clarify that 700 is the gearbox CPR in single-edge counting mode and 2800 = 700×4 is what the FIT0186Motor sees through attachFullQuad. Add a comment in FIT0186.h referencing the de …[truncated]",
  "spec_file": "docs/superpowers/specs/2026-04-23-fit0186-motor-design.md",
  "spec_line_range": "176-179",
  "spec_quote": "inline constexpr uint16_t ENCODER_CPR = 700;  // counts per revolution at gearbox output",
  "verification_status": "verified"
}
```

```json
{
  "id": "C12-P0-02",
  "title": "Motor spec says brake() shorts terminals via BTS7960; L298N brake() also writes 255/255 (not documented as supported)",
  "file": "src/motor/driver/L298NDriver.cpp",
  "line_range": "39-42",
  "quote": "void L298NDriver::brake() {\n    analogWrite(_pins.fwd, 255);\n    analogWrite(_pins.rev, 255);\n}",
  "risk": "The motor-design spec is written exclusively against the BTS7960 — the Driver/L298N abstraction does not appear in the spec. The runtime build (`[env:robot]`) excludes BTS7960Driver.cpp and uses L298NDriver instead (main_robot.cpp:58-60). The L298N brake (both pins HIGH = output enable on both half-bridges) is documented in the driver as 'short mot …[truncated]",
  "suggested_fix": "Either update the motor-design spec to describe both concrete drivers (L298N and BTS7960) including their brake/coast semantics, or amend it to note that the production driver was changed from BTS7960 …[truncated]",
  "spec_file": "docs/superpowers/specs/2026-04-23-fit0186-motor-design.md",
  "spec_line_range": "104-106",
  "spec_quote": "setSpeed(0) — coast (both H-bridge outputs LOW, motor disconnected)\n- brake() — active braking (both H-bridge outputs HIGH, terminals shorted)",
  "verification_status": "verified"
}
```

```json
{
  "id": "C12-P0-03",
  "title": "Drivetrain::update spec signature has no parameter; code takes float dt",
  "file": "src/drivetrain/OmniDrivetrain.h",
  "line_range": "25-33",
  "quote": "    void update(float dt) override {\n        for (auto* m : _motors) m->update();\n\n        float s0 = _pids[0]->compute(_targetRPMs[0], _motors[0]->getFilteredRPM(), dt);",
  "risk": "Spec deferred dt handling to the PID class ('PID receives dt externally or computes it internally — that's a PID design decision'). Code: PID::compute takes dt, OmniDrivetrain::update takes dt, and the caller (controlTask in main_robot.cpp:190-191) passes a hard-coded nominal `CONTROL_PERIOD_MS/1000 = 0.010f` rather than measuring actual elapsed ti …[truncated]",
  "suggested_fix": "Update the omni-drivetrain spec to describe the dt plumbing decision and to clarify that the caller passes nominal dt. Optionally consider computing actual dt in controlTask via micros() and forwardin …[truncated]",
  "spec_file": "docs/superpowers/specs/2026-05-06-omni-drivetrain-design.md",
  "spec_line_range": "110-112",
  "spec_quote": "    void drive(const BodyVelocity& velocity) override;\n    void update() override;\n    void stop() override;",
  "verification_status": "verified"
}
```

```json
{
  "id": "C12-P0-04",
  "title": "drivetrain-test-procedure says strafe puts motor 0 at largest magnitude; kinematics derivation says |ω₀|=vy/(rcosα) while |ω₁|=|ω₂|=0.5×vy/( …[truncated]",
  "file": "src/drivetrain/OmniKinematics.h",
  "line_range": "32-34",
  "quote": "        float w0 = scale * (-SIN_0   * v.vx - COS_0   * v.vy - R * v.omega);\n        float w1 = scale * (-SIN_120 * v.vx - COS_120 * v.vy - R * v.omega);\n        float w2 = scale * (-SIN_240 * v.vx - COS_240 * v.vy - R * v.omega);",
  "risk": "Magnitudes and signs match the derivation in omni-kinematics-derivation.md (motor 0 is 2× the magnitude of motors 1 and 2 on pure strafe). However the verification bullet in the design spec is editorial only ('contribute'). The test (`test_omni_kinematics.cpp`) is what locks the invariant. Drift here is mostly that the spec's verification bullet do …[truncated]",
  "suggested_fix": "Tighten the spec verification bullet to say 'motor 0 magnitude == 2 × motor 1 magnitude == 2 × motor 2 magnitude on pure strafe, with motor 0 sign opposite to motors 1/2' and confirm a unit test cover …[truncated]",
  "spec_file": "docs/superpowers/specs/2026-05-06-omni-drivetrain-design.md",
  "spec_line_range": "193-194",
  "spec_quote": "- `OmniKinematics`: pure strafe → motor 0 active, motors 1 and 2 contribute",
  "verification_status": "verified"
}
```

## P1 — verified

### Cell 1

```json
{
  "id": "C1-P1-01",
  "title": "PID deadband only fires when setpoint ≈ 0 — asymmetric zero-snap, integrator builds at non-zero setpoints below deadband",
  "file": "src/control/PID.h",
  "line_range": "22-25",
  "quote": "if (_deadband > 0.0f && std::abs(setpoint) < 1e-6f && std::abs(measurement) < _deadband) {\n            reset();\n            return 0.0f;\n        }",
  "risk": "The deadband (`5.0f` RPM in main_robot.cpp:66-68) gates *only* when setpoint is essentially zero AND measurement is below the deadband. For a non-zero setpoint smaller than the deadband (e.g. `setpoint = 2 RPM`, measurement = 0), no deadband applies — the PID computes error = 2 RPM, kp·error = 0.01, integral grows. Because `getFilteredRPM()` smooth …[truncated]",
  "suggested_fix": "Apply the deadband symmetrically: if `|setpoint| < deadband`, force setpoint = 0 *and* freeze the integrator (do not accumulate while in deadband). Or: gate the integrator on `|error| > some_threshold …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C1-P1-02",
  "title": "PID anti-windup divides by _ki without checking sign; flipped output range gives inverted clamp",
  "file": "src/control/PID.h",
  "line_range": "32-37",
  "quote": "if (_ki != 0.0f) {\n            float integralMax = _outputMax / _ki;\n            float integralMin = _outputMin / _ki;\n            if (_integral > integralMax) _integral = integralMax;\n            if (_integral < integralMin) _integral = integralMin;\n        }",
  "risk": "If a caller ever constructs a PID with negative `_ki` (e.g. a buggy refactor or a controller used in reverse), the integral bounds invert silently (integralMax becomes the lower bound). No assert protects against this. Same for `_outputMin > _outputMax`. This is also a textbook 'back-calculation anti-windup' approximation that ignores the kp and kd …[truncated]",
  "suggested_fix": "Constructor: `BB8_ASSERT(_outputMin <= _outputMax && _ki >= 0)`. Switch to a clamping anti-windup on the *output* (only accumulate `_integral += error*dt` if doing so wouldn't push output past clamps) …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C1-P1-03",
  "title": "OmniKinematics saturation preserves direction, but reads of maxRPM as motor-side cap conflate motor no-load with motor PID limit",
  "file": "src/drivetrain/OmniKinematics.h",
  "line_range": "45-50",
  "quote": "if (maxAbs > _config.maxRPM) {\n            float factor = _config.maxRPM / maxAbs;\n            for (int i = 0; i < 3; i++) {\n                rpms[i] *= factor;\n            }\n        }",
  "risk": "Direction preservation under saturation is correct (the central safety property — confirmed). However, `_config.maxRPM` is set to FIT0186 no-load RPM (251) — the *physical* no-load speed. The PID setpoint can therefore demand 251 RPM, but at that target the motor is at zero torque margin — any disturbance produces an unrecoverable lag and the PID d …[truncated]",
  "suggested_fix": "Reduce `MAX_RPM` to ~80% of no-load (200 RPM) and rename to `maxControllableRPM`. Add a clamp at the `_targetRPMs` write site in `OmniDrivetrain::drive()`.",
  "verification_status": "verified"
}
```

```json
{
  "id": "C1-P1-04",
  "title": "L298N vs BTS7960 divergence at zero: coast-vs-disable + brake-vs-stop, plus implicit Enable/Disable cycling",
  "file": "src/motor/driver/BTS7960Driver.cpp",
  "line_range": "20-30",
  "quote": "if (value == 0.0f) {\n        _hbridge.Disable();\n        return;\n    }\n\n    _hbridge.Enable();\n    uint8_t pwm = static_cast<uint8_t>(std::abs(value) * 255.0f);\n    if (pwm == 0) {\n        _hbridge.Disable();\n        return;\n    }",
  "risk": "L298N coast (setOutput(0)) writes 0/0 to PWM pins — instant freewheel, no h-bridge state change. BTS7960 calls `_hbridge.Disable()` which drops the EN lines low — also coast, but with a high-Z transient as EN settles. Worse: every nonzero call re-enters `_hbridge.Enable()` (line 25). Toggling Enable at 100 Hz wears the MOSFETs' gate driver and prod …[truncated]",
  "suggested_fix": "(a) Cache state in the driver and only call Enable/Disable on transition. (b) Replace `value == 0.0f` with `std::abs(value) < 1.0f/256.0f`. (c) Document brake semantics divergence as part of the `Driv …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C1-P1-05",
  "title": "FIT0186Motor encoder dt uses unsigned `micros()` subtraction — wrap-safe, but cast through float loses precision near 70-min wrap",
  "file": "src/motor/FIT0186Motor.h",
  "line_range": "37-41",
  "quote": "unsigned long now = micros();\n        float dtSeconds = static_cast<float>(now - _lastUpdateMicros) / 1'000'000.0f;\n\n        if (dtSeconds <= 0.0f) return;",
  "risk": "Unsigned subtraction is wrap-safe (test_micros_overflow verifies this at ULONG_MAX). However: when `now - _lastUpdateMicros` is converted to float, precision is fine up to ~16.7M (24-bit mantissa) — i.e. up to 16.7 seconds of skipped updates before float dt loses microsecond resolution. Normal 10 ms operation is far below. But if the control task e …[truncated]",
  "suggested_fix": "Switch to `int64_t` dt math (microsecond integer counts) and divide only at the end. Initialize `_lastCount` *before* `_lastUpdateMicros` in begin() to guarantee no stale-tick race.",
  "verification_status": "verified"
}
```

```json
{
  "id": "C1-P1-06",
  "title": "MovingAverage<5> survives motor stop with stale samples — first PID tick after re-drive sees lagged measurement",
  "file": "src/motor/FIT0186Motor.h",
  "line_range": "49-51",
  "quote": "_filter.push(_rawRPM);\n        _lastCount = count;\n        _lastUpdateMicros = now;",
  "risk": "After `OmniDrivetrain::stop()` calls `motor->brake()` and `_pids[i]->reset()`, the PID's integral and prevMeasurement are cleared (PID.h:55-60), but `MovingAverage<5> _filter` is NOT reset (FIT0186Motor exposes no reset hook). Consequently, when `update()` resumes after a stop, `getFilteredRPM()` returns the average of 4 stale + 1 fresh samples for …[truncated]",
  "suggested_fix": "Add `Motor::reset()` / `FIT0186Motor::resetFilter()` and call it from `OmniDrivetrain::stop()` alongside `pid->reset()`.",
  "verification_status": "verified"
}
```

```json
{
  "id": "C1-P1-07",
  "title": "OmniKinematics rejects tiltAngle == π/2 but not negative or > π/2 tilts",
  "file": "src/drivetrain/OmniKinematics.h",
  "line_range": "15-17",
  "quote": "BB8_ASSERT(std::abs(cosf(config.tiltAngle)) > 1e-6f,\n                   \"OmniKinematics: tiltAngle too close to pi/2\");",
  "risk": "BB8_ASSERT is a no-op in production. A tiltAngle of, say, π (180°) gives cos = -1, scale = -1/r — silently inverts all wheel directions. Negative tilt gives same magnitude as positive (cos is even), but the geometry only makes physical sense for `0 ≤ α < π/2`. A misconfigured constant is unchecked at runtime.",
  "suggested_fix": "Replace BB8_ASSERT with a runtime range check (or compile-time `static_assert` since RobotConstants::TILT_ANGLE is constexpr).",
  "verification_status": "verified"
}
```

```json
{
  "id": "C1-P1-08",
  "title": "OmniDrivetrain::drive() does not validate finiteness of incoming BodyVelocity",
  "file": "src/drivetrain/OmniDrivetrain.h",
  "line_range": "18-23",
  "quote": "void drive(const BodyVelocity& velocity) override {\n        auto rpms = _kinematics.toWheelRPMs(velocity);\n        for (int i = 0; i < 3; i++) {\n            _targetRPMs[i] = rpms[i];\n        }\n    }",
  "risk": "No check that `velocity.vx/vy/omega` are finite. Combined with C1-P0-01: any non-finite WS frame flows through. Additionally `_targetRPMs` is stored as the kinematics output even if NaN — subsequent `update()` calls feed NaN into PID, which (in production where BB8_ASSERT is off) returns NaN-ish output through `setSpeed`.",
  "suggested_fix": "Sanitize at the boundary: `if (!std::isfinite(v.vx) || ...) return;` or coerce to 0.",
  "verification_status": "verified"
}
```

### Cell 2

```json
{
  "id": "C2-P1-01",
  "title": "Staleness path zeroes setpoint but never brakes — coast on slope = drift",
  "file": "src/main_robot.cpp",
  "line_range": "170-187",
  "quote": "BodyVelocity drive_cmd{0.0f, 0.0f, 0.0f};",
  "risk": "On WS silence > 200 ms, `drive_cmd` is zero but controller.update still runs → PIDs against 0-RPM setpoint → L298N at PWM=0 = *coast* (CONTEXT §4). main_robot.cpp:6-9 says 'hard stop' but the path is hard-coast. `OmniDrivetrain::stop()` (brake + reset) is never invoked at runtime (CONTEXT §9.11).",
  "suggested_fix": "Once `age > STALENESS_TIMEOUT_MS` (or `!have_seen_fresh`), call `g_controller->stop()` instead of feeding zeros. Latch a `was_stopped` flag so it isn't called every tick.",
  "verification_status": "verified"
}
```

```json
{
  "id": "C2-P1-02",
  "title": "WS task alive but starved of frames — heartbeat OK, no commands, no producer fault",
  "file": "src/input/WebSocketCommandProducer.cpp",
  "line_range": "99-115",
  "quote": "esp_task_wdt_add(NULL);",
  "risk": "ws_prod pets TWDT regardless of frame flow; heartbeat keeps `_connected==true`. If the client hangs but TCP stays alive, `connected()` is true and no commands arrive. Control's staleness net catches it, but producer never disconnects a silent peer to free the single-client slot.",
  "suggested_fix": "Track `_lastFrameMs`; if >2 s since last text frame on active client, disconnect them. Log a one-shot warning from control when `now - last_fresh_ms > STALENESS_TIMEOUT_MS`.",
  "verification_status": "verified"
}
```

```json
{
  "id": "C2-P1-03",
  "title": "Identical-command repeat is treated as fresh — stuck stream never goes stale",
  "file": "src/main_robot.cpp",
  "line_range": "162-168",
  "quote": "if (fresh.has_value()) {",
  "risk": "`last_fresh_ms` resets on any latch hit, regardless of value change. A buggy client retransmitting `0.5,0,0` at 100 Hz keeps the loop 'fresh' indefinitely with no liveness signal in the wire protocol (no seq, no timestamp). All-zero stuck stream is safe but accidental; non-zero stuck stream is hazardous.",
  "suggested_fix": "Add a monotonic seq or wall-time field: `\"vx,vy,omega,seq\"`. Reject frames where seq did not advance.",
  "verification_status": "verified"
}
```

```json
{
  "id": "C2-P1-04",
  "title": "Quaternion consumed without normalization or finiteness check",
  "file": "src/imu/BNO055IMU.h",
  "line_range": "39-43",
  "quote": "r.orientation = Quat{static_cast<float>(q.w()),",
  "risk": "Direct double→float cast, no normalize, no finiteness check. `test_unnormalized_quaternion_not_renormalized` (test_bno055_imu.cpp:214-220) actively asserts unnormalized quats pass through. BNO055 can emit `(0,0,0,0)` after begin/cal glitch. A future controller doing `acos`/`atan2` will divide by `||q||=0` or produce NaN. Today's passthrough dodges …[truncated]",
  "suggested_fix": "In `read()` compute `n=sqrt(w²+x²+y²+z²)`; if `n<1e-3 || !isfinite(n)` flag invalid or substitute identity. Otherwise scale.",
  "verification_status": "verified"
}
```

```json
{
  "id": "C2-P1-05",
  "title": "`isCalibrated()` exists but no consumer gates on it",
  "file": "src/imu/BNO055IMU.h",
  "line_range": "82-84",
  "quote": "bool isCalibrated() {",
  "risk": "Calibration status is in `IMUReading` and exposed via `isCalibrated()`, but neither `main_robot.cpp` nor the controller consults it. A 0/3/2/1 reading is treated identically to 3/3/3/3. Auto-save logic is correct; consumer is missing.",
  "suggested_fix": "Require `gyro >= 2` before IMU-based control modes; propagate validity into `IMUReading::valid`.",
  "verification_status": "verified"
}
```

```json
{
  "id": "C2-P1-06",
  "title": "Parser tolerance is wider than tested — whitespace/sci-notation/hex-float not pinned",
  "file": "src/input/CommandFrameParser.h",
  "line_range": "46-55",
  "quote": "if (end == p || *end != ',') return {{}, FrameParseError::InvalidVx};",
  "risk": "`std::strtof` per C standard skips leading whitespace, accepts `+`/`-`, scientific notation, hex floats. None is wrong, but none is tested; a future stricter rewrite regresses silently. Embedded null short-circuits strtof; current behavior (parse error at vx) is correct but undocumented.",
  "suggested_fix": "Pin the grammar with explicit tests for: leading whitespace, '+1', '1e3', '0x1p2', embedded null, multi-line, '-0', locale decimal.",
  "verification_status": "verified"
}
```

```json
{
  "id": "C2-P1-07",
  "title": "`MockCommandProducer` flags non-atomic — interface contract ambiguous on threading",
  "file": "src/input/MockCommandProducer.h",
  "line_range": "17-30",
  "quote": "void start() override { _started = true; }",
  "risk": "Real WS producer uses `std::atomic<bool>` on `_connected`/`_running`. Mock uses plain `bool`. Any future test that drives the mock from a separate task inherits a data race the interface never warned about.",
  "suggested_fix": "Use `std::atomic<bool>` in the mock; or document `connected()` as snapshotting + cross-thread-safe on the interface.",
  "verification_status": "verified"
}
```

```json
{
  "id": "C2-P1-08",
  "title": "`CommandProducer` exposes no way to surface invalid frames",
  "file": "src/input/CommandProducer.h",
  "line_range": "11-18",
  "quote": "virtual bool connected() const = 0;",
  "risk": "Interface offers only `connected()`. WS impl counts `_parseFailCount` (WebSocketCommandProducer.cpp:150) but never publishes it. Client typo storm at 100 Hz is indistinguishable from 'no signal' to the app: latch empty → staleness → coast.",
  "suggested_fix": "Add `uint32_t parseFailures() const` and `uint32_t framesSinceLastValid() const`; control loop logs/raises on first nonzero observation.",
  "verification_status": "verified"
}
```

### Cell 3

```json
{
  "id": "C3-P1-01",
  "title": "Real WiFi/OTA credentials in plaintext working tree (gitignored but present on dev disk)",
  "file": "src/config/wifi_credentials.h",
  "line_range": "8-10",
  "quote": "#define WIFI_SSID     ",
  "risk": "`.gitignore` correctly ignores `src/config/wifi_credentials.h` and `git ls-files` confirms it is NOT tracked at f8ca95f — so this is not a public secrets leak. However: the file does contain a live SSID/password and the OTA password (a 3-character string, see `00-CONTEXT.md §9`). Risks: (1) trivial OTA password lets anyone on the trusted LAN re-fla …[truncated]",
  "suggested_fix": "(a) Rotate the OTA password to ≥16 random chars and use the same env-var pattern already documented in `wifi_credentials.example.h`. (b) Move WIFI_PASSWORD to the same env-var mechanism (compile-time …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C3-P1-02",
  "title": "Arduino `loop()` (loopTask) is not subscribed to TWDT — silent hang of WiFi reboot watchdog possible",
  "file": "src/main_robot.cpp",
  "line_range": "252-264",
  "quote": "void loop() {\n    // Housekeeping: OTA poll + WiFi reboot watchdog. Control runs in dedicated\n    // tasks. Tick at 10 Hz so OTA initiate requests are answered promptly.\n    ArduinoOTA.handle();",
  "risk": "Only `controlTask` (main_robot.cpp:148) and ws_prod (WebSocketCommandProducer.cpp:103) call `esp_task_wdt_add(NULL)`. The Arduino loopTask runs `ArduinoOTA.handle()` and the WiFi-offline reboot watchdog (lines 257-262). If `ArduinoOTA.handle()` ever blocks indefinitely (e.g. malformed packet handling in the library), the 30 s WiFi-offline reboot wa …[truncated]",
  "suggested_fix": "Either subscribe loopTask to TWDT and reset inside `loop()` (the 100 ms `vTaskDelay` gives ample slack against a 1 s timeout), OR replace the WiFi reboot watchdog with an independent `esp_timer` that …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C3-P1-03",
  "title": "WiFi event callback writes `g_lastWifiConnectedMs` non-atomically across cores",
  "file": "src/main_robot.cpp",
  "line_range": "83-98",
  "quote": "volatile uint32_t g_lastWifiConnectedMs = 0;\n\nvoid onWifiEvent(WiFiEvent_t event) {",
  "risk": "`g_lastWifiConnectedMs` is `volatile uint32_t` (not `std::atomic`). It is written in two contexts: (a) `onWifiEvent` which arduino-esp32 dispatches on its own event task (Core 0), and (b) `loop()` (Core 1) at line 258. It is read in `loop()` at line 259. `volatile` on a 32-bit aligned word is *probably* atomic on xtensa, but the C++ standard doesn' …[truncated]",
  "suggested_fix": "Declare `std::atomic<uint32_t> g_lastWifiConnectedMs{0}`; access with `.load(std::memory_order_relaxed)` and `.store(...)`.",
  "verification_status": "verified"
}
```

```json
{
  "id": "C3-P1-04",
  "title": "Pin map docs disagree with `pins.h` after the harness remap (f8ca95f)",
  "file": "docs/motor-spin-test-procedure.md",
  "line_range": "16-30",
  "quote": "| 2     | B             | GPIO 18   | GPIO 19   |",
  "risk": "`docs/motor-spin-test-procedure.md` line 18 lists motor 2 at GPIO 18/19; `src/config/pins.h:14` has `MOTOR2_PINS = {.fwd = 25, .rev = 26}`. Encoder 1 in docs (line 29) is `A=GPIO 4, B=GPIO 2`; `pins.h:17` has `A=2, B=4` (swapped). Encoder 2 in docs (line 30) is `A=GPIO 39, B=GPIO 36`; `pins.h:18` has `A=36, B=39` (also swapped). Separately, `docs/i …[truncated]",
  "suggested_fix": "Update all three docs (motor-spin, drivetrain-test, imu-test) to match `pins.h`. Replace the hardcoded pin array in `main_blink.cpp` with `MOTOR0_PINS`/`MOTOR1_PINS`/`MOTOR2_PINS` from `pins.h` so bli …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C3-P1-05",
  "title": "Asserts guard invariants that have NO runtime check on hardware (none of robot/motor_test/drivetrain_test set -DBB8_DEBUG)",
  "file": "src/config/debug.h",
  "line_range": "25-27",
  "quote": "#else\n#define BB8_ASSERT(cond, msg) ((void)0)\n#endif",
  "risk": "Only `[env:native]` sets `-DBB8_DEBUG` (`platformio.ini:83`). In `[env:robot]` and friends, `BB8_ASSERT` is `((void)0)`. The invariants thereby unprotected on production hardware include: (a) `L298NDriver::setOutput`'s `[-1,1]` range check — if any caller passes a NaN or out-of-range float, `static_cast<uint8_t>(std::abs(value) * 255.0f)` wraps mod …[truncated]",
  "suggested_fix": "Either define a separate `BB8_RUNTIME_CHECK(cond, action)` that, on hardware, clamps inputs and increments a fault counter instead of asserting; OR enable `-DBB8_DEBUG` for `[env:robot]` (the abort ha …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C3-P1-06",
  "title": "`MovingAverage<N>` is not safe for concurrent push/read across cores; underflow on first samples",
  "file": "src/util/MovingAverage.h",
  "line_range": "16-29",
  "quote": "void push(float value) {\n        _buffer[_index] = value;\n        _index = (_index + 1) % N;\n        if (_count < N) ++_count;\n    }\n\n    float average() const {",
  "risk": "Two issues. (1) Concurrency: `_buffer[_index] = value` then `_index = ...` then `_count = ...` is three independent writes on plain ints/floats. `average()` reads `_count` then iterates `_buffer[0..count)`. If a reader on Core A interleaves with a writer on Core B, it can read `_count==N` but a `_buffer[i]` that is in the middle of being written, r …[truncated]",
  "suggested_fix": "(1) Add a header comment + `static_assert` or runtime doc that `MovingAverage<N>` is single-threaded — and consider adding a `noexcept` + `portMUX_TYPE` variant for cross-core use. (2) Provide an expl …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C3-P1-07",
  "title": "BNO055 `begin()` failure path is an unbounded `ESP.restart()` boot loop with no backoff",
  "file": "src/main_robot.cpp",
  "line_range": "221-224",
  "quote": "if (!g_imu.begin()) {\n        Serial.println(\"FATAL: BNO055 init failed — restarting.\");\n        ESP.restart();\n    }",
  "risk": "Same applies to `connectWifi()`'s 30 s timeout → restart at line 134. If the IMU's I2C bus is permanently stuck (e.g. the BNO055 is unpowered or a wire is cut), the board enters a tight 200 ms reboot loop: `Serial.begin` + `delay(200)` + `Wire.begin` + 3× motor.begin + `g_imu.begin()` (probably a multi-second blocking timeout inside Adafruit_BNO055 …[truncated]",
  "suggested_fix": "Implement a degraded mode: if `g_imu.begin()` fails N times consecutively (track via RTC slow-mem counter), still bring up WiFi + WebSocket + OTA so the operator can re-flash. Run the control task wit …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C3-P1-08",
  "title": "PassthroughDrivetrainController::stop() exists but is never wired to OTA, WS disconnect, or IMU loss",
  "file": "src/main_robot.cpp",
  "line_range": "104-110",
  "quote": "if (g_producer) g_producer->stop();",
  "risk": "CONTEXT §9 #11 already calls this out as an observation; for safety lens it bumps to P1. `PassthroughDrivetrainController::stop()` would call `drivetrain.stop()` which actively brakes motors and resets PIDs (CONTEXT §6). Today every shutdown path (OTA, staleness, WS disconnect) zeroes the *setpoint* and lets PIDs settle. At zero setpoint with `dead …[truncated]",
  "suggested_fix": "OTA `onStart`: call `g_controller->stop()` *before* `g_producer->stop()`. Add a `safetyStop()` entry point on the controller that also re-enables brake mode.",
  "verification_status": "verified"
}
```

```json
{
  "id": "C3-P1-09",
  "title": "Dead entrypoints `main_original.cpp` + `main_blink.cpp` are present in src/ and rely on exclude-lists in every env",
  "file": "platformio.ini",
  "line_range": "7-47",
  "quote": "-<main_original.cpp> -<main_blink.cpp>",
  "risk": "Every env's `build_src_filter` explicitly excludes both files. The base env `[env:wemos_d1_uno32]` is the only one without all five test mains excluded — it has no `setup()`/`loop()` to link, so it would fail at link time and that fact is the only line of defense against accidental motor-spin builds. If a new contributor copies an env without `-<ma …[truncated]",
  "suggested_fix": "Move `main_blink.cpp` and `main_original.cpp` to `tools/` or `src/_archive/` outside the default `src/` filter scope. Or wrap their `setup()/loop()` in `#ifdef BB8_BLINK_TEST` so an accidental include …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C3-P1-10",
  "title": "Heap-allocated globals (`g_latch`, `g_producer`, `g_controller`) are nullable but dereferenced unconditionally in controlTask",
  "file": "src/main_robot.cpp",
  "line_range": "79-82",
  "quote": "CommandLatch<BodyVelocity>*      g_latch      = nullptr;\n    WebSocketCommandProducer*        g_producer   = nullptr;\n    PassthroughDrivetrainController* g_controller = nullptr;",
  "risk": "`controlTask` (line 162: `auto fresh = g_latch->read();`, line 187: `g_controller->update(...)`) dereferences these without a null check. They're set on the same task that creates `controlTask` *before* the task is created, so this is safe TODAY — but the pattern is fragile. If a future refactor moves task creation earlier (e.g. to fix C3-P0-02), t …[truncated]",
  "suggested_fix": "Either make them static (not heap) and construct in `setup()` via placement-new, OR add `if (!g_latch || !g_controller) { vTaskDelay(...); continue; }` at the top of the loop, OR pass pointers as the …[truncated]",
  "verification_status": "verified"
}
```

### Cell 4

```json
{
  "id": "C4-P1-04",
  "title": "FIT0186Motor::update() read-modify-write on _lastCount is not atomic with respect to multiple callers — abstraction does not enforce single- …[truncated]",
  "file": "src/motor/FIT0186Motor.h",
  "line_range": "37-52",
  "quote": "int64_t count = _encoder.getCount();\n        int64_t deltaCount = count - _lastCount;\n\n        _rawRPM = (static_cast<float>(deltaCount) / static_cast<float>(_encoderCPR))\n                  * (60.0f / dtSeconds);\n\n        _filter.push(_rawRPM);\n        _lastCount = count;\n        _lastUpdateMicros = now;",
  "risk": "",
  "verification_status": "verified"
}
```

```json
{
  "id": "C4-P1-05",
  "title": "Two independently sourced dts — encoder uses micros() inside update(), PID uses caller-supplied dt — under jitter they diverge",
  "file": "src/motor/FIT0186Motor.h",
  "line_range": "38-47",
  "quote": "unsigned long now = micros();\n        float dtSeconds = static_cast<float>(now - _lastUpdateMicros) / 1'000'000.0f;\n\n        if (dtSeconds <= 0.0f) return;\n\n        int64_t count = _encoder.getCount();\n        int64_t deltaCount = count - _lastCount;\n\n        _rawRPM = (static_cast<float>(deltaCount) / static_cast<float>(_encoderCPR)) …[truncated]",
  "risk": "",
  "verification_status": "verified"
}
```

```json
{
  "id": "C4-P1-06",
  "title": "PID derivative-on-measurement responds to staleness ramp as a real velocity signal — phantom D kick on every silence event",
  "file": "src/control/PID.h",
  "line_range": "39-45",
  "quote": "float derivative = 0.0f;\n        if (!_firstCompute) {\n            derivative = -(measurement - _prevMeasurement) / dt;\n        }",
  "risk": "",
  "verification_status": "verified"
}
```

```json
{
  "id": "C4-P1-07",
  "title": "PID integral anti-windup clamp uses outputMax/Ki, but compute() then clamps output again — wind-down asymmetry",
  "file": "src/control/PID.h",
  "line_range": "31-48",
  "quote": "if (_ki != 0.0f) {\n            float integralMax = _outputMax / _ki;\n            float integralMin = _outputMin / _ki;\n            if (_integral > integralMax) _integral = integralMax;\n            if (_integral < integralMin) _integral = integralMin;\n        }",
  "risk": "",
  "verification_status": "verified"
}
```

```json
{
  "id": "C4-P1-08",
  "title": "Encoder pins 34/35/36/39 are input-only (no internal pull-ups) — encoder relies on external open-collector pull-ups or active hot signals",
  "file": "src/config/pins.h",
  "line_range": "16-18",
  "quote": "inline constexpr EncoderPins MOTOR0_ENCODER_PINS = {.a = 34, .b = 35};\n\ninline constexpr EncoderPins MOTOR1_ENCODER_PINS = {.a = 2, .b = 4};\ninline constexpr EncoderPins MOTOR2_ENCODER_PINS = {.a = 36, .b = 39};",
  "risk": "",
  "verification_status": "verified"
}
```

```json
{
  "id": "C4-P1-09",
  "title": "L298NDriver::brake() (both pins HIGH = 255/255) is functionally equivalent to coast at 1 kHz on a real L298N — H-bridge sees 100% duty on bo …[truncated]",
  "file": "src/motor/driver/L298NDriver.cpp",
  "line_range": "39-42",
  "quote": "void L298NDriver::brake() {\n    analogWrite(_pins.fwd, 255);\n    analogWrite(_pins.rev, 255);\n}",
  "risk": "",
  "verification_status": "verified"
}
```

### Cell 5

```json
{
  "id": "C5-P1-01",
  "title": "Adafruit_BNO055::getQuat/getVector/getCalibration called inline in control task: 4 sequential I2C round-trips on the 10 ms hot path",
  "file": "src/main_robot.cpp",
  "line_range": "",
  "quote": "IMUReading imu_reading = g_imu.read();",
  "risk": "Eats budget. Three consequences: (a) jitter on motor PID update (PID dt is hard-coded 0.010f, but the actual wall-clock work happens later in the slot), (b) any I2C stretch from the BNO055 (the chip is known to stretch up to 1 ms on its slow MCU side) eats into the next tick and can trip TWDT (1 s) only on egregious stalls but routinely chews margi …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C5-P1-02",
  "title": "BNO055 calibration save (NVS putBytes) runs inline inside read() on the control task",
  "file": "src/imu/BNO055IMU.h",
  "line_range": "",
  "quote": "if (!_savedThisBoot && _bno.isFullyCalibrated()) {\n                saveCalibration();\n                _savedThisBoot = true;\n            }",
  "risk": "On the tick this fires, the control task overshoots its 10 ms deadline by 1-4× the period. PID dt remains the hard-coded 0.010f but actual wall-clock between motor updates is ~50 ms — that one cycle the wheels run open-loop at the last command for ~50 ms. If calibration completion happens mid-motion this is a single visible glitch (motor 'kick' or …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C5-P1-03",
  "title": "ws_prod task polls WebSocketsServer::loop() at ~1 ms via vTaskDelay (not vTaskDelayUntil) — sleep is constant, not period",
  "file": "src/input/WebSocketCommandProducer.cpp",
  "line_range": "",
  "quote": "while (_running.load()) {\n        if (_server) _server->loop();\n        esp_task_wdt_reset();\n        vTaskDelay(pdMS_TO_TICKS(kLoopDelayMs));\n    }",
  "risk": "Two issues. (1) Latency: a TEXT frame arriving immediately after server->loop() returns has to wait ~1 ms before being parsed and latched. Combined with the 10 ms control tick this adds 1 ms to teleop end-to-end latency — measurable but small. (2) Heartbeat timing is owned by `_server->loop()` (the library timer-checks heartbeat inside its pump). I …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C5-P1-05",
  "title": "CommandLatch is length-1 with xQueueOverwrite — at producer rate > control rate, all but the last frame between ticks are silently dropped ( …[truncated]",
  "file": "src/input/WebSocketCommandProducer.cpp",
  "line_range": "",
  "quote": "_latch.write(result.value);",
  "risk": "By design — newest command wins. The risk is only if the producer believes 'send it and it's queued' semantics. Since the wire protocol is conceptually a level signal (latest body-frame velocity), drop is correct. No code change needed but the operator-side doc should state 'commands are coalesced to ~100 Hz on the robot side; client-side rate abov …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C5-P1-06",
  "title": "Below the latch: TCP socket buffering is owned by lwIP and the WS library — bounded but not configured. Burst behavior is implicit.",
  "file": "src/input/WebSocketCommandProducer.cpp",
  "line_range": "",
  "quote": "if (_server) _server->loop();",
  "risk": "Bounded — won't OOM. But the drain spike can extend a single ws_prod iteration to dozens of ms, deferring the next heartbeat tick. Combined with the 6 s half-open detect, this is below the safety threshold but the worst-case ws_prod tick is essentially unbounded if a client sends a maximally fragmented WS message tree.",
  "verification_status": "verified"
}
```

```json
{
  "id": "C5-P1-09",
  "title": "WiFi-offline reboot watchdog uses a single `g_lastWifiConnectedMs` updated in two places (WiFi event callback + loop()) — race plus reentran …[truncated]",
  "file": "src/main_robot.cpp",
  "line_range": "",
  "quote": "if (WiFi.status() == WL_CONNECTED) {\n        g_lastWifiConnectedMs = millis();\n    } else if (millis() - g_lastWifiConnectedMs > WIFI_OFFLINE_REBOOT_MS) {\n        Serial.println(\"FATAL: WiFi offline >30s — restarting.\");\n        ESP.restart();\n    }",
  "risk": "Rare edge: reconnect happens within the same 100 ms window that loop() decides to reboot. Reboot is recoverable (it just restarts), so the impact is short interruption rather than data loss. Not data-corrupting.",
  "verification_status": "verified"
}
```

### Cell 6

```json
{
  "id": "C6-P1-04",
  "title": "lwIP / WiFi event task overlaps ws_prod on Core 0 — priority inversion risk",
  "file": "src/main_robot.cpp + src/input/WebSocketCommandProducer.cpp",
  "line_range": "main_robot.cpp:50, WebSocketCommandProducer.cpp:19-20",
  "quote": "constexpr BaseType_t  CONTROL_TASK_CORE     = 1;",
  "risk": "",
  "verification_status": "verified"
}
```

```json
{
  "id": "C6-P1-05",
  "title": "Arduino loopTask (Core 1, prio 1) is not subscribed to TWDT and runs OTA blocking I/O",
  "file": "src/main_robot.cpp",
  "line_range": "252-264",
  "quote": "void loop() {\n    // Housekeeping: OTA poll + WiFi reboot watchdog. Control runs in dedicated\n    // tasks. Tick at 10 Hz so OTA initiate requests are answered promptly.\n    ArduinoOTA.handle();",
  "risk": "",
  "verification_status": "verified"
}
```

```json
{
  "id": "C6-P1-06",
  "title": "Control task subscribes to TWDT but never unsubscribes; task is infinite anyway, so init must be idempotent — verify `esp_task_wdt_init` run …[truncated]",
  "file": "src/main_robot.cpp",
  "line_range": "228-247",
  "quote": "esp_task_wdt_init(TWDT_TIMEOUT_S, true);\n\n    xTaskCreatePinnedToCore(\n        &controlTask,\n        \"control\",\n        CONTROL_TASK_STACK_BYTES,\n        nullptr,\n        CONTROL_TASK_PRIORITY,\n        nullptr,\n        CONTROL_TASK_CORE);",
  "risk": "",
  "verification_status": "verified"
}
```

```json
{
  "id": "C6-P1-07",
  "title": "g_lastWifiConnectedMs is volatile-only — torn read between Core 0 WiFi event and Core 1 loopTask",
  "file": "src/main_robot.cpp",
  "line_range": "83, 87-88, 257-260",
  "quote": "volatile uint32_t g_lastWifiConnectedMs = 0;",
  "risk": "",
  "verification_status": "verified"
}
```

```json
{
  "id": "C6-P1-08",
  "title": "Control task period derived from `pdMS_TO_TICKS(10)` quantizes to FreeRTOS tick — verify 1 ms tick rate",
  "file": "src/main_robot.cpp",
  "line_range": "154-155, 204",
  "quote": "TickType_t lastWake = xTaskGetTickCount();\n    const TickType_t period = pdMS_TO_TICKS(CONTROL_PERIOD_MS);",
  "risk": "",
  "verification_status": "verified"
}
```

```json
{
  "id": "C6-P1-09",
  "title": "Stack sizes 8192 B not validated against actual call depth — IMU + PID + drivetrain on one frame",
  "file": "src/main_robot.cpp",
  "line_range": "48, 144-204",
  "quote": "constexpr uint32_t CONTROL_TASK_STACK_BYTES = 8192;",
  "risk": "",
  "verification_status": "verified"
}
```

```json
{
  "id": "C6-P1-10",
  "title": "Setup() heap-allocates g_latch/g_producer/g_controller after IMU init but before TWDT init — control task could start before producer",
  "file": "src/main_robot.cpp",
  "line_range": "228-247",
  "quote": "g_latch      = new CommandLatch<BodyVelocity>();\n    g_producer   = new WebSocketCommandProducer(*g_latch, WS_PORT);\n    g_controller = new PassthroughDrivetrainController(g_drivetrain, g_imu);\n\n    g_producer->start();\n\n    initOta();",
  "risk": "",
  "verification_status": "verified"
}
```

### Cell 7

```json
{
  "id": "C7-P1-01",
  "title": "Dead constants in FIT0186 hardware-spec header",
  "file": "src/motor/constants/FIT0186.h",
  "line_range": "",
  "quote": "    inline constexpr float GEAR_RATIO = 43.8f;\n    inline constexpr uint16_t NO_LOAD_RPM = 251;",
  "risk": "Two sources of truth for the motor's no-load RPM. A future motor swap that updates fit0186::NO_LOAD_RPM will silently not affect MAX_RPM, leading to saturation limits that mismatch reality.",
  "verification_status": "verified"
}
```

```json
{
  "id": "C7-P1-02",
  "title": "RAD_TO_RPM conversion duplicated between production code and test",
  "file": "src/drivetrain/OmniKinematics.h:36 + test/test_omni_kinematics/test_omni_kinematics.cpp:15",
  "line_range": "",
  "quote": "        constexpr float RAD_TO_RPM = 60.0f / (2.0f * static_cast<float>(M_PI));",
  "risk": "Three independent rad↔RPM idioms in a codebase whose entire control authority pipeline is RPM-space. A future units bug (e.g. someone writes `2π/60` instead of `60/2π`) is undetectable until live.",
  "verification_status": "verified"
}
```

```json
{
  "id": "C7-P1-03",
  "title": "Driver abstraction leaks: brake/zero semantics diverge across drivers",
  "file": "src/motor/driver/L298NDriver.cpp + src/motor/driver/BTS7960Driver.cpp",
  "line_range": "",
  "quote": "void L298NDriver::brake() {\n    analogWrite(_pins.fwd, 255);\n    analogWrite(_pins.rev, 255);\n}",
  "risk": "The PID hot path issues sub-1.0 floats every 10 ms. If you ever switch from L298N to BTS7960 (note: `BTS7960Driver.cpp` is excluded from every env, so this is untested), the same setpoint sequence will produce different deceleration profiles. The interface contract is not enforced.",
  "verification_status": "verified"
}
```

### Cell 9

```json
{
  "id": "C9-P1-01",
  "title": "Dead source files (main_blink.cpp, main_original.cpp) are excluded from every env but still live in src/",
  "file": "code/body-firmware/src/main_original.cpp",
  "line_range": "",
  "quote": " * BB-8 Body Firmware — Motor Spin Test\n * Serial input: \"<motor> <speed>\" e.g. \"0 0.5\" or \"1 -1.0\"\n * \"stop\" to brake all motors",
  "risk": "",
  "verification_status": "verified"
}
```

```json
{
  "id": "C9-P1-02",
  "title": "CONTEXT.md §9.1 is factually wrong about wifi_credentials.h being committed — but the file's own header markers are misleading",
  "file": "code/body-firmware/src/config/wifi_credentials.h",
  "line_range": "",
  "quote": " * BB-8 Body Firmware — local credentials.\n * This file is gitignored. Do NOT commit.",
  "risk": "",
  "verification_status": "verified"
}
```

### Cell 10

```json
{
  "id": "C10-P1-01",
  "title": "Staleness ramp scales omega identically to vx/vy — yaw rate has same 200 ms blind window as translation",
  "file": "src/main_robot.cpp",
  "line_range": "173-183",
  "quote": "BodyVelocity drive_cmd{0.0f, 0.0f, 0.0f};\n        if (have_seen_fresh) {\n            const uint32_t age = now - last_fresh_ms;\n            if (age < STALENESS_TIMEOUT_MS) {\n                const float scale = 1.0f - static_cast<float>(age) /\n                                            static_cast<float>(STALENESS_TIMEOUT_MS);\n                drive_ …[truncated]",
  "risk": "Context §9.9 flagged this. A rotating-only command (`vx=0, vy=0, omega=2.0`) is treated identically to a translating-only command. In a sphere-bot, yaw drift while ramping is arguably **more** dangerous than translation drift: a stale rotation continues to spin the drive platform up the inside of the sphere wall along an axis that the operator no l …[truncated]",
  "suggested_fix": "Split into `TRANSLATION_STALENESS_MS` and `ROTATION_STALENESS_MS`, compute two scale factors, apply per-channel. Or, simpler: keep one timer but ramp omega to zero in 50 ms while vx/vy use 200 ms.",
  "verification_status": "verified"
}
```

```json
{
  "id": "C10-P1-02",
  "title": "Staleness ramp is computed against last_known, not against the previously-emitted drive_cmd — new arrival cancels in-progress ramp instantly",
  "file": "src/main_robot.cpp",
  "line_range": "160-183",
  "quote": "while (true) {\n        // 1) Pull freshest command from latch.\n        auto fresh = g_latch->read();\n        const uint32_t now = millis();\n        if (fresh.has_value()) {\n            last_known      = *fresh;\n            last_fresh_ms   = now;\n            have_seen_fresh = true;\n        }\n\n        // 2) Drive command via staleness alone. ... …[truncated]",
  "risk": "When a new frame arrives, `last_known` is overwritten and `last_fresh_ms = now` -> age = 0, scale = 1.0. Imagine operator sends `vx=2.0` at t=0, then briefly disconnects (network blip) so no frames arrive from t=0 to t=180ms. The ramp has scaled the drive_cmd to `2.0 * (1 - 180/200) = 0.2`. At t=181ms a new frame arrives with `vx=2.0` (operator sti …[truncated]",
  "suggested_fix": "Track `prev_drive_cmd` across iterations. On fresh arrival, instead of `drive_cmd = last_known * 1.0`, blend: `drive_cmd = prev_drive_cmd + clamp((last_known - prev_drive_cmd), max_step)`. Alternative …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C10-P1-03",
  "title": "No sequence number / frame ID — dropped commands are undetectable, replay/reorder is invisible",
  "file": "src/input/CommandFrameParser.h",
  "line_range": "32-58",
  "quote": "inline constexpr size_t kCommandFrameMaxLen = 64;\n\ninline FrameParseResult parseCommandFrame(const uint8_t* payload, size_t length) {\n    if (length == 0 || length >= kCommandFrameMaxLen) {\n        return {{}, FrameParseError::EmptyOrTooLong};\n    }\n\n    char buf[kCommandFrameMaxLen];\n    for (size_t i = 0; i < length; ++i) buf[i] = static_cast<cha …[truncated]",
  "risk": "Protocol is `'vx,vy,omega'` with no sequence number, timestamp, or monotonic ID. With a length-1 latch and overwrite-semantics, drops are intentional but **silent**. From the operator's perspective there is no way to detect: (a) frames dropped due to TCP retransmit buffering at high rate, (b) frames dropped due to control loop temporarily falling b …[truncated]",
  "suggested_fix": "Extend protocol to `'seq,vx,vy,omega'` and reject frames with seq <= last_seq (rejecting reorder). Log per-second accepted/dropped/rejected counts. Forward-compat already supports this via the trailin …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C10-P1-04",
  "title": "Worst-case end-to-end latency is ~16 ms; spike potential to ~30+ ms under I2C contention",
  "file": "src/main_robot.cpp",
  "line_range": "144-206",
  "quote": "void controlTask(void* /*arg*/) {\n    // Subscribe to the Task Watchdog. ...\n    esp_task_wdt_add(NULL);\n\n    BodyVelocity last_known{};\n    uint32_t     last_fresh_ms = 0;\n    bool         have_seen_fresh = false;\n\n    TickType_t lastWake = xTaskGetTickCount();\n    const TickType_t period = pdMS_TO_TICKS(CONTROL_PERIOD_MS);",
  "risk": "Latency budget byte-to-PWM, worst case: (1) WS task vTaskDelay 1ms between server->loop() calls (WebSocketCommandProducer.cpp:108) — frame arrival could wait up to 1ms before parser sees it; (2) parse+latch.write is O(microseconds); (3) cross-core queue handoff — `xQueueOverwrite` from Core0 to Core1 is fast (a few µs) but the consumer is **not** w …[truncated]",
  "suggested_fix": "(1) Add per-tick max-latency log: time from `g_latch->read()` returning a value to motor->setSpeed completing. Log p99 every 1000 ticks. (2) Consider blocking-with-short-timeout in CommandLatch::read …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C10-P1-05",
  "title": "CommandLatch tests are single-threaded native — no cross-core race coverage",
  "file": "test/test_command_latch/test_command_latch.cpp",
  "line_range": "9-80",
  "quote": "static CommandLatch<BodyVelocity>* latch = nullptr;\n\nvoid setUp() {\n    latch = new CommandLatch<BodyVelocity>();\n}\n\nvoid tearDown() {\n    delete latch;\n    latch = nullptr;\n}\n\nvoid test_empty_read_returns_nullopt() {\n    auto v = latch->read();\n    TEST_ASSERT_FALSE(v.has_value());\n}\n\nvoid test_write_then_read_returns_value() {\n    BodyVelocity in …[truncated]",
  "risk": "All five test cases are single-threaded, sequential, on `[env:native]` with a FreeRTOS-stub queue (file dependency on freertos/queue.h means the native env must stub it). The latch's actual job is cross-core overwrite-with-readback: producer on Core0 calls `xQueueOverwrite` at WS-event rate (~100Hz), consumer on Core1 calls `xQueueReceive` at 100Hz …[truncated]",
  "suggested_fix": "Add on-target test (`test/test_command_latch_ontarget/`) that creates two FreeRTOS tasks on different cores, producer writes monotonically-increasing values, consumer reads and asserts (a) read value …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C10-P1-06",
  "title": "Parse failure leaves latch holding previous frame — silent staleness during sustained malformed-frame storm",
  "file": "src/input/WebSocketCommandProducer.cpp",
  "line_range": "143-159",
  "quote": "case WStype_TEXT: {\n            // Drop frames from non-active clients (defense in depth: server\n            // disconnect on rejection isn't always immediate).\n            if (_activeClient.load() != clientNum) return;\n\n            auto result = parseCommandFrame(payload, length);\n            if (!result.ok()) {\n                _parseFailCount.fet …[truncated]",
  "risk": "On parse failure the producer returns without publishing — the latch retains the last good frame. Combined with the consumer-side rule that `last_fresh_ms` is only updated when `g_latch->read().has_value()` returns true, the consumer sees no new frames during a malformed-frame storm and the staleness ramp engages **eventually** (after 200ms of no p …[truncated]",
  "suggested_fix": "If parse failure rate exceeds N/sec (e.g. 50), disconnect the active client. Or: track ratio of valid:invalid frames and disconnect if < 0.5.",
  "verification_status": "verified"
}
```

### Cell 11

```json
{
  "id": "C11-P1-06",
  "title": "Boot races: WS server accepts commands before TWDT/control task arm",
  "file": "src/main_robot.cpp",
  "line_range": "228-247",
  "quote": "g_latch      = new CommandLatch<BodyVelocity>();\n    g_producer   = new WebSocketCommandProducer(*g_latch, WS_PORT);\n    g_controller = new PassthroughDrivetrainController(g_drivetrain, g_imu);\n\n    g_producer->start();\n\n    initOta();\n\n    // Tighten the global Task Watchdog timeout. Affects IDLE tasks too, but\n    // 1s is comfortably above their …[truncated]",
  "risk": "Order: producer starts → OTA inits → TWDT inits → control task is created last. Between `g_producer->start()` (line 232) and `xTaskCreatePinnedToCore(controlTask, ...)` (line 240) the WS server is live and writing to the latch but nothing reads it. If a client connects in this ~ms-scale window the first command is delivered, then the control task s …[truncated]",
  "suggested_fix": "Reorder: create control task first (with an initial 'safety-disarmed' flag that forces drive_cmd=0 regardless of latch). Only flip the flag once `setup()` completes. Or: do not call `g_producer->start …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C11-P1-07",
  "title": "Half-open detection takes ~6 s but `_connected` is unused on hot path",
  "file": "src/input/WebSocketCommandProducer.cpp",
  "line_range": "47-52",
  "quote": "_server->onEvent([this](uint8_t num, WStype_t type, uint8_t* payload, size_t length) {\n        this->onWsEvent(num, static_cast<uint8_t>(type), payload, length);\n    });\n    // Detect half-open TCP within ~6s: ping every 2s, fail after 1s + 2 retries.\n    _server->enableHeartbeat(2000, 1000, 2);",
  "risk": "Heartbeat config matches the comment (ping 2 s, pong-timeout 1 s, 2 retries → upper bound 2 s × 3 = 6 s before disconnect event). The control loop intentionally ignores `_connected` (`main_robot.cpp:170-171`). In the half-open-silent case (T3) ramp catches it at 200 ms — fine. In half-open-with-stale-frames (T4, see C11-P0-01) ramp doesn't catch it …[truncated]",
  "suggested_fix": "Either (a) on `WStype_DISCONNECTED` for the active client, write a zero `BodyVelocity` into the latch immediately AND mark a 'disconnect-latched' flag the control loop reads to call `controller->stop( …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C11-P1-08",
  "title": "PID integrator persists across disconnect/reconnect cycles",
  "file": "src/main_robot.cpp",
  "line_range": "144-206",
  "quote": "BodyVelocity last_known{};\n    uint32_t     last_fresh_ms = 0;\n    bool         have_seen_fresh = false;",
  "risk": "On WS disconnect → reconnect, the control task's locals reset only on a process reset, not on a producer-restart. `_targetRPMs[]` inside `OmniDrivetrain` and the three `PID` integrators are also never reset by any code path the runtime exercises (`PID::reset()` is only called from `OmniDrivetrain::stop()` which is dead code — see C11-P0-05). After …[truncated]",
  "suggested_fix": "On `WStype_DISCONNECTED` for the active client OR on staleness timeout, the control loop should call `g_controller->stop()` once (latched) — this resets PIDs and `_targetRPMs`. Re-arm on first fresh f …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C11-P1-09",
  "title": "No brownout policy; no brake-on-power-loss",
  "file": "src/main_robot.cpp",
  "line_range": "210-250",
  "quote": "void setup() {\n    Serial.begin(115200);\n    delay(200);\n    Serial.println(\"BB-8 main_robot starting.\");\n\n    Wire.begin(I2C_SDA, I2C_SCL);\n\n    g_motor0.begin();\n    g_motor1.begin();\n    g_motor2.begin();",
  "risk": "Grep across the body-firmware tree: zero references to `brownout`, `setBrownoutDetector`, `WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG...)`, or `esp_brownout_register`. ESP32-Arduino enables the brownout detector by default (~2.43 V) and brownout → reset is the chip-level behavior. The reset path is T9: GPIOs go high-Z (good — L298N inputs floating → 0 V …[truncated]",
  "suggested_fix": "Add an `esp_register_shutdown_handler` that calls `g_drivetrain.stop()` to brake before reset. Add a brownout pretrigger via `esp_brownout_register_callback` (ESP-IDF), and consider raising brownout t …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C11-P1-10",
  "title": "loopTask (Arduino main) is not TWDT-subscribed — OTA hang is invisible",
  "file": "src/main_robot.cpp",
  "line_range": "252-264",
  "quote": "void loop() {\n    // Housekeeping: OTA poll + WiFi reboot watchdog. Control runs in dedicated\n    // tasks. Tick at 10 Hz so OTA initiate requests are answered promptly.\n    ArduinoOTA.handle();\n\n    if (WiFi.status() == WL_CONNECTED) {\n        g_lastWifiConnectedMs = millis();\n    } else if (millis() - g_lastWifiConnectedMs > WIFI_OFFLINE_REBOOT_M …[truncated]",
  "risk": "`controlTask` (line 148) and `taskBody` (`WebSocketCommandProducer.cpp:103`) both subscribe to TWDT. Arduino's `loopTask` is created by the framework and is NOT subscribed in this code. If `ArduinoOTA.handle()` deadlocks or `WiFi.status()` blocks, the housekeeping task hangs silently — the control task keeps running, motors keep tracking commands ( …[truncated]",
  "suggested_fix": "Either subscribe loopTask to TWDT (`esp_task_wdt_add(NULL); esp_task_wdt_reset()` inside `loop`) — but the function is short and a 1 s budget is generous — or move the WiFi-reboot watchdog into a dedi …[truncated]",
  "verification_status": "verified"
}
```

### Cell 12

```json
{
  "id": "C12-P1-01",
  "title": "drivetrain-test-procedure suggests Kp=1.0, Ki=0.1, Kd=0.01; code uses Kp=0.005, Ki=0.002, Kd=0.0005",
  "file": "src/main_robot.cpp",
  "line_range": "66-68",
  "quote": "PID g_pid0(0.005f, 0.002f, 0.0005f, -1.0f, 1.0f, 5.0f);\nPID g_pid1(0.005f, 0.002f, 0.0005f, -1.0f, 1.0f, 5.0f);\nPID g_pid2(0.005f, 0.002f, 0.0005f, -1.0f, 1.0f, 5.0f);",
  "risk": "Code gains are ~200× smaller than the doc's suggested starting point. Doc is a troubleshooting hint, but if a contributor uses it as a reference they will introduce 200× over-aggressive control, which on this drivetrain will saturate within one tick. The PID design spec (2026-05-06 omni drivetrain) explicitly defers tuning, so this is the only doc …[truncated]",
  "suggested_fix": "Replace the parenthetical in drivetrain-test-procedure.md with the actual tuned values from main_robot.cpp:66-68, or remove the specific numbers (since deadband=5 RPM is also undocumented).",
  "spec_file": "docs/drivetrain-test-procedure.md",
  "spec_line_range": "135-135",
  "spec_quote": "| Target RPMs look right but actuals don't converge | PID gains need tuning (Kp=1.0, Ki=0.1, Kd=0.01) |",
  "verification_status": "verified"
}
```

```json
{
  "id": "C12-P1-02",
  "title": "OmniDrivetrain spec PID interface omits dt; code requires dt",
  "file": "src/control/PID.h",
  "line_range": "15-15",
  "quote": "    float compute(float setpoint, float measurement, float dt) {",
  "risk": "Spec is an 'assumed interface' for the drivetrain to reason against. The mismatch isn't a bug — it just makes the spec obsolete as documentation of how the drivetrain talks to PID. Future readers will need to cross-reference PID.h.",
  "suggested_fix": "Update the spec snippet to match `float compute(float setpoint, float measurement, float dt)` or add a note that the final interface added explicit dt.",
  "spec_file": "docs/superpowers/specs/2026-05-06-omni-drivetrain-design.md",
  "spec_line_range": "154-160",
  "spec_quote": "class PID {\npublic:\n    float compute(float setpoint, float measurement) -> float;  // output in [-1, 1]\n    void reset();\n};",
  "verification_status": "verified"
}
```

```json
{
  "id": "C12-P1-03",
  "title": "PID deadband of 5 RPM is undocumented; PID spec excludes tuning",
  "file": "src/main_robot.cpp",
  "line_range": "66-68",
  "quote": "PID g_pid0(0.005f, 0.002f, 0.0005f, -1.0f, 1.0f, 5.0f);",
  "risk": "5 RPM deadband (the 6th arg) silently zeros the integral and output whenever setpoint==0 AND |measurement|<5 RPM. This produces a discontinuity (no PWM at all when commanding zero, even though the wheel may be coasting at <5 RPM). Not in any spec, but it is observable behavior — an operator commanding 'stop' through the staleness ramp will see PID …[truncated]",
  "suggested_fix": "Add a 'PID deadband: 5 RPM' line to either the drivetrain spec or the controller-discussion doc.",
  "spec_file": "docs/superpowers/specs/2026-05-06-omni-drivetrain-design.md",
  "spec_line_range": "164-171",
  "spec_quote": "## What This Spec Does NOT Cover\n\n- Acceleration ramping (PID handles smoothing naturally)\n- IMU integration / heading correction (DrivetrainController)\n- Roll/pitch stabilization (DrivetrainController)\n- FreeRTOS task setup and loop timing (main.cpp …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C12-P1-04",
  "title": "DrivetrainConfig.maxRPM described as 'motor no-load RPM'; OmniKinematics treats it as a hard saturation ceiling",
  "file": "src/drivetrain/OmniKinematics.h",
  "line_range": "45-50",
  "quote": "        if (maxAbs > _config.maxRPM) {\n            float factor = _config.maxRPM / maxAbs;\n            for (int i = 0; i < 3; i++) {\n                rpms[i] *= factor;\n            }\n        }",
  "risk": "Using no-load RPM as the saturation ceiling means under load the motors will never achieve the demanded RPM (load curve falls off quickly from no-load), and PID will permanently see a positive error. Anti-windup (PID.h:32-37) prevents integrator runaway but the output is pegged at outputMax. Spec doesn't distinguish 'no-load RPM' (a motor spec shee …[truncated]",
  "suggested_fix": "Rename `maxRPM` to `saturationRPM` (or add a comment 'set conservatively below no-load to leave PID headroom') and document in RobotConstants.h.",
  "spec_file": "docs/superpowers/specs/2026-05-06-omni-drivetrain-design.md",
  "spec_line_range": "47-52",
  "spec_quote": "struct DrivetrainConfig {\n    float wheelRadius;   // meters\n    float robotRadius;   // center to wheel contact point, meters\n    float tiltAngle;     // wheel tilt from vertical, radians\n    float maxRPM;        // motor no-load RPM (251 for FIT018 …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C12-P1-05",
  "title": "Spec '100 Hz target' is approximate; code is exact 10ms via vTaskDelayUntil",
  "file": "src/drivetrain/RobotConstants.h",
  "line_range": "15-16",
  "quote": "constexpr uint32_t STALENESS_TIMEOUT_MS = 200;  // ramp-to-zero window on silence\nconstexpr uint32_t CONTROL_PERIOD_MS    = 10;   // 100 Hz tick",
  "risk": "Editorial. 100 Hz target vs 10ms period is the same number. P1 because controlTask passes the nominal 0.010 directly to PID rather than measuring actual elapsed; if the spec said 'PID dt is the nominal CONTROL_PERIOD_MS' it would be load-bearing. It doesn't.",
  "suggested_fix": "Either accept this as editorial or add a 'PID dt = CONTROL_PERIOD_MS/1000, not measured' note to the spec.",
  "spec_file": "docs/superpowers/specs/2026-05-06-omni-drivetrain-design.md",
  "spec_line_range": "129-129",
  "spec_quote": "Called at a fixed rate (100 Hz target) from a FreeRTOS task on Core 1:",
  "verification_status": "verified"
}
```

```json
{
  "id": "C12-P1-06",
  "title": "Spec OmniDrivetrain class layout shows kinematics+pids only; code inherits _motors[3] from Drivetrain base",
  "file": "src/drivetrain/Drivetrain.h",
  "line_range": "27-29",
  "quote": "protected:\n    Motor* _motors[3];\n};",
  "risk": "Spec's class diagram doesn't show where motors live. Code factored them into the base. Functionally fine, but spec readers may think OmniDrivetrain holds its own motor refs. Editorial.",
  "suggested_fix": "Update OmniDrivetrain spec class sketch to show `_motors` is inherited from `Drivetrain<TVelocity>`.",
  "spec_file": "docs/superpowers/specs/2026-05-06-omni-drivetrain-design.md",
  "spec_line_range": "114-121",
  "spec_quote": "private:\n    void setMotorSpeeds(float s0, float s1, float s2);\n\n    OmniKinematics _kinematics;\n    PID* _pids[3];\n    float _targetRPMs[3] = {};\n};",
  "verification_status": "verified"
}
```

```json
{
  "id": "C12-P1-07",
  "title": "Spec stop() resets PIDs + brakes motors; code additionally clears _targetRPMs",
  "file": "src/drivetrain/OmniDrivetrain.h",
  "line_range": "39-43",
  "quote": "    void stop() override {\n        for (auto* m : _motors) m->brake();\n        for (auto* p : _pids) p->reset();\n        for (int i = 0; i < 3; i++) _targetRPMs[i] = 0.0f;\n    }",
  "risk": "Code adds `_targetRPMs[i] = 0.0f` which the spec omits. This matters because if `stop()` is called and then `update()` is called before the next `drive()`, the PID setpoint will be 0 instead of the last commanded value. Code behavior is more correct; spec is stale.",
  "suggested_fix": "Add the target-RPM zeroing to the spec snippet.",
  "spec_file": "docs/superpowers/specs/2026-05-06-omni-drivetrain-design.md",
  "spec_line_range": "139-144",
  "spec_quote": "Brakes all motors and resets PID state:\n\n```cpp\nfor (auto* m : _motors) m->brake();\nfor (auto* p : _pids) p->reset();\n```",
  "verification_status": "verified"
}
```

```json
{
  "id": "C12-P1-08",
  "title": "controller-discussion.md says 'tilt limiting is mandatory'; runtime controller is passthrough and ignores IMU",
  "file": "src/drivetrain/controller/PassthroughDrivetrainController.h",
  "line_range": "17-19",
  "quote": "    void update(const BodyVelocity& command, const IMUReading& /*imuData*/) override {\n        _drivetrain.drive(command);\n    }",
  "risk": "The controller spec is explicit: tilt limiting is mandatory due to the 13:1 force-to-weight ratio. The runtime controller is a no-op that forwards the command unchanged and discards IMU data. CONTEXT.md notes this is a known gap (#11 in §9), but the controller-discussion doc reads as a hard requirement — there is no doc-side acknowledgement that th …[truncated]",
  "suggested_fix": "Add a 'Status: not yet implemented; PassthroughDrivetrainController is a stub' header to controller-discussion.md, or convert this doc into a tracked TODO with explicit warning that the production con …[truncated]",
  "spec_file": "docs/controller-discussion.md",
  "spec_line_range": "16-23",
  "spec_quote": "## Why Tilt Limiting is Mandatory\n\nThe FIT0186 motors produce ~177N combined stall force (3 motors × 1.77 N·m stall torque / 0.03m wheel radius). The drive platform weighs ~1.4kg (~13.7N). The force-to-weight ratio is ~13:1, meaning the motors can tr …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C12-P1-09",
  "title": "200ms staleness ramp is implemented in main_robot but absent from any spec",
  "file": "src/main_robot.cpp",
  "line_range": "175-182",
  "quote": "            const uint32_t age = now - last_fresh_ms;\n            if (age < STALENESS_TIMEOUT_MS) {\n                const float scale = 1.0f - static_cast<float>(age) /\n                                            static_cast<float>(STALENESS_TIMEOUT_MS);\n                drive_cmd = { last_known.vx * scale,\n                              last_known.v …[truncated]",
  "risk": "Spec disclaims acceleration ramping; the code introduces a different kind of ramping (a watchdog-driven decay to zero on command silence). The spec exists for the drivetrain layer and the ramp lives in main_robot, so this is partly out of scope — but there is no doc whatsoever for the 200ms behavior, the linear shape, or the rationale (all three ar …[truncated]",
  "suggested_fix": "Add a short 'Command staleness / dead-window policy' section to either controller-discussion.md or a new teleop-loop spec.",
  "spec_file": "docs/superpowers/specs/2026-05-06-omni-drivetrain-design.md",
  "spec_line_range": "164-171",
  "spec_quote": "## What This Spec Does NOT Cover\n\n- Acceleration ramping (PID handles smoothing naturally)",
  "verification_status": "verified"
}
```

```json
{
  "id": "C12-P1-10",
  "title": "Motor design spec example platformio adds -DBB8_DEBUG unconditionally; production envs lack it (BB8_ASSERT is a no-op on hardware)",
  "file": "platformio.ini",
  "line_range": "8-22",
  "quote": "build_flags =\n    -std=gnu++17\n    -Wall\n    -Wextra\n    -Isrc/config\n    -Isrc/motor\n    -Isrc/motor/constants",
  "risk": "Spec said 'Remove `-DBB8_DEBUG` for release builds'. The actual platformio has `-DBB8_DEBUG` only on `[env:native]` (line 83), never on `[env:robot]` etc. So on hardware, every `BB8_ASSERT` is `((void)0)`. That includes asserts in OmniKinematics (positive radii, sane tilt angle), L298NDriver::setOutput (range check on `[-1,1]`), FIT0186Motor (posit …[truncated]",
  "suggested_fix": "Spec is accurate (it says release removes BB8_DEBUG) but CONTEXT.md observes nobody is set up to flip in a debug hardware build. Either add a `[env:robot_debug]` env that defines BB8_DEBUG, or note in …[truncated]",
  "spec_file": "docs/superpowers/specs/2026-04-23-fit0186-motor-design.md",
  "spec_line_range": "269-291",
  "spec_quote": "build_flags =\n    -std=gnu++17\n    -Wall\n    -Wextra\n    -Isrc/config\n    -Isrc/motor\n[...]\n    -DBB8_DEBUG",
  "verification_status": "verified"
}
```

```json
{
  "id": "C12-P1-11",
  "title": "Spec gives angles in 'clockwise from above'; derivation explains CW-angle convention with -sin/-cos; code's SIN_120=+0.866 matches +sin(120° …[truncated]",
  "file": "src/drivetrain/OmniKinematics.h",
  "line_range": "22-25",
  "quote": "        constexpr float SIN_120 =  0.86602540378f;\n        constexpr float COS_120 = -0.5f;\n        constexpr float SIN_240 = -0.86602540378f;\n        constexpr float COS_240 = -0.5f;",
  "risk": "Subtle. The spec defines θ_i as the CW angle from front and uses sin(θ_i) in the formula. sin(120° CW) seen as a normal math angle is sin(-120°) = -0.866. But sin(+120°) = +0.866, and the code uses +0.866. Reading the derivation literally, the code should use SIN_120 = -0.866 for CW. The two cancel out because in the derivation 'CW angles correspon …[truncated]",
  "suggested_fix": "Add a comment in OmniKinematics.h immediately above the SIN_120/SIN_240 declarations: `// θ is CW from front but evaluated as standard math sin/cos; the negative signs in the formula handle the CW con …[truncated]",
  "spec_file": "docs/omni-kinematics-derivation.md",
  "spec_line_range": "52-55",
  "spec_quote": "In our frame (x-forward, y-left, z-up), CW angles correspond to negative rotation. The wheel at CW angle θ_i has position direction (cos θ, −sin θ) and rolling direction (−sin θ, −cos θ). Projecting the contact-point velocity onto this direction:\n\n`` …[truncated]",
  "verification_status": "verified"
}
```

## P2 — verified

### Cell 1

```json
{
  "id": "C1-P2-01",
  "title": "L298N PWM range conversion uses static_cast<uint8_t> truncation, not rounding",
  "file": "src/motor/driver/L298NDriver.cpp",
  "line_range": "22-22",
  "quote": "uint8_t pwm = static_cast<uint8_t>(std::abs(value) * 255.0f);",
  "risk": "Truncation: setOutput(1.0) → 255, setOutput(0.999) → 254 (intended). But setOutput(-1.0) → abs = 1.0 → 255 (OK); setOutput close to 0 truncates to 0 silently — combined with `if (p …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C1-P2-02",
  "title": "OmniKinematics recomputes cosAlpha and scale on every call",
  "file": "src/drivetrain/OmniKinematics.h",
  "line_range": "27-28",
  "quote": "float cosAlpha = cosf(_config.tiltAngle);\n        float scale = 1.0f / (_config.wheelRadius * cosAlpha);",
  "risk": "`cosf` and division per call. At 100 Hz it's cheap, but the value never changes after construction. Precompute in the constructor.",
  "verification_status": "verified"
}
```

```json
{
  "id": "C1-P2-03",
  "title": "FIT0186Motor exposes `_encoder` as a public member for tests",
  "file": "src/motor/FIT0186Motor.h",
  "line_range": "70-71",
  "quote": "// Public for testing — test code sets _encoder._count directly\n    ESP32Encoder _encoder;",
  "risk": "Public mutable state for test affordance. Easy to misuse from production code. Minor.",
  "verification_status": "verified"
}
```

```json
{
  "id": "C1-P2-04",
  "title": "OmniKinematics has no `forward()` (forward-kinematics) function — review item is moot",
  "file": "src/drivetrain/OmniKinematics.h",
  "line_range": "9-18",
  "quote": "class OmniKinematics {\npublic:\n    explicit OmniKinematics(const DrivetrainConfig& config) : _config(config) {",
  "risk": "The review prompt asked to verify `forward()` and `inverse()`. The class exposes only `toWheelRPMs` (the inverse map BodyVelocity→wheel RPMs). No forward map exists. Not currently …[truncated]",
  "verification_status": "verified"
}
```

### Cell 2

```json
{
  "id": "C2-P2-01",
  "title": "`IMUField operator&` returns bool — composing two masks with `&` is ill-typed",
  "file": "src/imu/IMUReading.h",
  "line_range": "23-29",
  "quote": "inline bool operator&(IMUField a, IMUField b) {",
  "risk": "Asymmetric bitmask operators (CONTEXT §9.4). Low-risk today; future caller doing mask intersection gets a type error or implicit-conversion surprise.",
  "verification_status": "verified"
}
```

```json
{
  "id": "C2-P2-02",
  "title": "Unused IMU read per tick costs 4 I²C transactions for nothing",
  "file": "src/main_robot.cpp",
  "line_range": "186-187",
  "quote": "IMUReading imu_reading = g_imu.read();",
  "risk": "CONTEXT §9.10 calls this out. Passthrough discards the result; worst-case I²C cost eats meaningfully into a 10 ms budget and raises TWDT panic risk on bus stretch.",
  "verification_status": "verified"
}
```

```json
{
  "id": "C2-P2-03",
  "title": "No test asserts BNO055-begin-failure → ESP.restart path",
  "file": "src/main_robot.cpp",
  "line_range": "221-224",
  "quote": "FATAL: BNO055 init failed",
  "risk": "test_begin_failure_returns_false (test_bno055_imu.cpp:94) verifies the IMU returns false, but the main-side reaction is untested. Untestable on native without harness.",
  "verification_status": "verified"
}
```

### Cell 3

```json
{
  "id": "C3-P2-01",
  "title": "I2C pin documented in code comment is stale",
  "file": "src/main_robot.cpp",
  "line_range": "52-54",
  "quote": "// I2C pins for BNO055 (matches main_imu_test.cpp)\nconstexpr int I2C_SDA = 21;\nconstexpr int I2C_SCL = 22;",
  "risk": "Comment claims parity with `main_imu_test.cpp` — true (both use 21/22), but `docs/imu-test-procedure.md:15-16` documents SDA=26, SCL=25 for the same sensor. See C3-P1-04 above.",
  "verification_status": "verified"
}
```

```json
{
  "id": "C3-P2-02",
  "title": "Lifecycle interface has no started()/state query; double-start is a silent no-op, double-stop is benign",
  "file": "src/util/Lifecycle.h",
  "line_range": "13-24",
  "quote": "class Lifecycle {\npublic:\n    virtual ~Lifecycle() = default;\n\n    // Bring the component online: start any FreeRTOS task, open sockets, etc.\n    virtual void start() = 0;\n\n    // Bring the component …[truncated]",
  "risk": "No `bool running() const` and no explicit state machine. `WebSocketCommandProducer::start()` (`.cpp:42-43`) silently returns if already running; `stop()` (`.cpp:68-69`) silently re …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C3-P2-03",
  "title": "MovingAverage::reset() touches `_buffer = {}` which compiles to `memset` — fine, but `average()` after reset returns 0.0f regardless of any …[truncated]",
  "file": "src/util/MovingAverage.h",
  "line_range": "31-35",
  "quote": "void reset() {\n        _buffer = {};\n        _index = 0;\n        _count = 0;\n    }",
  "risk": "Tested by `test_reset` (line 33) and `test_push_after_reset` (line 49). No reset() consumer exists in the production tree — `FIT0186Motor` never calls it after construction. The fa …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C3-P2-04",
  "title": "POD util types lack equality and stream ops — debug printing requires manual field-by-field printf",
  "file": "src/util/Vec3.h",
  "line_range": "8-12",
  "quote": "struct Vec3 {\n    float x = 0;\n    float y = 0;\n    float z = 0;\n};",
  "risk": "Same for `Quat`, `Euler`, `CalStatus`. None of these have `operator==` or `operator<<`. Test code (`test_passthrough_controller.cpp:62-64`) uses field-by-field `TEST_ASSERT_EQUAL_F …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C3-P2-05",
  "title": "Test coverage gap: no `controlTask` unit test for the staleness ramp, no `setup()` order test",
  "file": "test/test_passthrough_controller/test_passthrough_controller.cpp",
  "line_range": "57-70",
  "quote": "void test_update_forwards_command_to_drivetrain() {",
  "risk": "The two test cases (`test_update_forwards_command_to_drivetrain`, `test_stop_calls_drivetrain_stop`) cover the pure passthrough — they don't exercise: (a) staleness ramp math at ag …[truncated]",
  "verification_status": "verified"
}
```

### Cell 4

```json
{
  "id": "C4-P2-10",
  "title": "OmniKinematics recomputes cosf(tiltAngle) on every drive() call",
  "file": "src/drivetrain/OmniKinematics.h",
  "line_range": "27-28",
  "quote": "float cosAlpha = cosf(_config.tiltAngle);\n        float scale = 1.0f / (_config.wheelRadius * cosAlpha);",
  "risk": "",
  "verification_status": "verified"
}
```

```json
{
  "id": "C4-P2-11",
  "title": "MovingAverage average() recomputes sum every call instead of running sum",
  "file": "src/util/MovingAverage.h",
  "line_range": "22-29",
  "quote": "float average() const {\n        if (_count == 0) return 0.0f;\n        float sum = 0.0f;\n        for (size_t i = 0; i < _count; ++i) {\n            sum += _buffer[i];\n        }\n        return sum / stat …[truncated]",
  "risk": "",
  "verification_status": "verified"
}
```

```json
{
  "id": "C4-P2-12",
  "title": "Static-storage construction of L298NDriver/FIT0186Motor relies on global init order — same TU only",
  "file": "src/main_robot.cpp",
  "line_range": "58-70",
  "quote": "L298NDriver g_driver0(MOTOR0_PINS);\nL298NDriver g_driver1(MOTOR1_PINS);\nL298NDriver g_driver2(MOTOR2_PINS);\n\nFIT0186Motor<5> g_motor0(g_driver0, MOTOR0_ENCODER);\nFIT0186Motor<5> g_motor1(g_driver1, MO …[truncated]",
  "risk": "",
  "verification_status": "verified"
}
```

```json
{
  "id": "C4-P2-13",
  "title": "OmniDrivetrain::update() does not check that motors are bound — _motors[i] dereference if construction half-failed",
  "file": "src/drivetrain/OmniDrivetrain.h",
  "line_range": "25-32",
  "quote": "void update(float dt) override {\n        for (auto* m : _motors) m->update();",
  "risk": "",
  "verification_status": "verified"
}
```

### Cell 5

```json
{
  "id": "C5-P2-07",
  "title": "Heartbeat is driven by `_server->loop()` inline — same task, not a separate timer. A long parse delays the next heartbeat check.",
  "file": "src/input/WebSocketCommandProducer.cpp",
  "line_range": "",
  "quote": "_server->enableHeartbeat(2000, 1000, 2);",
  "risk": "Low. TWDT (1 s panic) bounds the worst case before heartbeat even becomes the concern. Spurious disconnect requires ws_prod to be 'mostly' but not 'totally' wedged.",
  "verification_status": "verified"
}
```

```json
{
  "id": "C5-P2-08",
  "title": "BNO055 begin() is blocking, and it runs **before** `connectWifi()` and **before** `g_producer->start()` in setup() — operator cannot connect …[truncated]",
  "file": "src/main_robot.cpp",
  "line_range": "",
  "quote": "if (!g_imu.begin()) {\n        Serial.println(\"FATAL: BNO055 init failed — restarting.\");\n        ESP.restart();\n    }",
  "risk": "Operator UX: dashboard cannot WS-connect during the boot window. This is acceptable for a 2-11 s window. The risk is if BNO055 has an intermittent I2C startup failure, the device r …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C5-P2-10",
  "title": "ws_prod task priority (2) is below loopTask (1)? Actually equal — but lower than control (4); the WiFi event task pumps callbacks at priorit …[truncated]",
  "file": "src/input/WebSocketCommandProducer.cpp",
  "line_range": "",
  "quote": "constexpr UBaseType_t kTaskPriority = 2;",
  "risk": "No bug. Note for future: if anyone moves the OTA callback to Core 0 (e.g. WS-triggered firmware update), the priority relationship between loopTask and ws_prod becomes a single-cor …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C5-P2-11",
  "title": "BNO055IMU `IMUField operator&` returns `bool` — composing more than two masks with `&` won't work",
  "file": "src/imu/IMUReading.h",
  "line_range": "",
  "quote": "inline bool operator&(IMUField a, IMUField b) {\n    return (static_cast<uint8_t>(a) & static_cast<uint8_t>(b)) != 0;\n}",
  "risk": "Latent. Not a realtime bug today; will become a correctness bug the first time someone tries multi-bit masking and reads stale IMU data.",
  "verification_status": "verified"
}
```

### Cell 6

```json
{
  "id": "C6-P2-11",
  "title": "vTaskDelayUntil with `lastWake = xTaskGetTickCount()` initialized before subscribing to TWDT — first iter's deadline may already be in the p …[truncated]",
  "file": "src/main_robot.cpp",
  "line_range": "148-155",
  "quote": "esp_task_wdt_add(NULL);\n\n    BodyVelocity last_known{};\n    uint32_t     last_fresh_ms = 0;\n    bool         have_seen_fresh = false;\n\n    TickType_t lastWake = xTaskGetTickCount();",
  "risk": "",
  "verification_status": "verified"
}
```

```json
{
  "id": "C6-P2-12",
  "title": "Serial.print/printf cross-task contention — ws_prod, controlTask, onWifiEvent, onWsEvent all write Serial",
  "file": "src/main_robot.cpp + src/input/WebSocketCommandProducer.cpp",
  "line_range": "main_robot.cpp:89-93, 108, 115, 119, 129-133, 199, 222, 249, 260; producer 127-141, 166-168, 173, 186",
  "quote": "Serial.printf(\"[ctrl] stack HWM min over 100 iters: %u words free\\n\", stack_hwm_min);",
  "risk": "",
  "verification_status": "verified"
}
```

```json
{
  "id": "C6-P2-13",
  "title": "loop() at 10 Hz is the only thing handling OTA — high-prio control task can starve it during sustained 100 Hz iterations if IMU stalls",
  "file": "src/main_robot.cpp",
  "line_range": "252-264",
  "quote": "vTaskDelay(pdMS_TO_TICKS(LOOP_TICK_MS));",
  "risk": "",
  "verification_status": "verified"
}
```

```json
{
  "id": "C6-P2-14",
  "title": "MovingAverage<N>::reset() not used; per-tick `average()` recomputes sum O(N)",
  "file": "src/util/MovingAverage.h",
  "line_range": "22-29",
  "quote": "float average() const {\n        if (_count == 0) return 0.0f;\n        float sum = 0.0f;\n        for (size_t i = 0; i < _count; ++i) {\n            sum += _buffer[i];\n        }\n        return sum / stat …[truncated]",
  "risk": "",
  "verification_status": "verified"
}
```

### Cell 7

```json
{
  "id": "C7-P2-04",
  "title": "API ergonomics: Motor::setSpeed/Driver::setOutput take unchecked float",
  "file": "src/motor/Motor.h + src/motor/driver/Driver.h + src/drivetrain/BodyVelocity.h",
  "line_range": "",
  "quote": "    // Set motor output. -1.0 = full reverse, 0.0 = coast, 1.0 = full forward.\n    virtual void setSpeed(float speed) = 0;",
  "risk": "A caller passing a PID output before clamping, or accidentally an RPM value instead of normalized output, compiles and silently drives the motor with garbage. The PID output range …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C7-P2-05",
  "title": "Drivetrain interface leaks 3-motor implementation into the base",
  "file": "src/drivetrain/Drivetrain.h",
  "line_range": "",
  "quote": "    Drivetrain(Motor& m0, Motor& m1, Motor& m2)\n        : _motors{&m0, &m1, &m2} {}\n[...]\nprotected:\n    Motor* _motors[3];",
  "risk": "Locks the interface to omni geometry. Future hardware variants (the doc trail mentions a possible tilt-stabilizer / different bot) can't reuse the abstraction.",
  "verification_status": "verified"
}
```

```json
{
  "id": "C7-P2-06",
  "title": "PID is reusable in principle; signature/docs imply motor-only",
  "file": "src/control/PID.h",
  "line_range": "",
  "quote": "    PID(float kp, float ki, float kd, float outputMin, float outputMax,\n        float deadband = 0.0f)",
  "risk": "Low. The component is reusable; this is a documentation gap that may discourage reuse.",
  "verification_status": "verified"
}
```

```json
{
  "id": "C7-P2-07",
  "title": "L298N vs BTS7960 driver: 80% structural duplication, no shared base",
  "file": "src/motor/driver/L298NDriver.cpp + src/motor/driver/BTS7960Driver.cpp",
  "line_range": "",
  "quote": "    uint8_t pwm = static_cast<uint8_t>(std::abs(value) * 255.0f);",
  "risk": "Maintenance hazard. Already drifted: BTS7960 has a second redundant `if (pwm == 0)` check after the early `value == 0.0f` check; L298N has no such redundancy. Float-equality agains …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C7-P2-08",
  "title": "Inconsistent RPM unit suffix on parameters & members",
  "file": "src/motor/Motor.h + src/drivetrain/OmniDrivetrain.h + src/drivetrain/DrivetrainConfig.h",
  "line_range": "",
  "quote": "    virtual void setSpeed(float speed) = 0;",
  "risk": "`setSpeed` is the worst offender: a caller might pass an RPM in and expect a speed. The compiler doesn't catch it (see C7-P2-04). All other actuation code consistently suffixes RPM …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C7-P2-09",
  "title": "ESP32Encoder member is public 'for testing'",
  "file": "src/motor/FIT0186Motor.h",
  "line_range": "",
  "quote": "    // Public for testing — test code sets _encoder._count directly\n    ESP32Encoder _encoder;",
  "risk": "Any code that holds a `FIT0186Motor&` can now reach into the encoder and call e.g. `attachFullQuad` a second time. Not a runtime risk today (no such callers) but the test seam is s …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C7-P2-10",
  "title": "Heavy includes in motor header (Arduino.h, ESP32Encoder.h)",
  "file": "src/motor/FIT0186Motor.h",
  "line_range": "",
  "quote": "#include <ESP32Encoder.h>\n#include <Arduino.h>",
  "risk": "Firmware, so build-time impact is minor — but `Arduino.h` also redefines macros (e.g. `min`, `max`, `B0`) that have bitten C++ projects before. The note in the lens question accept …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C7-P2-11",
  "title": "Comments that restate code; missing WHY on PID dt-floor and OmniDrivetrain::stop unwiring",
  "file": "src/motor/Motor.h + src/control/PID.h + src/drivetrain/OmniDrivetrain.h",
  "line_range": "",
  "quote": "    // Refresh encoder state and recompute RPM. Call at a fixed interval.\n    virtual void update() = 0;",
  "risk": "Per the project rule ('only leave doc'), most actuation comments are noise; one important WHY is missing (the 1e-4 dt-floor — is this 100 µs? Is it tied to micros() resolution? To …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C7-P2-12",
  "title": "DrivetrainConfig uses bare float for tiltAngle without enforced unit",
  "file": "src/drivetrain/DrivetrainConfig.h",
  "line_range": "",
  "quote": "    float tiltAngle;     // wheel tilt from vertical, radians",
  "risk": "If anyone constructs a `DrivetrainConfig{...}` without going through `RobotConstants::drivetrainConfig()` (tests, future ports), passing 30.0f instead of 30°→rad is a silent ~22% s …[truncated]",
  "verification_status": "verified"
}
```

### Cell 8

```json
{
  "id": "C8-P2-01",
  "title": "CommandProducer interface is thin to the point of being unbalanced — Mock/WS contract differs",
  "file": "src/input/CommandProducer.h:11-17 ; src/input/MockCommandProducer.h:14-31 ; src/input/WebSocketCommandProducer.cpp:117-179",
  "line_range": "",
  "quote": "virtual bool connected() const = 0;",
  "risk": "",
  "verification_status": "verified"
}
```

```json
{
  "id": "C8-P2-02",
  "title": "Mock/real divergence: WS-only behaviors not modeled by Mock — test-fidelity gap",
  "file": "src/input/MockCommandProducer.h:17-31 ; src/input/WebSocketCommandProducer.cpp:42-115",
  "line_range": "",
  "quote": "void inject(const T& cmd) { if (_started) _latch.write(cmd); }",
  "risk": "",
  "verification_status": "verified"
}
```

```json
{
  "id": "C8-P2-03",
  "title": "IMU<T> templating leaks BNO055-shaped reading into otherwise-generic consumers",
  "file": "src/imu/IMU.h:8-16 ; src/imu/BNO055IMU.h:15-21 ; src/drivetrain/controller/PassthroughDrivetrainController.h:13",
  "line_range": "",
  "quote": "class BNO055IMU : public IMU<IMUReading> {",
  "risk": "",
  "verification_status": "verified"
}
```

```json
{
  "id": "C8-P2-04",
  "title": "IMUReading carries more than current consumers use — wasted I2C and bandwidth",
  "file": "src/imu/IMUReading.h:31-38 ; src/main_robot.cpp:72-74,186-187 ; src/drivetrain/controller/PassthroughDrivetrainController.h:17",
  "line_range": "",
  "quote": "void update(const BodyVelocity& command, const IMUReading& /*imuData*/) override {",
  "risk": "",
  "verification_status": "verified"
}
```

```json
{
  "id": "C8-P2-05",
  "title": "IMUField operator& returns bool — asymmetric with operator|, prevents mask composition",
  "file": "src/imu/IMUReading.h:23-29",
  "line_range": "",
  "quote": "inline bool operator&(IMUField a, IMUField b) {",
  "risk": "",
  "verification_status": "verified"
}
```

```json
{
  "id": "C8-P2-06",
  "title": "CommandFrameParser is reusable but has unrelated knobs hardcoded; format constants split across files",
  "file": "src/input/CommandFrameParser.h:32-58 ; src/input/WebSocketCommandProducer.cpp:18-22",
  "line_range": "",
  "quote": "inline constexpr size_t kCommandFrameMaxLen = 64;",
  "risk": "",
  "verification_status": "verified"
}
```

```json
{
  "id": "C8-P2-07",
  "title": "Plain-text 'vx,vy,omega' frame format — debuggability vs parse cost, document tradeoff",
  "file": "src/input/CommandFrameParser.h:34-59 ; src/input/WebSocketCommandProducer.h:8-23",
  "line_range": "",
  "quote": "float vx = std::strtof(p, &end);",
  "risk": "",
  "verification_status": "verified"
}
```

```json
{
  "id": "C8-P2-08",
  "title": "Header hygiene mostly clean, but BNO055IMU.h pulls Arduino.h and Adafruit_BNO055 into the included world",
  "file": "src/imu/BNO055IMU.h:10-13 ; src/input/WebSocketCommandProducer.h:36",
  "line_range": "",
  "quote": "class WebSocketsServer;  // forward decl from arduinoWebSockets",
  "risk": "",
  "verification_status": "verified"
}
```

```json
{
  "id": "C8-P2-09",
  "title": "Naming consistency: `latch`/`producer`/`frame` consistent; `connected` flag is overloaded",
  "file": "src/input/WebSocketCommandProducer.h:64-65 ; src/input/WebSocketCommandProducer.cpp:117-142",
  "line_range": "",
  "quote": "std::atomic<bool> _connected;",
  "risk": "",
  "verification_status": "verified"
}
```

```json
{
  "id": "C8-P2-10",
  "title": "Constants in code vs config: WS port and parser limits not in config/ header",
  "file": "src/main_robot.cpp:41 ; src/input/WebSocketCommandProducer.cpp:18-22 ; src/input/CommandFrameParser.h:32",
  "line_range": "",
  "quote": "constexpr uint16_t WS_PORT                 = 80;",
  "risk": "",
  "verification_status": "verified"
}
```

```json
{
  "id": "C8-P2-11",
  "title": "Dead/under-used: BNO055IMU::isCalibrated() has no runtime caller",
  "file": "src/imu/BNO055IMU.h:82-84",
  "line_range": "",
  "quote": "bool isCalibrated() {",
  "risk": "",
  "verification_status": "verified"
}
```

```json
{
  "id": "C8-P2-12",
  "title": "Parser silently truncates: messages of exactly `kCommandFrameMaxLen` are rejected, between (max-len, ∞) crashes prevented but no clear contr …[truncated]",
  "file": "src/input/CommandFrameParser.h:35-56",
  "line_range": "",
  "quote": "if (length == 0 || length >= kCommandFrameMaxLen) {",
  "risk": "",
  "verification_status": "verified"
}
```

### Cell 9

```json
{
  "id": "C9-P2-03",
  "title": "main_robot.cpp and main_drivetrain_test.cpp duplicate the entire motor/PID/drivetrain bring-up verbatim",
  "file": "code/body-firmware/src/main_robot.cpp",
  "line_range": "",
  "quote": "L298NDriver g_driver0(MOTOR0_PINS);\nL298NDriver g_driver1(MOTOR1_PINS);\nL298NDriver g_driver2(MOTOR2_PINS);\n\nFIT0186Motor<5> g_motor0(g_driver0, MOTOR0_ENCODER);\nFIT0186Motor<5> g_motor1(g_driver1, MO …[truncated]",
  "risk": "",
  "verification_status": "verified"
}
```

```json
{
  "id": "C9-P2-04",
  "title": "Test mains use ad-hoc String.indexOf parsing instead of CommandFrameParser",
  "file": "code/body-firmware/src/main_drivetrain_test.cpp",
  "line_range": "",
  "quote": "    int firstSpace = line.indexOf(' ');\n    int secondSpace = line.indexOf(' ', firstSpace + 1);\n\n    if (firstSpace < 0 || secondSpace < 0) {\n        Serial.println(\"ERR: format is <vx> <vy> <omega_d …[truncated]",
  "risk": "",
  "verification_status": "verified"
}
```

```json
{
  "id": "C9-P2-05",
  "title": "main_imu_test.cpp owns motor-pin teardown logic that should live with the motor module",
  "file": "code/body-firmware/src/main_imu_test.cpp",
  "line_range": "",
  "quote": "static void killMotorPins() {\n    constexpr uint8_t pins[] = {\n        MOTOR0_PINS.fwd, MOTOR0_PINS.rev,\n        MOTOR1_PINS.fwd, MOTOR1_PINS.rev,\n        MOTOR2_PINS.fwd, MOTOR2_PINS.rev,\n    }; …[truncated]",
  "risk": "",
  "verification_status": "verified"
}
```

```json
{
  "id": "C9-P2-06",
  "title": "BB8_ASSERT is a no-op on every flashed env — silent runtime gap",
  "file": "code/body-firmware/src/config/debug.h",
  "line_range": "",
  "quote": "#ifdef BB8_DEBUG\n\n#ifndef BB8_ASSERT_HANDLER\n#define BB8_ASSERT_HANDLER(msg, file, line) do { \\\n    Log.fatal(\"ASSERT FAILED: %s (%s:%d)\" CR, msg, file, line); \\\n    abort(); \\\n} while(0)\n#endif\n\n#def …[truncated]",
  "risk": "",
  "verification_status": "verified"
}
```

```json
{
  "id": "C9-P2-07",
  "title": "Lifecycle is a 2-method interface used by exactly one implementer — barely earning its keep",
  "file": "code/body-firmware/src/util/Lifecycle.h",
  "line_range": "",
  "quote": "class Lifecycle {\npublic:\n    virtual ~Lifecycle() = default;\n\n    // Bring the component online: start any FreeRTOS task, open sockets, etc.\n    virtual void start() = 0;\n\n    // Bring the component …[truncated]",
  "risk": "",
  "verification_status": "verified"
}
```

```json
{
  "id": "C9-P2-08",
  "title": "Vec3 / Quat / Euler / CalStatus are minimal but lack equality / printing — bound to be reinvented",
  "file": "code/body-firmware/src/util/Vec3.h",
  "line_range": "",
  "quote": "struct Vec3 {\n    float x = 0;\n    float y = 0;\n    float z = 0;\n};",
  "risk": "",
  "verification_status": "verified"
}
```

```json
{
  "id": "C9-P2-09",
  "title": "platformio.ini env sprawl: 5 envs, 4 of them duplicate the same exclusion list with one toggle changed",
  "file": "code/body-firmware/platformio.ini",
  "line_range": "",
  "quote": "[env:drivetrain_test]\nextends = env:wemos_d1_uno32\nbuild_src_filter = +<*> -<motor/driver/BTS7960Driver.cpp> -<main_original.cpp> -<main_blink.cpp> -<main_motor_test.cpp> -<main_imu_test.cpp> -<main_r …[truncated]",
  "risk": "",
  "verification_status": "verified"
}
```

```json
{
  "id": "C9-P2-10",
  "title": "Inconsistent main file naming — main_original is the lone non-conforming entry",
  "file": "code/body-firmware/src/main_original.cpp",
  "line_range": "",
  "quote": " * BB-8 Body Firmware — Motor Spin Test\n * Serial input: \"<motor> <speed>\" e.g. \"0 0.5\" or \"1 -1.0\"\n * \"stop\" to brake all motors",
  "risk": "",
  "verification_status": "verified"
}
```

```json
{
  "id": "C9-P2-11",
  "title": "config/pins.h pulls in L298NDriver.h and FIT0186.h transitively — pins should not know drivers",
  "file": "code/body-firmware/src/config/pins.h",
  "line_range": "",
  "quote": "#include \"L298NDriver.h\"\n#include \"MotorConfig.h\"\n#include \"FIT0186.h\"\n\ninline constexpr L298NPins MOTOR0_PINS = {.fwd = 14, .rev = 27};",
  "risk": "",
  "verification_status": "verified"
}
```

```json
{
  "id": "C9-P2-12",
  "title": "main_imu_test.cpp hard-codes I2C pins as 21,22 instead of importing from a shared constant",
  "file": "code/body-firmware/src/main_imu_test.cpp",
  "line_range": "",
  "quote": "    Wire.begin(21, 22);",
  "risk": "",
  "verification_status": "verified"
}
```

```json
{
  "id": "C9-P2-13",
  "title": "main_robot.cpp boots heap objects without nullptr defense in controlTask",
  "file": "code/body-firmware/src/main_robot.cpp",
  "line_range": "",
  "quote": "    while (true) {\n        // 1) Pull freshest command from latch.\n        auto fresh = g_latch->read();",
  "risk": "",
  "verification_status": "verified"
}
```

### Cell 10

```json
{
  "id": "C10-P2-01",
  "title": "now = millis() captured once but used both for fresh-time-stamp and age — fine, but document the convention",
  "file": "src/main_robot.cpp",
  "line_range": "162-175",
  "quote": "auto fresh = g_latch->read();\n        const uint32_t now = millis();\n        if (fresh.has_value()) {\n            last_known      = *fresh;\n            last_fresh_ms   = now;\n            have_seen_fre …[truncated]",
  "risk": "Staleness uses the **consumer-side receive timestamp** (`millis()` when controlTask drained the latch), not the **producer-side publish timestamp** (when ws_prod called latch.write …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C10-P2-02",
  "title": "OmniDrivetrain::drive is called every tick even when command unchanged — kinematics + saturation recomputed needlessly",
  "file": "src/drivetrain/controller/PassthroughDrivetrainController.h",
  "line_range": "17-19",
  "quote": "void update(const BodyVelocity& command, const IMUReading& /*imuData*/) override {\n        _drivetrain.drive(command);\n    }",
  "risk": "Context §9.15 flagged this. `PassthroughDrivetrainController::update` always calls `_drivetrain.drive(command)`, which recomputes `OmniKinematics::toWheelRPMs` (saturation scan, th …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C10-P2-03",
  "title": "Producer backpressure on jammed consumer is correct (drop-newest via overwrite) but not asserted by test",
  "file": "src/util/sync/CommandLatch.h",
  "line_range": "25-27",
  "quote": "void write(const T& value) {\n        xQueueOverwrite(_queue, &value);\n    }",
  "risk": "If the control task hangs (won't, TWDT fires) or is otherwise slower than the producer, `xQueueOverwrite` replaces the queued value rather than blocking — the producer never stalls …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C10-P2-04",
  "title": "BodyVelocity has no NaN/Inf guard at the seam — non-finite floats traverse the entire pipeline",
  "file": "src/drivetrain/BodyVelocity.h",
  "line_range": "1-7",
  "quote": "#pragma once\n\nstruct BodyVelocity {\n    float vx = 0;     // forward speed, m/s (positive = forward)\n    float vy = 0;     // strafe speed, m/s (positive = left)\n    float omega = 0;  // rotation rate …[truncated]",
  "risk": "`std::strtof` parses 'nan', 'inf', '-inf' as valid IEEE values. `CommandFrameParser` accepts them. The latch transports them. The staleness ramp multiplies them by `scale` (still N …[truncated]",
  "verification_status": "verified"
}
```

### Cell 11

```json
{
  "id": "C11-P2-12",
  "title": "Zero-command coast vs hold-position on an incline",
  "file": "src/control/PID.h",
  "line_range": "22-26",
  "quote": "if (_deadband > 0.0f && std::abs(setpoint) < 1e-6f && std::abs(measurement) < _deadband) {\n            reset();\n            return 0.0f;\n        }",
  "risk": "Definition of 'motors off' in this codebase = setpoint=0 to PID, which under the deadband snap (setpoint≈0 AND |measured|<5 RPM) returns 0.0 and resets the integrator. If the robot …[truncated]",
  "verification_status": "verified"
}
```

```json
{
  "id": "C11-P2-13",
  "title": "Producer rejected client path can leak repeated connect attempts (no rate limit)",
  "file": "src/input/WebSocketCommandProducer.cpp",
  "line_range": "121-132",
  "quote": "case WStype_CONNECTED: {\n            // Single-client policy: accept the first client, reject the rest.\n            // Two operators sending commands would oscillate the latch.\n            uint8_t exp …[truncated]",
  "risk": "A second connecting client is disconnected, but each rejection runs `Serial.printf` synchronously on the ws_prod task. An attacker (or buggy client) hammering connect attempts can …[truncated]",
  "verification_status": "verified"
}
```

### Cell 12

```json
{
  "id": "C12-P2-01",
  "title": "OmniDrivetrain spec setMotorSpeeds is a separate helper; in code it's an inline private one-liner",
  "file": "src/drivetrain/OmniDrivetrain.h",
  "line_range": "46-50",
  "quote": "    void setMotorSpeeds(float s0, float s1, float s2) {\n        _motors[0]->setSpeed(s0);\n        _motors[1]->setSpeed(s1);\n        _motors[2]->setSpeed(s2);\n    }",
  "risk": "None — purely structural.",
  "spec_file": "docs/superpowers/specs/2026-05-06-omni-drivetrain-design.md",
  "spec_line_range": "146-148",
  "spec_quote": "### `setMotorSpeeds(s0, s1, s2)`\n\nHelper that sets each motor to its respective speed.",
  "verification_status": "verified"
}
```

```json
{
  "id": "C12-P2-02",
  "title": "FIT0186Motor::_encoder is public for testing; spec doesn't mention this hack",
  "file": "src/motor/FIT0186Motor.h",
  "line_range": "70-72",
  "quote": "    // Public for testing — test code sets _encoder._count directly\n    ESP32Encoder _encoder;",
  "risk": "Encapsulation leak in service of a test. Doesn't affect runtime.",
  "spec_file": "docs/superpowers/specs/2026-04-23-fit0186-motor-design.md",
  "spec_line_range": "211-243",
  "spec_quote": "Concrete `Motor` implementation. Owns a `Driver&` reference and an `ESP32Encoder` instance.",
  "verification_status": "verified"
}
```

```json
{
  "id": "C12-P2-03",
  "title": "controller-discussion uses 0.03m wheel radius in stall-force calculation; RobotConstants.h says 0.046225m",
  "file": "src/drivetrain/RobotConstants.h",
  "line_range": "9-9",
  "quote": "constexpr float WHEEL_RADIUS  = 0.046225f;  // 9.245 cm diameter / 2, in meters",
  "risk": "The controller-discussion's 'force-to-weight ratio 13:1' argument is sized on a 30mm wheel; the real wheel is 46.225mm. Recomputed stall force = 3 × 1.77 / 0.046225 = ~115N (still …[truncated]",
  "spec_file": "docs/controller-discussion.md",
  "spec_line_range": "17-17",
  "spec_quote": "The FIT0186 motors produce ~177N combined stall force (3 motors × 1.77 N·m stall torque / 0.03m wheel radius).",
  "verification_status": "verified"
}
```

```json
{
  "id": "C12-P2-04",
  "title": "OmniKinematics asserts tilt is not near pi/2, but BB8_ASSERT is a no-op on hardware",
  "file": "src/drivetrain/OmniKinematics.h",
  "line_range": "15-16",
  "quote": "        BB8_ASSERT(std::abs(cosf(config.tiltAngle)) > 1e-6f,\n                   \"OmniKinematics: tiltAngle too close to pi/2\");",
  "risk": "On production builds BB8_ASSERT is a no-op (see C12-P1-10). The 1/cos(α) divide by ~0 produces inf which propagates into RPM targets → saturation kicks in → all wheels capped at MA …[truncated]",
  "spec_file": "docs/omni-kinematics-derivation.md",
  "spec_line_range": "73-74",
  "spec_quote": "At α = 0 (vertical wheels, flat floor), cos(0) = 1 and this reduces to the standard equation. As α → 90° (wheels horizontal), cos(α) → 0 and the required wheel speed → ∞, which is the physical limit where the wheels can no longer drive the sphere.",
  "verification_status": "verified"
}
```

```json
{
  "id": "C12-P2-05",
  "title": "Motor spec setSpeed(0) = coast; BTS7960Driver setOutput(0) calls Disable() (not coast in the same sense)",
  "file": "src/motor/driver/BTS7960Driver.cpp",
  "line_range": "20-23",
  "quote": "    if (value == 0.0f) {\n        _hbridge.Disable();\n        return;\n    }",
  "risk": "BTS7960's `Disable()` (drives the L_EN/R_EN low) tri-states the H-bridge — terminals high impedance. The L298N coast (both PWM at 0) leaves the H-bridge enabled with both outputs l …[truncated]",
  "spec_file": "docs/superpowers/specs/2026-04-23-fit0186-motor-design.md",
  "spec_line_range": "104-104",
  "spec_quote": "- `setSpeed(0)` — coast (both H-bridge outputs LOW, motor disconnected)",
  "verification_status": "verified"
}
```

```json
{
  "id": "C12-P2-06",
  "title": "imu-test-procedure says SDA=26, SCL=25; main_robot.cpp uses SDA=21, SCL=22",
  "file": "src/main_robot.cpp",
  "line_range": "53-54",
  "quote": "constexpr int I2C_SDA = 21;\nconstexpr int I2C_SCL = 22;",
  "risk": "Operator following the test procedure will wire the IMU to the wrong pins and `g_imu.begin()` will fail, triggering `ESP.restart()` (main_robot.cpp:222-223). Reboot loop until rewi …[truncated]",
  "spec_file": "docs/imu-test-procedure.md",
  "spec_line_range": "11-17",
  "spec_quote": "| BNO055 Pin | ESP32 Pin |\n|------------|-----------|\n| VIN        | 3.3V      |\n| GND        | GND       |\n| SDA        | GPIO 26   |\n| SCL        | GPIO 25   |",
  "verification_status": "verified"
}
```

## Unverified findings (suspected hallucinations — NOT included in synthesis)

- `C11-P1-11` | Zero test coverage of failure paths | file: `test/` | line_range: `all` | quote: `test_omni_drivetrain/test_omni_drivetrain.cpp:    dt.stop();` — file referenced does not resolve to a real file under code/body-firmware/
