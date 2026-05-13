# Body Firmware Review — SUMMARY

**Date:** 2026-05-11
**Anchored at commit:** f8ca95f (f8ca95ff7fd8117f732fb026e019d957cf209645)
**Pipeline:** 1 context agent -> 12 cell agents -> 1 verifier -> 1 synthesizer
**Verified findings:** 168 (28 P0, 72 P1, 68 P2)

> **Read order**: §1 first (the prioritized action list — what to fix). §2 next (cross-cutting themes — why these issues exist). §3 if you want disagreements between cells. §4 onward is appendix.

The headline: most P0s collapse to two root causes — (a) the failsafe paths do not actually stop motors, and (b) untrusted floats enter the pipeline unchecked. Fix those two and a dozen findings retire at once. Everything else (PID quantization, boot ordering, IMU silence) is real but secondary.

---

## 1. Prioritized action list

A single flat list. Each item is one **action** (not one finding); actions are deduped across cells. Sorted by what would most reduce risk on the next on-robot session.

### P0 actions

#### 1. Wire `OmniDrivetrain::stop()` into every failsafe path and zero the latch on disconnect/OTA
- **Files:** `src/main_robot.cpp:104-110, 144-206`; `src/input/WebSocketCommandProducer.cpp:68-78, 134-142`; `src/drivetrain/controller/PassthroughDrivetrainController.h:21-23`
- **What's wrong:** Every shutdown path in the runtime (WS disconnect, half-open TCP, OTA flash, IMU init failure, WiFi reboot) sets PID setpoint to zero and lets the integrator settle. `OmniDrivetrain::stop()` (which actually calls `motor->brake()` and resets PIDs and `_targetRPMs`) is **dead code** — no runtime trigger calls it. Worst case: half-open TCP keeps the latch holding the last non-zero command for ~6 s before heartbeat detection (C11-P0-01 makes this an unbounded duration on stale-frame replay), so motors continue under PID control until a 200 ms ramp begins — and even then they coast, not brake.
- **Fix:** (a) On `WStype_DISCONNECTED` for the active client, write `BodyVelocity{0,0,0}` to the latch. (b) On producer `stop()`, do the same before deleting the task. (c) In `ArduinoOTA.onStart`, call `g_controller->stop()` **before** `g_producer->stop()`, and `vTaskSuspend(controlTaskHandle)`. (d) Once `age >= STALENESS_TIMEOUT_MS` in `controlTask`, latch a one-shot `g_controller->stop()` instead of feeding zeros forever.
- **Sources:** C1-P0-03, C2-P1-01, C3-P0-04, C3-P1-08, C4-P0-03, C6-P0-03, C10-P0-01, C10-P0-02, C11-P0-04, C11-P0-05, C11-P1-07, C11-P1-08, C11-P2-12
- **Effort:** M

#### 2. Reject non-finite and out-of-range floats at the parser boundary
- **Files:** `src/input/CommandFrameParser.h:46-58`; `src/drivetrain/OmniDrivetrain.h:18-23`; `src/motor/driver/L298NDriver.cpp:20-22`
- **What's wrong:** `std::strtof` parses `"nan"`, `"inf"`, `"-inf"`, `"1e30"` as valid floats. Nothing downstream catches them. `BodyVelocity{NaN,...}` poisons the kinematics, propagates through PID, and reaches `static_cast<uint8_t>(NaN * 255.0f)` in `L298NDriver::setOutput` — undefined behavior on the H-bridge. `BB8_ASSERT` is a no-op in every flashed env (only `[env:native]` defines `-DBB8_DEBUG`), so the existing range checks do not fire on hardware.
- **Fix:** In `parseCommandFrame`, after each `strtof`, reject `!std::isfinite(v)` and any `|v|` above per-axis caps (`kMaxVxMps`, `kMaxOmegaRadps`) added to `RobotConstants.h`. Defensive `std::isfinite`/`std::clamp` in `OmniDrivetrain::drive()` and `L298NDriver::setOutput()` as the last line of defense. Replace the assert-only checks with a `BB8_RUNTIME_CHECK` macro that runs on hardware.
- **Sources:** C1-P0-01, C1-P1-08, C2-P0-01, C2-P0-02, C2-P1-04, C3-P1-05, C9-P2-06, C10-P2-04, C12-P1-10
- **Effort:** S

#### 3. Fix boot ordering so motors are actively zeroed before any blocking init
- **Files:** `src/main_robot.cpp:210-247`
- **What's wrong:** Current order: `Serial.begin -> Wire.begin -> motor.begin (pins go OUTPUT) -> imu.begin -> connectWifi (<=30 s) -> producer.start -> OTA -> TWDT init -> controlTask create`. Between motor pinmode and control task creation there is a window of up to ~30 s where H-bridge GPIOs are configured outputs but no task is enforcing zero. WiFi join failure reboots in a tight loop. Pins also can float between `Wire.begin` and `motor.begin`, and `main_imu_test.cpp` has a `killMotorPins()` helper that production never calls. Separately, `g_producer->start()` runs before `controlTask` is created, briefly leaving the WS server live with no consumer of the latch.
- **Fix:** Reorder setup to: (1) `killMotorPins()` as the first line of `setup()`, driving the six H-bridge IN GPIOs LOW before anything else; (2) `esp_task_wdt_init(1, true)` before any task creation; (3) construct latch/controller and start `controlTask` **before** `connectWifi()`; (4) start `g_producer` last. Treat the control task as the always-on safety baseline.
- **Sources:** C3-P0-01, C3-P0-02, C3-P0-03, C6-P1-10, C11-P1-06
- **Effort:** S

#### 4. Fix the staleness ramp so a non-zero stuck stream and a yanked cable both go to zero
- **Files:** `src/main_robot.cpp:150-183`; `src/input/CommandFrameParser.h`
- **What's wrong:** The ramp resets `last_fresh_ms` whenever **any** frame lands in the latch — even an identical-value replay from a stuck client, proxy, or OS retransmit. `connected()` is intentionally ignored on the hot path, so half-open-TCP-with-replay-frames bypasses the ramp entirely; motors hold the last command indefinitely. Separately, the ramp scales `omega` and `vx/vy` identically with one 200 ms window — yaw drift in a sphere bot is arguably the most dangerous failure mode and deserves a tighter window.
- **Fix:** Extend the wire protocol to `"seq,vx,vy,omega"` (forward-compat because the parser tolerates trailing fields). Reject frames whose `seq` did not advance. Add `ROTATION_STALENESS_MS = 50` distinct from the 200 ms translation window.
- **Sources:** C2-P1-03, C10-P1-01, C10-P1-03, C11-P0-01, C11-P1-07
- **Effort:** M

#### 5. Make `IMU::read()` reportable; detect silent I2C failure
- **Files:** `src/imu/IMU.h:8-16`; `src/imu/BNO055IMU.h:33-80`; `src/main_robot.cpp:186-187`
- **What's wrong:** `Adafruit_BNO055` returns zeros/last-state on bus error. `BNO055IMU::read()` returns an `IMUReading` whose defaults (identity quaternion, zero gyro) are indistinguishable from "valid, upright, motionless". Passthrough discards the reading today so blast radius is bounded — but the tilt-limit controller mandated by `controller-discussion.md` will trust this data. A dead BNO055 will become a silent unsafe-control input the moment the stub gets replaced.
- **Fix:** Return `std::optional<IMUReading>` (or add `IMUReading::valid`). Track consecutive failures; on N failures, treat IMU as stale (same effect as command staleness — zero + brake when a non-passthrough controller is active). Wire `isCalibrated()` into the controller so 0/3/2/1 cal status does not enter stabilization modes.
- **Sources:** C2-P0-03, C2-P1-05, C11-P0-02, C2-P2-02, C8-P2-11
- **Effort:** S

#### 6. Detect per-motor encoder/stall faults before PID windup turns one wheel into a runaway
- **Files:** `src/drivetrain/OmniDrivetrain.h:25-33`; `src/motor/FIT0186Motor.h`
- **What's wrong:** If an encoder lead disconnects or a wheel stalls while the operator commands non-zero, `getFilteredRPM()` reports ~0 against a non-zero `_targetRPMs[i]`. PID error stays at full setpoint, integrator winds to its anti-windup clamp, output saturates at +1.0. The other two wheels keep tracking. Result: one wheel at full PWM, two on PID — net body velocity is **not** the operator command, and recovery requires a power cycle.
- **Fix:** Add a per-motor fault detector in `FIT0186Motor`: if `|commanded PWM| > 0.3` for >N ms AND `|measured RPM| < threshold`, raise a fault flag. `OmniDrivetrain::update()` then routes all three motors through `stop()` and reports the fault. Tie into Action 1's safety stop.
- **Sources:** C11-P0-03, C1-P1-06, C4-P1-08
- **Effort:** M

#### 7. Get the 100 Hz control loop off `Serial.printf` and onto a real elapsed `dt`
- **Files:** `src/main_robot.cpp:190-202`
- **What's wrong:** Two related issues. (a) `dt = CONTROL_PERIOD_MS/1000` is hard-coded — under jitter the PID integrator and derivative term lie about wall-clock time, and the BNO055 NVS calibration save (C5-P1-02) can overrun a tick by 4x while PID still believes `dt=10 ms`. (b) The HWM dump uses `Serial.printf` inline on the hot path; one Serial flush at 115200 baud can blow the 10 ms budget.
- **Fix:** Measure actual `dt` via `micros()` in `controlTask`, pass that to `drivetrain.update()`. Move the HWM dump to a debug-only env or to the Arduino `loopTask` (10 Hz). Make the calibration NVS save deferred to a low-priority background task or to `loopTask`.
- **Sources:** C4-P0-01, C5-P1-01, C5-P1-02, C6-P0-01, C6-P0-02, C12-P0-03, C12-P1-05
- **Effort:** S

#### 8. Resolve `ENCODER_CPR` spec drift before any PID retune
- **Files:** `src/motor/constants/FIT0186.h:11`; `docs/superpowers/specs/2026-04-23-fit0186-motor-design.md:176-179`
- **What's wrong:** Code says `ENCODER_CPR = 2800`, spec says `700`. The factor of 4 corresponds to single-edge vs quadrature counting; `ESP32Encoder::attachFullQuad` is what the motor uses, so 2800 is the right number in code-space. But the spec disagreement means anyone retuning PID from the spec will be 4x off; saturation in `OmniKinematics` also caps at the no-load RPM value, so the mismatch silently bounds achievable speeds.
- **Fix:** **Update the spec** (not the code) to say "700 counts/rev gearbox output, x4 quadrature edge counts = 2800 counts/rev seen by software". Add a comment in `FIT0186.h` linking back to the spec.
- **Sources:** C12-P0-01
- **Effort:** S

#### 9. Throttle and authenticate the WebSocket entry point
- **Files:** `src/input/WebSocketCommandProducer.cpp:117-179`
- **What's wrong:** WS is unauthenticated `ws://` on port 80. A second connecting client is `disconnect()`-ed but each rejection runs a synchronous `Serial.printf` on the ws_prod task; an attacker hammering connect attempts can starve the heartbeat poll. The periodic counter dump (`Serial.printf("[ws] frames=%u ...")`) fires every 1000 frames unconditionally and can interleave with WS event handling. No rate-limit on parse-failure storms.
- **Fix:** Move connect-reject logging behind the same rate-limited helper as parse-fail. Drop the unconditional periodic counter dump or gate it on a debug flag. Add a parse-failure rate ceiling (e.g. >50/s for >2 s -> `_server->disconnect(activeClient)`). Document the LAN-trust assumption.
- **Sources:** C5-P0-04, C2-P1-02, C5-P1-03, C5-P1-06, C10-P1-06, C11-P2-13
- **Effort:** S

#### 10. Document and re-evaluate L298N `brake()` semantics
- **Files:** `src/motor/driver/L298NDriver.cpp:39-42`; `docs/superpowers/specs/2026-04-23-fit0186-motor-design.md:104-106`
- **What's wrong:** Spec was written exclusively against BTS7960 (whose brake calls `_hbridge.Stop()`). Production uses L298N. L298N "brake" writes 255/255 to the PWM pins — at 1 kHz with a real L298N this is electrically closer to coast than to brake (the carrier is fast enough that the H-bridge sees ~100% duty on both sides), so the supposedly active-stop behavior may degrade to coast on real hardware. Action 1 routes failsafes through `brake()`; if brake is effectively coast, Action 1 only solves the unbounded-time-to-stop problem, not the "actively decelerate" problem.
- **Fix:** Empirically verify L298N brake on real hardware (clamp meter on motor leads, current at start of brake vs coast). If 255/255 is effectively coast, either raise PWM frequency via `analogWriteFrequency()` (or `ledcSetup`) on the H-bridge pins or implement brake by holding `setOutput(0)` *and* asserting an EN-low equivalent. Update the motor-design spec to describe both drivers.
- **Sources:** C4-P1-09, C12-P0-02, C1-P1-04, C7-P1-03, C12-P2-05
- **Effort:** S to verify, M to mitigate if confirmed

### P1 actions

#### 11. Make `PID::compute()` symmetric and crash-resistant
- **Files:** `src/control/PID.h:20-48`
- **What's wrong:** Three related issues. (a) `if (dt < 1e-4) return _lastOutput` — if `_lastOutput` was at +1.0 saturation, a degenerate caller could leave the motor wired to full forward indefinitely. (b) Deadband only fires when `setpoint ~ 0`; non-zero sub-deadband setpoints (e.g. 2 RPM) accumulate integral. (c) Anti-windup uses `_outputMax / _ki` with no sign check on `_ki`; a negative gain inverts the clamp silently. (d) Derivative-on-measurement reads the staleness ramp as a real velocity signal, kicking D every time silence engages.
- **Fix:** Return 0 (not `_lastOutput`) when `dt < 1e-4`. Apply deadband symmetrically: if `|setpoint| < deadband`, force `setpoint = 0` AND freeze the integrator. Assert `_ki >= 0` and `_outputMin <= _outputMax` in the constructor. Switch to clamping-on-output anti-windup (only accumulate integral when doing so does not push output past clamp).
- **Sources:** C1-P0-02, C1-P1-01, C1-P1-02, C4-P1-06, C4-P1-07
- **Effort:** M

#### 12. Rationalize concurrency: atomics where they're needed, single-thread proofs where they're not
- **Files:** `src/main_robot.cpp:83`; `src/util/MovingAverage.h`; `src/input/MockCommandProducer.h:17-30`
- **What's wrong:** `g_lastWifiConnectedMs` is `volatile uint32_t` and is written by both the WiFi event task (Core 0) and `loop()` (Core 1) — torn-word risk on Xtensa is small but the standard does not guarantee atomicity. `MovingAverage<5>` is documented as single-thread but the contract is implicit. `MockCommandProducer` uses plain bool flags while the real producer uses `std::atomic` — interface contract is ambiguous on threading.
- **Fix:** Make `g_lastWifiConnectedMs` `std::atomic<uint32_t>`. Add a `static_assert`/comment that `MovingAverage<N>` is single-threaded. Match the mock's flag types to the real producer's `std::atomic`.
- **Sources:** C3-P1-03, C3-P1-06, C5-P1-09, C6-P1-07, C2-P1-7, C8-P2-09
- **Effort:** S

#### 13. Subscribe Arduino `loopTask` to TWDT or move its watchdog work
- **Files:** `src/main_robot.cpp:252-264`
- **What's wrong:** The Arduino `loopTask` runs `ArduinoOTA.handle()` and the WiFi-offline reboot watchdog. It is not subscribed to TWDT. If `ArduinoOTA.handle()` ever deadlocks (malformed packet, library bug), the 30 s WiFi reboot is silently bypassed and the control task happily keeps driving motors against the last command.
- **Fix:** Add `esp_task_wdt_add(NULL); esp_task_wdt_reset()` inside `loop()`. The 100 ms `vTaskDelay` gives ample slack against 1 s. Alternatively move the WiFi-offline timer to a dedicated FreeRTOS timer with explicit TWDT subscription.
- **Sources:** C3-P1-02, C6-P1-05, C11-P1-10
- **Effort:** S

#### 14. Add brownout policy and brake-on-shutdown
- **Files:** `src/main_robot.cpp:210-250`
- **What's wrong:** No `esp_register_shutdown_handler`, no `esp_brownout_register_callback`. On brownout the ESP32-Arduino default detector resets the chip; GPIOs go high-Z and L298N inputs floating yield 0 V (coast). No software opportunity to brake.
- **Fix:** Register a shutdown handler that calls `g_drivetrain.stop()` (active brake). Optionally hook `esp_brownout_register_callback` to brake before reset. Raise the brownout threshold modestly if the battery droop margin allows.
- **Sources:** C11-P1-09
- **Effort:** M

#### 15. Add `Motor::reset()` so filters are cleared with PIDs
- **Files:** `src/motor/FIT0186Motor.h:49-51`; `src/drivetrain/OmniDrivetrain.h:39-43`
- **What's wrong:** `OmniDrivetrain::stop()` resets PID state but leaves `MovingAverage<5> _filter` holding stale samples. When motion resumes, the first 5 ticks of PID see a lagged measurement.
- **Fix:** Add `Motor::reset()` / `FIT0186Motor::resetFilter()`. Call from `stop()` alongside `pid->reset()`.
- **Sources:** C1-P1-06
- **Effort:** S

#### 16. Reduce `MAX_RPM` saturation ceiling below physical no-load
- **Files:** `src/drivetrain/RobotConstants.h`; `src/drivetrain/OmniKinematics.h:45-50`
- **What's wrong:** Saturation cap is set to FIT0186 no-load RPM (251), the physical ceiling where torque margin is zero. PID can demand 251 RPM but cannot reach it under any load; integrator pegs and stays pegged.
- **Fix:** Reduce `MAX_RPM` to ~80% of no-load (e.g. 200). Rename to `MAX_CONTROLLABLE_RPM`. Add a defensive clamp in `OmniDrivetrain::drive()` at the `_targetRPMs` write site.
- **Sources:** C1-P1-03, C12-P1-04
- **Effort:** S

#### 17. Enforce `Driver` brake/coast contract; cache enable state in BTS7960
- **Files:** `src/motor/driver/L298NDriver.cpp`; `src/motor/driver/BTS7960Driver.cpp`
- **What's wrong:** L298N and BTS7960 produce electrically different behavior for identical `setOutput()` inputs. BTS7960 also toggles `Enable()`/`Disable()` on every nonzero call — at 100 Hz this wears the gate driver. Float-equality against 0.0 (`if (value == 0.0f)`) also misses sub-LSB-PWM values.
- **Fix:** Cache enable state and only call `Enable()/Disable()` on transition. Replace `value == 0.0f` with `std::abs(value) < 1.0f/256.0f`. Document divergence in `Driver.h` or remove one driver entirely.
- **Sources:** C1-P1-04, C7-P1-03, C12-P0-02
- **Effort:** S

#### 18. Defer or remove the per-tick IMU read while passthrough is the controller
- **Files:** `src/main_robot.cpp:186-187`; `src/imu/BNO055IMU.h:33-80`
- **What's wrong:** Control task does 4 I2C transactions every tick (`Quat + Euler + Gyro + Cal`) for a controller that discards the result. BNO055 can stretch up to 1 ms; under contention the 10 ms budget is pressured for no benefit.
- **Fix:** Either (a) gate the read on `_controller->wantsIMU()`, (b) move IMU reads to a separate Core 0 task that publishes through a second latch, or (c) cut the field mask to only what `isCalibrated()` truly needs. Tie into Action 5.
- **Sources:** C2-P2-02, C5-P1-01, C8-P2-04
- **Effort:** M

#### 19. Rotate WiFi/OTA credentials and harden the OTA password
- **Files:** `src/config/wifi_credentials.h`
- **What's wrong:** The file is correctly gitignored (CONTEXT §9.1 was incorrect — see §3 below). However the dev-disk copy holds real credentials and a 3-character OTA password (`"bb8"`). Trivial dictionary on the LAN.
- **Fix:** Rotate OTA password to >=16 random chars and source from `${sysenv.OTA_PASSWORD}` (already supported by `[env:robot_ota]`). Move WIFI_PASSWORD to the same env-var pattern. Move the example file's defaults to obviously-fake values.
- **Sources:** C3-P1-01, C9-P1-02
- **Effort:** S

#### 20. Make `g_latch`/`g_producer`/`g_controller` lifetime explicit
- **Files:** `src/main_robot.cpp:79-82, 144-206`
- **What's wrong:** Heap-allocated globals are nullable but `controlTask` dereferences them without a null check. Safe today because `setup()` constructs them before creating the task, but Action 3's reorder (controlTask earlier) breaks this assumption.
- **Fix:** Construct as static storage with a `bool g_safetyDisarmed = true` initial-state flag the control task respects. Flip the flag at end of `setup()` once all globals are ready.
- **Sources:** C3-P1-10, C9-P2-13, C11-P1-06
- **Effort:** S

#### 21. Replace `ESP.restart()` boot loops with degraded modes
- **Files:** `src/main_robot.cpp:131-138, 221-224`
- **What's wrong:** WiFi-timeout and BNO055-init failure both trigger `ESP.restart()` with no backoff. A permanently unpowered BNO055 (a real failure mode after a harness change) puts the robot in a ~200 ms reboot loop forever, preventing reflash because the WS server never comes up.
- **Fix:** Track consecutive failures in RTC slow-mem. After N retries, bring up WiFi + WS + OTA in a "degraded" mode (no IMU, no control task, command latch ignored). Operator can then re-flash.
- **Sources:** C3-P1-07, C5-P2-08
- **Effort:** M

#### 22. Audit pin-doc drift and centralize I2C pins
- **Files:** `docs/motor-spin-test-procedure.md`; `docs/drivetrain-test-procedure.md`; `docs/imu-test-procedure.md`; `src/main_robot.cpp:53-54`; `src/main_imu_test.cpp`; `src/config/pins.h`
- **What's wrong:** All three test procedure docs disagree with `pins.h` after the `f8ca95f` harness remap (motor 2 was 18/19, now 25/26; encoders 1 and 2 have swapped A/B in docs; `imu-test-procedure.md` lists SDA=26/SCL=25 while runtime uses 21/22). `main_blink.cpp` still hard-codes the old pin map.
- **Fix:** Update all three docs to match `pins.h`. Move `I2C_SDA`/`I2C_SCL` into `pins.h` and reference from both `main_robot.cpp` and `main_imu_test.cpp`. Delete `main_blink.cpp` and `main_original.cpp` from `src/` (they exist only as build-filter exclusions).
- **Sources:** C3-P1-04, C3-P2-01, C9-P1-01, C9-P2-05, C9-P2-10, C9-P2-12, C12-P2-06, C3-P1-09
- **Effort:** S

#### 23. Add `BB8_RUNTIME_CHECK` for hot-path invariants
- **Files:** `src/config/debug.h`
- **What's wrong:** `BB8_ASSERT` is a no-op outside `[env:native]`. Every range check on hardware is silently disabled — the asserts in `OmniKinematics` (positive radii, sane tilt angle), `L298NDriver::setOutput` (`[-1,1]` range), `FIT0186Motor` (positive CPR) all elide to nothing on the flashed image.
- **Fix:** Add a `BB8_RUNTIME_CHECK(cond, action)` macro distinct from `BB8_ASSERT` that, on hardware, performs `action` (e.g. clamp the input, increment a fault counter) without aborting. Use it at every safety-critical boundary.
- **Sources:** C3-P1-05, C9-P2-06, C12-P1-10, C12-P2-04
- **Effort:** S to add macro, M to retrofit call sites

#### 24. Add seam-level failure-path tests
- **Files:** `test/`
- **What's wrong:** Native test inventory covers pid, kinematics, drivetrain.stop(), passthrough.stop(), latch, parser, fit0186 motor, drivers, moving average, bno055 imu, mock producer — all single-threaded, all happy-path or boundary. **Zero** tests cover: staleness ramp, WS disconnect -> motors-off, IMU read failure, encoder fault, OTA hook firing, TWDT pretrigger, brownout, non-finite WS frames. Latch tests are single-threaded native; the actual job is cross-core overwrite-with-readback.
- **Fix:** Add `test_staleness_ramp/`, `test_disconnect_recovery/`, `test_imu_failure/`, `test_encoder_fault/`, `test_adversarial_input/` using existing mocks. Add an on-target `test_command_latch_ontarget/` that drives the latch from two FreeRTOS tasks on different cores.
- **Sources:** C2-P1-06, C2-P2-03, C3-P2-05, C10-P1-05, C11-P1-11 (unverified but the gap is real)
- **Effort:** L

#### 25. Tighten the heartbeat/half-open story; surface `parseFailures`
- **Files:** `src/input/WebSocketCommandProducer.h/cpp`; `src/input/CommandProducer.h`
- **What's wrong:** Heartbeat detects half-open in ~6 s. `_connected` is intentionally ignored on the hot path. `CommandProducer` exposes no way to surface invalid frames — `_parseFailCount` is counted but never published. A client typo storm at 100 Hz is indistinguishable from "no signal".
- **Fix:** Add `uint32_t parseFailures() const` and `uint32_t framesSinceLastValid() const` to `CommandProducer`. Control loop logs/raises on first nonzero observation. Also have `ws_prod` disconnect the active client if no valid frame in >2 s. Make heartbeat config configurable.
- **Sources:** C2-P1-08, C2-P1-02, C5-P2-07, C8-P2-01, C8-P2-02, C10-P1-06
- **Effort:** S

#### 26. Spec hygiene: bring specs back in sync with code (after code changes from §1.1)
- **Files:** `docs/superpowers/specs/2026-04-23-fit0186-motor-design.md`; `docs/superpowers/specs/2026-05-06-omni-drivetrain-design.md`; `docs/controller-discussion.md`; `docs/drivetrain-test-procedure.md`
- **What's wrong:** Multiple spec/code mismatches, mostly editorial: PID interface signature missing `dt`; OmniDrivetrain `stop()` snippet missing `_targetRPMs` clear; spec PID gain hints `(Kp=1.0, Ki=0.1, Kd=0.01)` 200x too aggressive vs actual; `controller-discussion.md` says tilt limiting is mandatory while runtime is a stub; 200 ms staleness ramp absent from any spec; controller-discussion uses 0.03 m wheel radius vs real 0.046225 m.
- **Fix:** Decide spec-vs-code direction per item. Most should fix the spec (signature, snippets, gain hint, wheel radius). `controller-discussion.md` should get a "Status: not yet implemented; passthrough is the runtime stub" banner. Add a new short "Teleop loop & staleness policy" spec.
- **Sources:** C12-P0-03, C12-P0-04, C12-P1-01, C12-P1-02, C12-P1-03, C12-P1-04, C12-P1-06, C12-P1-07, C12-P1-08, C12-P1-09, C12-P1-11, C12-P2-03
- **Effort:** M

### P2 actions (compressed)

Each P2 finding is listed once; cite list at right. Suggested follow-ups, none safety-critical.

| # | Action | Sources |
|---|---|---|
| 27 | Precompute `cosAlpha`, `scale` once in `OmniKinematics` constructor; cache saturation-scaled output if command unchanged | C1-P2-02, C4-P2-10, C10-P2-02 |
| 28 | Switch `MovingAverage::average()` to a running sum (O(1) instead of O(N)) | C4-P2-11, C6-P2-14 |
| 29 | Use `static_cast<uint8_t>(roundf(...))` not truncation in `L298NDriver::setOutput` | C1-P2-01 |
| 30 | Hide `FIT0186Motor::_encoder` behind a friend-class or `#ifdef BB8_DEBUG` accessor | C1-P2-03, C7-P2-09, C12-P2-02 |
| 31 | Fix asymmetric `IMUField operator&` (returns `bool`, breaks chaining) | C2-P2-01, C5-P2-11, C8-P2-05 |
| 32 | Add `operator==`, stream/Print formatters to `Vec3/Quat/Euler/CalStatus` | C3-P2-04, C9-P2-08 |
| 33 | Add `bool running() const` to `Lifecycle` interface; reconsider whether `Lifecycle` earns its keep at one implementer | C3-P2-02, C9-P2-07 |
| 34 | Decouple `config/pins.h` from `L298NDriver.h` (pins should not know drivers) | C9-P2-11 |
| 35 | Consolidate the duplicated bring-up between `main_robot.cpp` and `main_drivetrain_test.cpp` into a `RobotAssembly` helper | C9-P2-03, C9-P2-04, C4-P2-12 |
| 36 | Hoist `WS_PORT` and parser limits into `src/input/InputConfig.h` | C8-P2-10, C8-P2-06 |
| 37 | Strongly type `setSpeed(float)` and similar to discourage RPM/normalized confusion | C7-P2-04, C7-P2-08 |
| 38 | Drop the FIT0186 `NO_LOAD_RPM` constant or make `MAX_RPM` reference it (single source of truth) | C7-P1-01, C7-P2-12 |
| 39 | Add a unit conversion header (`util/units.h`) to dedupe rad<->RPM math | C7-P1-02 |
| 40 | Generalize `Drivetrain` base — it currently hardcodes 3 motors | C7-P2-05 |
| 41 | Consolidate driver duplication (L298N vs BTS7960 ~80% same) behind a shared helper | C7-P2-07 |
| 42 | Add doc/PID context comments to non-motor-specific files | C7-P2-06, C7-P2-11 |
| 43 | Reduce `Arduino.h` include leak in `FIT0186Motor.h`, `BNO055IMU.h` | C7-P2-10, C8-P2-08 |
| 44 | Add `parseFailures()` to `CommandProducer`; document parser tolerance grammar | C8-P2-01, C8-P2-06, C8-P2-12, C2-P1-06 |
| 45 | Rename `_connected` to disambiguate transport-up vs active-client | C8-P2-09 |
| 46 | Wire `BNO055IMU::isCalibrated()` into a real consumer (or remove) | C8-P2-11 |
| 47 | Collapse the 5-env `platformio.ini` sprawl with a shared `extends` base for `build_src_filter` exclude lists | C9-P2-09 |
| 48 | Document or wrap `xQueueOverwrite` drop-newest semantics; add a producer-faster-than-consumer stress test | C10-P2-03 |
| 49 | Document that staleness uses consumer-side receive timestamp, not producer-side publish | C10-P2-01 |
| 50 | Move `Serial.printf` cross-task contention behind a logger | C6-P2-12, C5-P0-04 |
| 51 | Stack HWM 8192 B not validated post-OTA — re-check after Action 1 lands | C6-P1-09 |
| 52 | `vTaskDelayUntil` init `lastWake` before TWDT subscribe — verify first-tick deadline | C6-P2-11 |
| 53 | `loop()` 10 Hz tick can starve under control-task stretch; consider Core 0 watchdog for OTA | C6-P2-13 |
| 54 | Latent: `_savedThisBoot` calibration-save can't be re-armed without reboot | C8-P2-11 |
| 55 | Forward-kinematics `OmniKinematics::forward()` not implemented (only `toWheelRPMs`) — note for future odometry | C1-P2-04 |
| 56 | `FIT0186Motor::update()` is not safe under multiple callers — document single-call contract | C4-P1-04 |
| 57 | Encoder pins 34/35/36/39 are input-only — no internal pull-ups; verify external pull-up presence on harness | C4-P1-08 |
| 58 | `OmniDrivetrain::update()` does not check that motors are bound — minor; relies on construction order | C4-P2-13 |
| 59 | TWDT subscribe by control task is single-shot; if `esp_task_wdt_init` is later called twice, behavior depends on ESP-IDF version | C6-P1-06 |
| 60 | Verify FreeRTOS tick rate is 1 kHz so `pdMS_TO_TICKS(10) = 10` exactly | C6-P1-08 |
| 61 | `BodyVelocity` POD lacks finiteness/clamp helpers — see Action 2 | C10-P2-04 |
| 62 | Periodic counter dump every 1000 frames is uncoordinated with rate-limited parse-fail log | C5-P0-04, C11-P2-13 |
| 63 | Add an explicit `setMotorSpeeds` helper to match the spec or update the spec to inline | C12-P2-01 |

---

## 2. Cross-cutting themes

### Theme A — Failsafe paths don't actually stop motors
- **Cells:** C1, C2, C3, C4, C6, C10, C11, C12
- **Findings:** C1-P0-03, C2-P1-01, C3-P0-04, C3-P1-08, C4-P0-03, C6-P0-03, C10-P0-01, C10-P0-02, C11-P0-04, C11-P0-05, C11-P1-07, C11-P1-08, C11-P2-12, C12-P0-02
- **Root cause:** Every shutdown signal (WS disconnect, half-open TCP, OTA, IMU init failure, staleness, WiFi reboot) zeroes the PID *setpoint* and lets the integrator settle, rather than calling `OmniDrivetrain::stop()` (which actively brakes via `motor->brake()` and resets state). `stop()` is dead code: the only caller, `PassthroughDrivetrainController::stop()`, is itself never wired to any runtime signal. Worse, even the brake primitive itself is suspect on L298N at 1 kHz (Action 10).
- **Single fix:** Land **Action 1** as one PR. Wire `g_controller->stop()` into the four shutdown sites; have the producer publish `BodyVelocity{0,0,0}` on disconnect/stop; once `age >= STALENESS_TIMEOUT_MS` latch a brake state instead of feeding zero forever. Verify the brake primitive on real hardware (Action 10) before relying on it.

### Theme B — Untrusted floats traverse the entire pipeline
- **Cells:** C1, C2, C3, C10, C12
- **Findings:** C1-P0-01, C1-P1-08, C2-P0-01, C2-P0-02, C2-P1-04, C3-P1-05, C9-P2-06, C10-P2-04, C12-P1-10
- **Root cause:** The parser accepts `nan`, `inf`, and any magnitude that fits in a float. Defensive checks downstream are `BB8_ASSERT`, which is a no-op outside `[env:native]`. The signed->`uint8_t` cast in `L298NDriver::setOutput` on a NaN is undefined behavior.
- **Single fix:** **Action 2** + **Action 23**. Reject non-finite and out-of-range at the parser. Introduce `BB8_RUNTIME_CHECK` for invariants that must hold on hardware. Defensive clamps in `OmniDrivetrain::drive()` and `L298NDriver::setOutput()` close the door.

### Theme C — `dt` and time accounting is informal across the pipeline
- **Cells:** C1, C4, C5, C6, C12
- **Findings:** C1-P1-05, C4-P0-01, C4-P1-05, C4-P1-06, C5-P1-01, C5-P1-02, C6-P0-01, C6-P0-02, C12-P0-03, C12-P1-05
- **Root cause:** PID receives a hard-coded `dt = 0.010 s`; `FIT0186Motor` computes its own `dt` from `micros()`; the calibration NVS save can overrun the budget by 4x; the 10 ms wall-clock dt is consumed as gospel even when reality jitters. The two `dt` sources have no contract.
- **Single fix:** Measure actual `dt` in `controlTask` via `micros()`, pass that value into `drivetrain.update()`. Move blocking work (Serial.printf, NVS save) off the hot path. Tied to **Action 7**.

### Theme D — Boot sequencing leaves motors armed before the safety baseline exists
- **Cells:** C3, C5, C6, C11
- **Findings:** C3-P0-01, C3-P0-02, C3-P0-03, C5-P2-08, C6-P1-10, C11-P1-06
- **Root cause:** Motors get `pinMode(OUTPUT)` ~30 s before the control task starts (because `connectWifi()` blocks first). The producer starts before the control task is created. TWDT init runs after both task creations. Each step is justifiable in isolation; collectively the boot has a multi-second window with armed motors and no software watchdog enforcing zero.
- **Single fix:** **Action 3**. Reorder: kill motor pins -> init TWDT -> construct globals -> start `controlTask` -> connect WiFi -> start producer. Use a `safetyDisarmed` flag in `controlTask` that forces `drive_cmd=0` until setup completes.

### Theme E — Asserts are a runtime no-op on every flashed firmware
- **Cells:** C1, C3, C9, C12
- **Findings:** C1-P0-01 (parent), C3-P1-05, C9-P2-06, C12-P1-10, C12-P2-04
- **Root cause:** `-DBB8_DEBUG` is only set on `[env:native]`. Every `BB8_ASSERT` in production hardware code is `((void)0)`. The codebase has invested in invariant checking but disabled it where it would catch real bugs.
- **Single fix:** **Action 23**. Add `BB8_RUNTIME_CHECK(cond, action)` that runs on hardware with a clamp/counter rather than abort. Retrofit `OmniKinematics`, `L298NDriver`, `FIT0186Motor`, and `OmniDrivetrain::drive()` to use it.

### Theme F — The wire protocol carries no liveness
- **Cells:** C2, C10, C11
- **Findings:** C2-P1-03, C10-P1-01, C10-P1-02, C10-P1-03, C11-P0-01, C11-P1-07
- **Root cause:** `"vx,vy,omega"` has no sequence number or timestamp. Identical replays from a buggy client/proxy keep `last_fresh_ms` advancing; the staleness ramp never engages on a non-zero stuck stream. Drops, reorders, and replays are invisible. `omega` and `vx/vy` share one staleness window even though yaw drift is the more dangerous failure mode.
- **Single fix:** **Action 4**. Extend the protocol to `"seq,vx,vy,omega"`; reject frames where `seq` did not advance. Per-channel staleness windows (`ROTATION_STALENESS_MS=50` vs translation 200 ms).

### Theme G — Silent failure of sensors that the future controller will trust
- **Cells:** C2, C8, C11
- **Findings:** C2-P0-03, C2-P1-04, C2-P1-05, C8-P2-04, C8-P2-11, C11-P0-02
- **Root cause:** `BNO055IMU::read()` cannot fail in the type system. A dead bus returns identity quaternion and zero gyro — indistinguishable from "upright and motionless". `isCalibrated()` is never consulted by any runtime code. The passthrough controller masks the problem today; a tilt-stabilizer drops into a silent bad-input failure.
- **Single fix:** **Action 5**. Return `std::optional<IMUReading>` or add `IMUReading::valid`. Gate any IMU-based control on a consecutive-failure counter and on `isCalibrated() >= 2` for gyro.

### Theme H — Test coverage is happy-path only; failure paths and concurrency are unverified
- **Cells:** C2, C3, C10, C11
- **Findings:** C2-P2-03, C3-P2-05, C10-P1-05, C2-P1-06, C11-P1-11
- **Root cause:** Native tests are deep on each unit's nominal+boundary behavior, but **zero** cover staleness ramp, disconnect propagation, IMU failure, encoder fault, OTA hook, brownout, non-finite input, or cross-core latch racing. All safety claims in this review are static-analysis only.
- **Single fix:** **Action 24**. Build out a `test/test_seam_*` family using the existing `MockCommandProducer`, mock motor, and mock IMU. Add one on-target test for cross-core latch correctness.

---

## 3. Disagreements between cells

- **`wifi_credentials.h` committed status (C3 vs C9):** CONTEXT.md §9.1 claimed the file was checked in; Cell 3 cited that as a finding (C3-P1-01 originally framed it as committed). Cell 9 (C9-P1-02) verified via `git ls-files` that the file is **gitignored and untracked**. **Believe Cell 9.** The action remains valid (rotate credentials, harden OTA password) because the dev-disk copy still exists and the 3-character OTA password is real, but it should be framed as "rotate" not "remove from history". Action 19 reflects this.

- **`OmniKinematics::forward()` (C1):** Cell 1's review prompt asked to verify `forward()` and `inverse()`. C1-P2-04 correctly notes the function does not exist (only `toWheelRPMs`, the inverse map). No real bug. Marked moot.

- **`OmniDrivetrain::stop()` reach (multiple cells):** C1, C3, C4, C11, C12 all flagged this independently. No disagreement on facts; this is Theme A.

- **Identical-command replay (C2 vs C11):** C2-P1-03 (P1) and C11-P0-01 (P0) describe the same code with different severities. C11's framing emphasizes the half-open-TCP-with-replay-frames case which is unbounded; that escalates to P0. **Believe C11.** Action 4 reflects this.

- **L298N brake semantics (C4 vs C12):** C4-P1-09 says 255/255 at 1 kHz on a real L298N is effectively coast; C12-P0-02 says the spec mandates brake. They are consistent — the code calls `analogWrite(255)` but the hardware may not behave as "brake" intends. Action 10 calls for empirical verification.

- **Cell 11's `C11-P1-11` (test coverage):** Verifier dropped this single finding as suspected hallucination because the citation pointed to `test_omni_drivetrain/test_omni_drivetrain.cpp` which the verifier could not resolve. The gap it describes is **real** (per the Cell 11 prose), but the specific filename was off. Action 24 captures the substance.

---

## 4. What's already strong

- **Pure-functional kinematics with saturation that preserves direction.** `OmniKinematics::toWheelRPMs` does the right thing under high-magnitude inputs (`max-scan + common-factor scale`), and the matrix matches the derivation doc exactly (Cell 1 observation).
- **PID derivative-on-measurement** is correctly implemented (`PID.h:41`) — no spike on setpoint change; covered by `test_derivative_on_measurement_no_spike_on_setpoint_change` (Cell 1).
- **Core pinning and task priorities.** Control task on Core 1 at prio 4; ws_prod on Core 0 at prio 2; `vTaskDelayUntil` phase-locks period; both real tasks subscribe to TWDT (Cell 6).
- **Cross-core handoff uses a FreeRTOS queue with proper barriers.** `xQueueOverwrite`/`xQueueReceive` give drop-newest semantics with no producer stall (Cell 10).
- **Single-client policy via `compare_exchange_strong`** correctly prevents two operators from oscillating the latch (Cell 10).
- **`MovingAverage<N>` uses `static_assert(N > 0)`** and the template-only design avoids heap (Cell 9).
- **Header hygiene.** `WebSocketCommandProducer.h` uses a forward decl for `WebSocketsServer` — avoids dragging WS internals into every translation unit (Cell 8).
- **`CommandFrameParser`** is properly extracted from the producer, header-only, and unit-tested for boundary cases (Cell 8).
- **Boot init guards.** WiFi timeout and BNO055 failure both reboot on failure rather than silently continuing in a degraded state. (Behavior is *too* aggressive — see Action 21 — but the fail-safe direction is right.)
- **TWDT is configured with `panic=true` at 1 s.** Bounds worst-case hang on any subscribed task (Cell 6, Cell 11).
- **Encoder count math is wrap-safe.** Verified by `test_micros_overflow` at `ULONG_MAX` (Cell 1).
- **The `Lifecycle` / `CommandProducer` split** is the right shape — producers that own a task get start/stop without forcing it on the polled-mock pattern (Cell 8).

---

## 5. Coverage map

| Cell | Lens | P0 | P1 | P2 | Total verified |
|------|------|----|----|----|----------------|
| C1 | Safety / actuation | 3 | 8 | 4 | 15 |
| C2 | Safety / sensing | 3 | 8 | 3 | 14 |
| C3 | Safety / glue | 4 | 10 | 5 | 19 |
| C4 | Realtime / actuation | 3 | 6 | 4 | 13 |
| C5 | Realtime / sensing | 1 | 6 | 4 | 11 |
| C6 | Realtime / glue | 3 | 7 | 4 | 14 |
| C7 | Quality / actuation | 0 | 3 | 9 | 12 |
| C8 | Quality / sensing | 0 | 0 | 12 | 12 |
| C9 | Quality / glue | 0 | 2 | 11 | 13 |
| C10 | Seam / teleop | 2 | 6 | 4 | 12 |
| C11 | Seam / failure | 5 | 5 | 2 | 12 |
| C12 | Spec drift | 4 | 11 | 6 | 21 |
| **Total** | | **28** | **72** | **68** | **168** |

Most-signal lenses: **safety/glue (C3)** and **spec drift (C12)** produced the most findings overall; **seam/failure (C11)** produced the highest P0 density (5/12 = 42% P0). Quality lenses (C7/C8/C9) found mostly P2 — the architecture itself is sound; the safety surfaces are where the bugs live.

Most-fragile module surfaces (by P0 count across cells):
- `main_robot.cpp` setup/control loop: 9 P0s
- `WebSocketCommandProducer.cpp` event handling: 5 P0s
- `OmniDrivetrain.h::stop()` reach: 5 P0s
- `BNO055IMU.h::read()` silent failure: 3 P0s
- `CommandFrameParser.h` input validation: 3 P0s

---

## 6. What this review did NOT cover

- **No on-hardware execution.** All findings are static analysis. Behaviors like "L298N brake at 1 kHz is effectively coast" (Action 10) are inferred, not measured.
- **No timing measurement.** Worst-case loop times, I2C contention, and TWDT margins are estimated from code shapes.
- **No fuzzing.** The parser was inspected for `strtof` edge cases but not run against adversarial inputs.
- **No WebSocket library internals review.** `arduinoWebSockets` 2.7.3 is treated as a black box; its heartbeat, fragmentation, and TCP buffering behavior are accepted at the documented contract.
- **No `ESP32Encoder` library review.** Treated as correct; quad counting and signed `int64_t` semantics are taken at face value.
- **No `Adafruit_BNO055` library review.** Bus error returns are inferred from API shape, not from library source.
- **No power / electrical review.** Brownout policy is discussed (Action 14) but no V-droop measurements, no battery model, no harness EMI review.
- **No stream-client / webapp review.** This review is scoped to body firmware only; the operator side (`code/stream-client/`, `code/webapp/`) was not inspected.
- **No PCB / harness review.** Pin floating during boot (C3-P0-01) is inferred from GPIO behavior, not from a schematic check.

Residual risk: any failure mode whose surface is in one of the above gaps. Most likely: real L298N brake behavior diverging from the code's intent.

---

## 7. Suggested next steps

1. **Triage Actions 1-10 (P0) into a ticket batch.** They're the document the on-call team should hold open until the next session.
2. **Land Action 1 (failsafe rework) as one PR.** This retires Theme A — fourteen findings — and is the single highest-leverage change.
3. **Land Action 2 + Action 23 together** (non-finite rejection + `BB8_RUNTIME_CHECK`). Closes Theme B and Theme E with one macro-level mechanism in place.
4. **Verify L298N brake on real hardware (Action 10)** before merging Action 1 to production. If brake is effectively coast, Action 1 alone does not solve the deceleration problem.
5. **Land Action 3 (boot reorder) as a second PR.** Closes Theme D; relies on Action 1 being in place so the always-on control task has a real stop to fall back to.
6. **Land Action 24 (seam tests) before any further controller work.** None of the safety changes above are currently regression-testable.
7. **Backfill Action 4 (wire protocol seq) after coordinating with stream-client.** Requires operator-side change.
8. **Re-run this review after Themes A, B, D land** to confirm retirement of cited findings and catch any new shape introduced by the refactor.
