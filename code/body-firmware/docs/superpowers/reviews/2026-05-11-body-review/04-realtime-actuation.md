# Cell 4 — Real-time / Concurrency lens, Actuation scope

**Anchor:** f8ca95f
**Scope:** `src/motor/**`, `src/drivetrain/**`, `src/control/PID.h`
**Lens:** real-time deadlines, ISR safety, jitter, blocking, init ordering

```json
{
  "summary": {
    "p0": 3,
    "p1": 6,
    "p2": 4,
    "headline": "PWM at 1 kHz with 10 ms PID dt only delivers ~10 carrier cycles per update and an 8-bit DAC step ~25 RPM at low duty — the controller can chatter at the carrier rate; PID dt is hardcoded to 0.010s while the encoder uses real micros() dt, so under jitter the PID I/D terms drift from the actuator; encoder count is correctly _ENTER_CRITICAL-guarded by the library, but the FIT0186Motor::update() read-modify-write of _lastCount is not atomic with respect to a second caller (currently single-core, but the abstraction does not enforce it)."
  }
}
```

---

## P0 findings

```json
{
  "id": "C4-P0-01",
  "severity": "P0",
  "title": "PID dt is constant 0.010s, decoupled from real loop period — under jitter the integrator and derivative term lie",
  "file": "src/main_robot.cpp",
  "lines": "190-191",
  "quote": "const float dt = static_cast<float>(CONTROL_PERIOD_MS) / 1000.0f;\n        g_drivetrain.update(dt);",
  "why_p0": "PID dt is hardcoded to 10 ms regardless of the actual interval between calls. vTaskDelayUntil holds *average* phase but does not prevent individual ticks from slipping when WiFi/I2C bursts or the BNO055 read (4 I2C transactions on the hot path) preempts the scheduler. The encoder, meanwhile, uses real micros() inside FIT0186Motor::update() — so getFilteredRPM is computed against true elapsed time but fed into a PID that thinks dt is exactly 10 ms. If a tick lands at 14 ms because the prior tick was 6 ms, the encoder reports an inflated RPM (delta/0.014) which the PID compares against a setpoint integrated as if dt=0.010. Net: I term winds in the wrong direction proportional to jitter, D term scales inversely. With Ki=0.002 and an output clamp of ±1, the integrator saturates at ±500 RPM·s of accumulated error which is reachable in seconds of mismatched ticks. Hardware-damage risk on a tilted-omni: jitter-driven output spikes can flip wheel direction at full PWM.",
  "fix_sketch": "Pass dt from vTaskGetTickCount() delta (or just micros()) into both motor->update() *and* PID. Or: derive control dt the same way FIT0186Motor::update() does (micros() diff) and use a single dt for both. Reject ticks where |dt - nominal| > 50%."
}
```

```json
{
  "id": "C4-P0-02",
  "severity": "P0",
  "title": "1 kHz LEDC PWM × 100 Hz PID = only ~10 carrier cycles per update; 8-bit resolution at low duty quantizes to ~25 RPM steps",
  "file": "src/motor/driver/L298NDriver.cpp",
  "lines": "22-36",
  "quote": "uint8_t pwm = static_cast<uint8_t>(std::abs(value) * 255.0f);\n\n    if (pwm == 0) {\n        analogWrite(_pins.fwd, 0);\n        analogWrite(_pins.rev, 0);\n        return;\n    }",
  "why_p0": "Arduino-ESP32 analogWrite default is 1 kHz at 8-bit (verified in framework-arduinoespressif32 esp32-hal-ledc.c: `static int analog_frequency = 1000; static uint8_t analog_resolution = 8;`). The PID runs at 100 Hz, so each PID output drives the L298N for exactly 10 PWM cycles. With Kp=0.005 (PID output is in normalized [-1,1] units, mapped to 0-255), one PWM count = 1/255 = 0.0039 of full scale ≈ 1.0 RPM-equivalent at no-load (251 RPM). But Kp×error of 0.005 × 50RPM = 0.25 → 64 PWM counts; below 0.004 of output (1 PWM count) the PID has *no actuator authority*. Combined with the 5-RPM deadband (PID.h-style snap), there is a dead zone of ~1-25 RPM at low setpoints where the PID either commands zero or jumps to first-quantization-step output. On BTS7960 (faster slew, inductive load) 1 kHz audible whine is a known issue and the IC datasheet recommends >5 kHz to keep dissipation in the SOA region. With small motors (FIT0186, 6V at <1A) and 1 kHz, the motors may not spin below ~30% duty due to friction/back-EMF — exactly the regime PID converges to at low setpoints. This manifests as motors that stall on small commands and chatter on large ones.",
  "fix_sketch": "Call `analogWriteFrequency(20000)` (20 kHz, above audible) and `analogWriteResolution(10)` (1024 steps) in L298NDriver::begin(). For BTS7960 specifically, follow the IR datasheet (>1 kHz minimum, 20–25 kHz typical). Verify with a scope and measure motor I_avg at low duty. Then revisit PID gains."
}
```

```json
{
  "id": "C4-P0-03",
  "severity": "P0",
  "title": "OmniDrivetrain::stop() is unreachable from any runtime path — failsafe never engages active braking; staleness ramp only sets setpoint to zero",
  "file": "src/main_robot.cpp",
  "lines": "173-191",
  "quote": "BodyVelocity drive_cmd{0.0f, 0.0f, 0.0f};\n        if (have_seen_fresh) {\n            const uint32_t age = now - last_fresh_ms;\n            if (age < STALENESS_TIMEOUT_MS) {\n                const float scale = 1.0f - static_cast<float>(age) /\n                                            static_cast<float>(STALENESS_TIMEOUT_MS);",
  "why_p0": "On WS disconnect or staleness, drive_cmd becomes {0,0,0} and is run through PassthroughDrivetrainController::update → OmniDrivetrain::drive → kinematics → setpoint=0. The PID then computes an output to bring measured RPM to 0; if the wheel is spinning (rolling downhill, kicked), the PID *commands negative output* (actively driving backward) until error closes, then overshoots into the deadband. This is the *opposite* of safe failsafe behavior: a runaway robot will fight the operator's last hope (reconnecting and pressing stop) because the PID is still active. OmniDrivetrain::stop() (which calls Motor::brake() — on L298N, both pins HIGH = short = active brake) is the correct failsafe, but is wired to PassthroughDrivetrainController::stop() which is never invoked. The control loop has no call site for either. Real-time impact: on cliff/bump/spin-up scenarios the robot stays under PID control with zero setpoint, not stopped.",
  "fix_sketch": "When age > STALENESS_TIMEOUT_MS *and* the motors still have non-zero filtered RPM (or just unconditionally after a hold-down period), call g_drivetrain.stop() once. Track an internal 'stopped' state to avoid spamming brake() at 100 Hz. Also invoke g_controller->stop() on OTA start, on ws disconnect callback (Core 0), and on TWDT pre-panic if reachable."
}
```

---

## P1 findings

```json
{
  "id": "C4-P1-04",
  "severity": "P1",
  "title": "FIT0186Motor::update() read-modify-write on _lastCount is not atomic with respect to multiple callers — abstraction does not enforce single-threaded use",
  "file": "src/motor/FIT0186Motor.h",
  "lines": "37-52",
  "quote": "int64_t count = _encoder.getCount();\n        int64_t deltaCount = count - _lastCount;\n\n        _rawRPM = (static_cast<float>(deltaCount) / static_cast<float>(_encoderCPR))\n                  * (60.0f / dtSeconds);\n\n        _filter.push(_rawRPM);\n        _lastCount = count;\n        _lastUpdateMicros = now;",
  "why_p1": "ESP32Encoder::getCount() is itself ISR-safe — the library wraps the read in `_ENTER_CRITICAL()` (verified in ESP32Encoder.cpp line 309-314) and uses pcnt hardware so torn reads on the 64-bit count via the hardware overflow accumulator are correctly compensated. But FIT0186Motor::update() then performs an unguarded read-modify-write on _lastCount, _lastUpdateMicros, _rawRPM, and _filter. The current code calls update() only from the control task (Core 1), so this is safe today. But the Motor abstraction (virtual void update()) does not document the threading contract, and a future addition (e.g. a telemetry task that calls getRPM/getFilteredRPM cross-core) would silently corrupt the moving average and produce torn float reads on _rawRPM. _filter.push is also called from update() without synchronization — if average() is called from another core, the read of _count vs _buffer is racy.",
  "fix_sketch": "Document in Motor.h that update()/getRPM()/getFilteredRPM() are caller-pinned (i.e. must be called from one task). Or add a portMUX_TYPE inside FIT0186Motor for the count delta region and atomic<float> _rawRPM. Cheapest fix: comment + a runtime check in update() (xTaskGetCurrentTaskHandle vs cached)."
}
```

```json
{
  "id": "C4-P1-05",
  "severity": "P1",
  "title": "Two independently sourced dts — encoder uses micros() inside update(), PID uses caller-supplied dt — under jitter they diverge",
  "file": "src/motor/FIT0186Motor.h",
  "lines": "38-47",
  "quote": "unsigned long now = micros();\n        float dtSeconds = static_cast<float>(now - _lastUpdateMicros) / 1'000'000.0f;\n\n        if (dtSeconds <= 0.0f) return;\n\n        int64_t count = _encoder.getCount();\n        int64_t deltaCount = count - _lastCount;\n\n        _rawRPM = (static_cast<float>(deltaCount) / static_cast<float>(_encoderCPR))\n                  * (60.0f / dtSeconds);",
  "why_p1": "Context §9 item 6 acknowledges this. The encoder measures real elapsed time (good), but the PID sees the hardcoded 0.010 s (see C4-P0-01). The MovingAverage<5> then smooths *RPM-already-scaled-by-jittered-dt*, so a tick at 6 ms followed by 14 ms produces two raw RPM samples that average correctly *only because the dts roughly balance*. If the slow tick is isolated (e.g. one I2C stall), the average is biased high for the next 50 ms (5-tap window @ 100 Hz). PID sees this biased measurement against a setpoint computed from a non-jittered timeline.",
  "fix_sketch": "Pass a single dt down: `controlTask` computes `dt = (now - last_tick) / 1e6f`, passes it to motor->update(dt) and pid->compute(.., .., dt). Remove the micros() call inside FIT0186Motor."
}
```

```json
{
  "id": "C4-P1-06",
  "severity": "P1",
  "title": "PID derivative-on-measurement responds to staleness ramp as a real velocity signal — phantom D kick on every silence event",
  "file": "src/control/PID.h",
  "lines": "39-45",
  "quote": "float derivative = 0.0f;\n        if (!_firstCompute) {\n            derivative = -(measurement - _prevMeasurement) / dt;\n        }",
  "why_p1": "The PID measures filtered RPM, not setpoint, so derivative kick on setpoint changes is correctly avoided. But the staleness ramp in main_robot.cpp drives the *setpoint* from `last_known * 1.0` to `last_known * 0.0` linearly over 200 ms. As setpoint falls below measurement, error flips sign, PID commands brake. The motor decelerates, measurement falls, and `-(measurement - prev)/dt` produces a *negative* derivative which (with Kd=0.0005 and dt=0.010) adds to the output. With Kd=0.0005 and a 50 RPM/tick deceleration, D contribution is 0.0005 × 5000 = 2.5 — clamped to 1.0, i.e. saturated. Combined with C4-P0-02 (1 kHz PWM), each tick during the ramp the motor sees full-reverse PWM for 10 ms, then near-zero. Audible motor groan and high current spikes on the L298N.",
  "fix_sketch": "Filter the derivative (low-pass on D term — common α≈0.1). Or limit Kd contribution per tick to a fraction of output range. Or detect setpoint changes > threshold and call PID::reset()."
}
```

```json
{
  "id": "C4-P1-07",
  "severity": "P1",
  "title": "PID integral anti-windup clamp uses outputMax/Ki, but compute() then clamps output again — wind-down asymmetry",
  "file": "src/control/PID.h",
  "lines": "31-48",
  "quote": "if (_ki != 0.0f) {\n            float integralMax = _outputMax / _ki;\n            float integralMin = _outputMin / _ki;\n            if (_integral > integralMax) _integral = integralMax;\n            if (_integral < integralMin) _integral = integralMin;\n        }",
  "why_p1": "The integral clamp assumes the I term alone fills the output range. Then output = Kp*e + Ki*I + Kd*d is clamped again. Two issues: (a) when Kp*e is large and positive, integralMax permits I=+1 even though combined output is already saturated → integral keeps accumulating (since the clamp is on the integral *state*, not on `output > outputMax` post-sum). Wait — re-reading: the clamp is on `_integral` BEFORE accumulating any further, but the actual back-calculation against saturation is missing. So if Kp*e drives output to +1 (saturated) and the PID then clamps output to +1, on the next tick the integral has *already* been incremented by error*dt, *then* clamped to +500 (1/0.002), but at the start of the next tick it isn't clamped further if error stays positive. Real anti-windup needs `back-calculation`: integral -= (output_pre_clamp - output_clamped) / Ki. The current scheme is 'integral state clamping' which is weak. With Ki=0.002 and a sustained 100 RPM error for 5 s, integral hits 500 in 2.5 s — fine, it stops growing — but it does *not unwind* when error flips. Recovery from a slow-down trace is sluggish.",
  "fix_sketch": "Use back-calculation: after final output clamp, do `_integral -= (output_pre_clamp - output_clamped) / _ki`. Verify _ki != 0 first."
}
```

```json
{
  "id": "C4-P1-08",
  "severity": "P1",
  "title": "Encoder pins 34/35/36/39 are input-only (no internal pull-ups) — encoder relies on external open-collector pull-ups or active hot signals",
  "file": "src/config/pins.h",
  "lines": "16-18",
  "quote": "inline constexpr EncoderPins MOTOR0_ENCODER_PINS = {.a = 34, .b = 35};\n\ninline constexpr EncoderPins MOTOR1_ENCODER_PINS = {.a = 2, .b = 4};\ninline constexpr EncoderPins MOTOR2_ENCODER_PINS = {.a = 36, .b = 39};",
  "why_p1": "GPIO 34, 35, 36, 39 on the ESP32 are input-only (sensor pins) — no internal pull-up/down hardware. ESP32Encoder's PCNT peripheral can still count edges, but the line must be driven on both sides or have external pull-ups. FIT0186 hall encoders are open-drain on many variants; without external pulls the line floats and PCNT will mis-count on EMI noise from the L298N PWM switching at 1 kHz on adjacent pins. Combined with the 1 kHz PWM (carrier well within the encoder edge bandwidth), capacitive coupling between motor PWM traces and encoder traces could inject false counts. Real-time impact: PID measurement noise → loud chatter, possible runaway if false count rate exceeds true rate.",
  "fix_sketch": "Verify hardware has external 4.7k–10k pull-ups on each encoder line to 3V3. If not, move motor1 (currently on 2/4) or accept the constraint. Also confirm encoder traces are routed away from PWM traces."
}
```

```json
{
  "id": "C4-P1-09",
  "severity": "P1",
  "title": "L298NDriver::brake() (both pins HIGH = 255/255) is functionally equivalent to coast at 1 kHz on a real L298N — H-bridge sees 100% duty on both sides, which is short-through on some L298N clones",
  "file": "src/motor/driver/L298NDriver.cpp",
  "lines": "39-42",
  "quote": "void L298NDriver::brake() {\n    analogWrite(_pins.fwd, 255);\n    analogWrite(_pins.rev, 255);\n}",
  "why_p1": "L298N brake semantics require BOTH input pins set to the same level with the EN pin high (typically pulled high in this circuit). Writing 255/255 via analogWrite drives both PWM outputs high at 100% duty *but with 1 kHz pulse width modulation* — on a true LEDC peripheral, 255/255 should produce a clean DC high, but the PWM 'helper' in Arduino-ESP32 can briefly drop to zero between PWM cycles (LEDC reload), giving inconsistent brake behavior. Worse, on the L298N IC, both IN1 and IN2 high with EN high *is* a brake-low (or brake-high depending on the part) — but with a PWM signal, the H-bridge sees square waves and may oscillate between brake and freewheel. On the BTS7960, _hbridge.Stop() is the correct primitive — see asymmetric definitions in context §9 item 2.",
  "fix_sketch": "Use digitalWrite(fwd, HIGH); digitalWrite(rev, HIGH); for active brake — bypasses LEDC. Document that 'coast' (0/0) is the safer no-op for a sensorless e-stop."
}
```

---

## P2 findings

```json
{
  "id": "C4-P2-10",
  "severity": "P2",
  "title": "OmniKinematics recomputes cosf(tiltAngle) on every drive() call",
  "file": "src/drivetrain/OmniKinematics.h",
  "lines": "27-28",
  "quote": "float cosAlpha = cosf(_config.tiltAngle);\n        float scale = 1.0f / (_config.wheelRadius * cosAlpha);",
  "why_p2": "cosf is single-cycle on Xtensa with -O2 but the divide is ~30 cycles. drive() is called every tick (100 Hz). Saving ~40 cycles × 100 Hz × 3 motors = 12k cycles/s — negligible but free if cached in the constructor."
}
```

```json
{
  "id": "C4-P2-11",
  "severity": "P2",
  "title": "MovingAverage average() recomputes sum every call instead of running sum",
  "file": "src/util/MovingAverage.h",
  "lines": "22-29",
  "quote": "float average() const {\n        if (_count == 0) return 0.0f;\n        float sum = 0.0f;\n        for (size_t i = 0; i < _count; ++i) {\n            sum += _buffer[i];\n        }\n        return sum / static_cast<float>(_count);\n    }",
  "why_p2": "5-tap window, called once per PID compute per motor = 3 calls/tick @ 100 Hz × 5 adds = 1500 fp adds/s. Trivial. But a running sum (sum += new - oldest) inside push() avoids the loop and gives O(1) average. Worth doing if window size grows."
}
```

```json
{
  "id": "C4-P2-12",
  "severity": "P2",
  "title": "Static-storage construction of L298NDriver/FIT0186Motor relies on global init order — same TU only",
  "file": "src/main_robot.cpp",
  "lines": "58-70",
  "quote": "L298NDriver g_driver0(MOTOR0_PINS);\nL298NDriver g_driver1(MOTOR1_PINS);\nL298NDriver g_driver2(MOTOR2_PINS);\n\nFIT0186Motor<5> g_motor0(g_driver0, MOTOR0_ENCODER);\nFIT0186Motor<5> g_motor1(g_driver1, MOTOR1_ENCODER);\nFIT0186Motor<5> g_motor2(g_driver2, MOTOR2_ENCODER);",
  "why_p2": "All globals are in one TU (main_robot.cpp) so within-TU declaration order applies — drivers first, motors second. Safe today. If a future commit moves L298NDriver instances to another TU, the static init fiasco re-emerges (FIT0186Motor stores Driver&). Comment recommended; or move construction into setup() like g_latch/g_producer/g_controller already do."
}
```

```json
{
  "id": "C4-P2-13",
  "severity": "P2",
  "title": "OmniDrivetrain::update() does not check that motors are bound — _motors[i] dereference if construction half-failed",
  "file": "src/drivetrain/OmniDrivetrain.h",
  "lines": "25-32",
  "quote": "void update(float dt) override {\n        for (auto* m : _motors) m->update();",
  "why_p2": "Drivetrain.h base stores Motor* _motors[3] from references in the constructor — references are never null, so the pointers are non-null by construction. Safe today. If anyone ever adds a default constructor or zero-fill, this becomes a crash. No fix needed unless API expands."
}
```

---

## Lens questions — direct answers

1. **PWM freq vs control loop:** Insufficient. 1 kHz × 100 Hz = 10 PWM cycles per PID update. See C4-P0-02. L298N tolerates 1 kHz electrically but motor performance suffers (audible, lossy, deadband ≥ ~30% duty on small DC motors).
2. **Encoder ISR safety:** `ESP32Encoder::getCount()` itself is `_ENTER_CRITICAL`-guarded (lines 309-314 of library). 64-bit count is safely composed from a 32-bit accumulator + 16-bit hardware register inside the critical section. FIT0186Motor's read-modify-write on `_lastCount` is *not* guarded but is currently single-threaded — see C4-P1-04.
3. **dt in PID:** Hardcoded `CONTROL_PERIOD_MS/1000.0f` in main_robot.cpp:190. Encoder uses `micros()` independently. Mismatch under jitter — see C4-P0-01 and C4-P1-05.
4. **MovingAverage state:** Single-core today (control task only). Not safe cross-core if anyone adds a telemetry consumer. See C4-P1-04.
5. **Loop slip from 100→80 Hz failure mode:** PID integral biases (C4-P0-01), D term spikes (C4-P1-06), motor driver holds last PWM (LEDC continues at last duty even if controller misses ticks — no built-in failsafe in the L298N driver). TWDT (1 s) catches a full hang but not a 20% throughput loss.
6. **stop() reachability:** Unreachable from runtime. See C4-P0-03. Should be called from staleness expiry (hard zero) and ws disconnect handler. The WS event callback runs on Core 0; calling g_drivetrain.stop() from there crosses the unprotected motor abstraction (C4-P1-04). Safest: set a `volatile bool g_e_stop_requested` from Core 0 callbacks, check in control task.
7. **Blocking calls inside Motor/Driver:** No `delay()` in motor/driver paths. `analogWrite` is non-blocking (LEDC hardware). `ESP32Encoder::getCount` enters a critical section briefly (~µs). No Serial.print in motor hot paths. BNO055IMU.begin/restoreCalibration has `delay(20)` calls — only at setup, not at runtime. The control task's `g_imu.read()` does 4 blocking I2C transactions per tick (~2-5 ms total at 100 kHz I2C) which can starve the 10 ms budget under contention — observed in context §9 item 10.
8. **Construction / init order:** All motor/driver globals in one TU → safe (C4-P2-12). FreeRTOS-dependent objects (latch, producer, controller) correctly moved to setup() after scheduler is running. No null-deref risk in current layout.

---

**P0 summary (3 lines):**
1. PID dt is hardcoded 0.010s while encoder uses real micros() — jitter desynchronizes the controller from the actuator and winds integral in the wrong direction (C4-P0-01).
2. 1 kHz LEDC PWM × 100 Hz PID gives only ~10 carrier cycles per update with 8-bit resolution → quantization deadband, audible chatter, motors stall at low duty (C4-P0-02).
3. OmniDrivetrain::stop() is unreachable from any runtime path — staleness ramp leaves the PID active with setpoint=0, not an active brake, so a kicked/rolled robot is not arrested (C4-P0-03).
