# Safety/Correctness × Actuation — Cell 1

Scope: `src/motor/*`, `src/drivetrain/*`, `src/control/PID.h`, plus matching tests for gap-finding.

## P0 — block on-robot use

```json
{
  "id": "C1-P0-01",
  "title": "NaN/Inf BodyVelocity propagates to analogWrite as undefined uint8_t",
  "file": "src/motor/driver/L298NDriver.cpp",
  "line_range": "20-22",
  "quote": "BB8_ASSERT(value >= -1.0f && value <= 1.0f, \"setOutput value out of range [-1,1]\");\n\n    uint8_t pwm = static_cast<uint8_t>(std::abs(value) * 255.0f);",
  "risk": "BB8_ASSERT is a no-op outside `[env:native]` (debug.h:26). `CommandFrameParser` uses `std::strtof` with no finite/bounds check (CommandFrameParser.h:46-54), so a client sending `'nan,0,0'` or `'1e30,0,0'` produces a BodyVelocity that propagates through `OmniKinematics::toWheelRPMs` (NaN poisons all 3 RPMs; saturation scaling does not catch NaN because `maxAbs` stays 0), through `PID::compute` (assert is no-op), and into `setOutput`. `static_cast<uint8_t>(NaN * 255.0f)` is implementation-defined and typically yields 0 on ARM/Xtensa — but `static_cast<uint8_t>(1e30f * 255.0f)` is *undefined behavior* per C++ (out-of-range float-to-int conversion). On ESP32 the observed result is often 0 or 255; the latter is a runaway-motor hazard. The only thing standing between a malformed WS frame and a stuck full-duty H-bridge is undefined behavior.",
  "suggested_fix": "Add a runtime clamp in `Driver::setOutput` implementations (replace the BB8_ASSERT with `if (!std::isfinite(value)) value = 0.0f; value = std::clamp(value, -1.0f, 1.0f);`). Also reject non-finite frames at the parser boundary in `CommandFrameParser`.",
  "test_gap": "No test in test_l298n_driver.cpp/test_bts7960_driver.cpp/test_omni_kinematics.cpp feeds NaN/Inf into the API. Add `test_setOutput_nan_safe`, `test_setOutput_huge_value_safe`, and `test_toWheelRPMs_nan_input`."
}
```

```json
{
  "id": "C1-P0-02",
  "title": "Sub-threshold dt freezes saturated PID output indefinitely",
  "file": "src/control/PID.h",
  "line_range": "20-20",
  "quote": "if (dt < 1e-4f) return _lastOutput;",
  "risk": "If `dt < 100µs` is ever passed (e.g. caller drift, double-pump, or a future loop change), the PID returns the previous output verbatim. If `_lastOutput` was at `+1.0` saturation, the motor stays at full forward across an unbounded number of such ticks — there is no max-dwell or fallback. The control task's nominal dt is 10 ms (well above), but the staleness ramp does *not* protect this: a stale frame still drives a non-zero setpoint via the ramp until age >= 200 ms, so a back-to-back identical-tick scheduling glitch in vTaskDelayUntil combined with this guard can latch a saturated output. The `test_zero_dt_returns_last_output` test (test_pid.cpp:153-158) confirms this is intentional behavior, but the safety implication (latched saturation) isn't covered.",
  "suggested_fix": "Return 0 (not last_output) when dt is sub-threshold, OR keep last_output but clamp to a watchdog max-dwell (e.g. if the same dt-skip happens >N consecutive calls, reset to 0).",
  "test_gap": "test_pid.cpp:153 explicitly accepts the latched-output behavior; no test verifies behavior on N consecutive sub-threshold dt calls starting from a saturated state."
}
```

```json
{
  "id": "C1-P0-03",
  "title": "L298N brake (255/255) is left latched if anything calls brake() then setOutput() never executes",
  "file": "src/drivetrain/OmniDrivetrain.h",
  "line_range": "39-43",
  "quote": "void stop() override {\n        for (auto* m : _motors) m->brake();\n        for (auto* p : _pids) p->reset();\n        for (int i = 0; i < 3; i++) _targetRPMs[i] = 0.0f;\n    }",
  "risk": "`stop()` brakes the motors (both pins HIGH on L298N — a hard short between motor terminals, which is the correct fast-stop but high-current). It is *never called* by the runtime control loop (see CONTEXT §9.11): the staleness path drives `BodyVelocity{0,0,0}` through the *PID*, not through stop(). However, if any future code path calls `stop()` and the control loop continues running `update(dt)` after, `update()` immediately writes `setSpeed(s)` from the PID outputs and overrides brake — but if `update()` ever fails to run (e.g. control task hung but ws_prod task triggers stop via a future hook), the H-bridge is left shorted at 255/255 with no off-path. The combination of `brake = both-high short` + no `safe()`/coast primitive is dangerous as a default failsafe.",
  "suggested_fix": "Add a `Motor::coast()` / `Driver::coast()` primitive that writes 0/0 on L298N and Disable on BTS7960; have `OmniDrivetrain::stop()` call `coast()` not `brake()` unless an explicit hard-brake intent is signaled. Document brake semantics as 'hot short, do not leave latched'.",
  "test_gap": "No test verifies what happens after `stop()` if no further `update()` is invoked (post-stop quiescence). Test in test_omni_drivetrain.cpp:171 (`test_stop_then_update_motors_stay_zero`) requires `update()` to run *after* stop to verify zero output — masks the brake-latched-without-update hazard."
}
```

## P1 — should fix before next milestone

```json
{
  "id": "C1-P1-01",
  "title": "PID deadband only fires when setpoint ≈ 0 — asymmetric zero-snap, integrator builds at non-zero setpoints below deadband",
  "file": "src/control/PID.h",
  "line_range": "22-25",
  "quote": "if (_deadband > 0.0f && std::abs(setpoint) < 1e-6f && std::abs(measurement) < _deadband) {\n            reset();\n            return 0.0f;\n        }",
  "risk": "The deadband (`5.0f` RPM in main_robot.cpp:66-68) gates *only* when setpoint is essentially zero AND measurement is below the deadband. For a non-zero setpoint smaller than the deadband (e.g. `setpoint = 2 RPM`, measurement = 0), no deadband applies — the PID computes error = 2 RPM, kp·error = 0.01, integral grows. Because `getFilteredRPM()` smooths over 5 samples and the FIT0186 likely *cannot* move below ~5 RPM (static friction + driver dead zone — see test_l298n_driver.cpp:117-123 where 0.001 maps to PWM 0), the system has a permanent steady-state error that the integrator chases. Eventually `_integral` hits the anti-windup clamp at `outputMax/_ki = 1.0/0.002 = 500`, producing kp·error + ki·integral = small + 1.0 = clamped output `+1.0`. The motor *snaps* on the next time the setpoint exits the dead band downward, and reverses violently if the user toggles direction at low command. Symmetric for negative side. This is a known PID-with-stiction failure mode and a real teleop hazard.",
  "suggested_fix": "Apply the deadband symmetrically: if `|setpoint| < deadband`, force setpoint = 0 *and* freeze the integrator (do not accumulate while in deadband). Or: gate the integrator on `|error| > some_threshold`.",
  "test_gap": "No test in test_pid.cpp exercises the deadband at a non-zero sub-deadband setpoint. `test_zero_error_integral_stays` is the closest but only tests setpoint=0. Add `test_deadband_no_windup_at_small_setpoint`."
}
```

```json
{
  "id": "C1-P1-02",
  "title": "PID anti-windup divides by _ki without checking sign; flipped output range gives inverted clamp",
  "file": "src/control/PID.h",
  "line_range": "32-37",
  "quote": "if (_ki != 0.0f) {\n            float integralMax = _outputMax / _ki;\n            float integralMin = _outputMin / _ki;\n            if (_integral > integralMax) _integral = integralMax;\n            if (_integral < integralMin) _integral = integralMin;\n        }",
  "risk": "If a caller ever constructs a PID with negative `_ki` (e.g. a buggy refactor or a controller used in reverse), the integral bounds invert silently (integralMax becomes the lower bound). No assert protects against this. Same for `_outputMin > _outputMax`. This is also a textbook 'back-calculation anti-windup' approximation that ignores the kp and kd contribution: when kp·error or kd·derivative alone saturates the output, the integrator keeps growing because the clamp uses only `_ki`. With the production tunings (kp=0.005, ki=0.002, deadband=5) at large step inputs this windup is small but real.",
  "suggested_fix": "Constructor: `BB8_ASSERT(_outputMin <= _outputMax && _ki >= 0)`. Switch to a clamping anti-windup on the *output* (only accumulate `_integral += error*dt` if doing so wouldn't push output past clamps).",
  "test_gap": "test_anti_windup (test_pid.cpp:68) only tests positive-ki, P=0 case. No test with kp>0 and ki>0 simultaneously saturating; no test with negative ki."
}
```

```json
{
  "id": "C1-P1-03",
  "title": "OmniKinematics saturation preserves direction, but reads of maxRPM as motor-side cap conflate motor no-load with motor PID limit",
  "file": "src/drivetrain/OmniKinematics.h",
  "line_range": "45-50",
  "quote": "if (maxAbs > _config.maxRPM) {\n            float factor = _config.maxRPM / maxAbs;\n            for (int i = 0; i < 3; i++) {\n                rpms[i] *= factor;\n            }\n        }",
  "risk": "Direction preservation under saturation is correct (the central safety property — confirmed). However, `_config.maxRPM` is set to FIT0186 no-load RPM (251) — the *physical* no-load speed. The PID setpoint can therefore demand 251 RPM, but at that target the motor is at zero torque margin — any disturbance produces an unrecoverable lag and the PID drives output to +1.0 indefinitely. The saturation cap should be the *controllable* RPM, not the no-load RPM. Also note: saturation is checked *only* in `toWheelRPMs`; if `_targetRPMs` is ever set from outside that path, no clamp.",
  "suggested_fix": "Reduce `MAX_RPM` to ~80% of no-load (200 RPM) and rename to `maxControllableRPM`. Add a clamp at the `_targetRPMs` write site in `OmniDrivetrain::drive()`.",
  "test_gap": "test_saturation_scaling (test_omni_kinematics.cpp:72) verifies the scaling preserves direction at exactly the cap, but does not stress at >>cap or check the ratio between saturated and unsaturated wheels under combined vx+vy+omega."
}
```

```json
{
  "id": "C1-P1-04",
  "title": "L298N vs BTS7960 divergence at zero: coast-vs-disable + brake-vs-stop, plus implicit Enable/Disable cycling",
  "file": "src/motor/driver/BTS7960Driver.cpp",
  "line_range": "20-30",
  "quote": "if (value == 0.0f) {\n        _hbridge.Disable();\n        return;\n    }\n\n    _hbridge.Enable();\n    uint8_t pwm = static_cast<uint8_t>(std::abs(value) * 255.0f);\n    if (pwm == 0) {\n        _hbridge.Disable();\n        return;\n    }",
  "risk": "L298N coast (setOutput(0)) writes 0/0 to PWM pins — instant freewheel, no h-bridge state change. BTS7960 calls `_hbridge.Disable()` which drops the EN lines low — also coast, but with a high-Z transient as EN settles. Worse: every nonzero call re-enters `_hbridge.Enable()` (line 25). Toggling Enable at 100 Hz wears the MOSFETs' gate driver and produces audible click + small current spike. The `setOutput(0.0f)` check uses exact float equality (`value == 0.0f`) — a value like `1e-9f` skips the disable branch, enters Enable, then computes `pwm = 0` and re-Disables. The redundant Enable+Disable at every near-zero call is a real concern when re-enabling BTS7960.",
  "suggested_fix": "(a) Cache state in the driver and only call Enable/Disable on transition. (b) Replace `value == 0.0f` with `std::abs(value) < 1.0f/256.0f`. (c) Document brake semantics divergence as part of the `Driver` interface contract.",
  "test_gap": "test_bts7960_driver.cpp tests do not measure call count of Enable/Disable across repeated nonzero+zero calls. `test_dead_zone` confirms the bug exists (0.001 -> Disable) but treats it as expected behavior."
}
```

```json
{
  "id": "C1-P1-05",
  "title": "FIT0186Motor encoder dt uses unsigned `micros()` subtraction — wrap-safe, but cast through float loses precision near 70-min wrap",
  "file": "src/motor/FIT0186Motor.h",
  "line_range": "37-41",
  "quote": "unsigned long now = micros();\n        float dtSeconds = static_cast<float>(now - _lastUpdateMicros) / 1'000'000.0f;\n\n        if (dtSeconds <= 0.0f) return;",
  "risk": "Unsigned subtraction is wrap-safe (test_micros_overflow verifies this at ULONG_MAX). However: when `now - _lastUpdateMicros` is converted to float, precision is fine up to ~16.7M (24-bit mantissa) — i.e. up to 16.7 seconds of skipped updates before float dt loses microsecond resolution. Normal 10 ms operation is far below. But if the control task ever stalls (TWDT timeout = 1 s, panic resets), there's no scenario where dt grows large enough to matter. The bigger latent issue is the `dtSeconds <= 0.0f` guard: with unsigned subtraction, the only way dt is ≤ 0 in float is if `now == _lastUpdateMicros` exactly. If a future patch makes micros() unreliable (e.g. ROM patch), or if `_lastUpdateMicros` is initialized to a nonzero value (which is `micros()` in `begin()`) but the encoder's `_lastCount` is set right after, there's a sub-µs race window where deltaCount is computed against a stale `_lastCount`. Currently OK; flagging because the test_micros_overflow test only covers ULONG_MAX-near boundary.",
  "suggested_fix": "Switch to `int64_t` dt math (microsecond integer counts) and divide only at the end. Initialize `_lastCount` *before* `_lastUpdateMicros` in begin() to guarantee no stale-tick race.",
  "test_gap": "test_fit0186_motor.cpp:75 (`test_micros_overflow`) covers the wrap. No test exercises a very-long-skip dt that exceeds float precision."
}
```

```json
{
  "id": "C1-P1-06",
  "title": "MovingAverage<5> survives motor stop with stale samples — first PID tick after re-drive sees lagged measurement",
  "file": "src/motor/FIT0186Motor.h",
  "line_range": "49-51",
  "quote": "_filter.push(_rawRPM);\n        _lastCount = count;\n        _lastUpdateMicros = now;",
  "risk": "After `OmniDrivetrain::stop()` calls `motor->brake()` and `_pids[i]->reset()`, the PID's integral and prevMeasurement are cleared (PID.h:55-60), but `MovingAverage<5> _filter` is NOT reset (FIT0186Motor exposes no reset hook). Consequently, when `update()` resumes after a stop, `getFilteredRPM()` returns the average of 4 stale + 1 fresh samples for the first 5 ticks — a 50 ms lag during which the PID error is wrong. At 5 ms tick this is 5 ticks of bad measurement. Worst case: motor was spinning at +100 RPM, stop, restart at target -100 RPM. First fresh sample is ~0 RPM. Filtered avg shows +80 RPM, PID sees error = -100 - 80 = -180 RPM, drives output to -1.0 (clamped). The current code only calls `stop()` indirectly, but if added in a future failsafe path this is a launch hazard.",
  "suggested_fix": "Add `Motor::reset()` / `FIT0186Motor::resetFilter()` and call it from `OmniDrivetrain::stop()` alongside `pid->reset()`.",
  "test_gap": "No test verifies filter state across a stop-then-drive sequence. `test_filter_convergence_on_reversal` (test_fit0186_motor.cpp:122) shows the filter takes 5 ticks to converge — confirms the hazard exists."
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
  "test_gap": "test_omni_kinematics.cpp has no test for out-of-range tilt; `test_tilt_angle_effect` only covers 0 and 30°."
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
  "test_gap": "No NaN test in test_omni_drivetrain.cpp or test_omni_kinematics.cpp."
}
```

## P2 — nice to have / future

```json
{
  "id": "C1-P2-01",
  "title": "L298N PWM range conversion uses static_cast<uint8_t> truncation, not rounding",
  "file": "src/motor/driver/L298NDriver.cpp",
  "line_range": "22-22",
  "quote": "uint8_t pwm = static_cast<uint8_t>(std::abs(value) * 255.0f);",
  "risk": "Truncation: setOutput(1.0) → 255, setOutput(0.999) → 254 (intended). But setOutput(-1.0) → abs = 1.0 → 255 (OK); setOutput close to 0 truncates to 0 silently — combined with `if (pwm == 0)` short-circuits at value < ~1/255 ≈ 0.0039 to coast. This is a 0.4% deadband on the PWM side that the PID doesn't know about — minor coupling.",
  "suggested_fix": "Round (`+ 0.5f` before truncation). Or document the 1/256 PWM-side deadband and incorporate into PID deadband."
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
  "suggested_fix": "Move `_scale = 1.0f / (config.wheelRadius * cosf(config.tiltAngle));` to a member set in the constructor."
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
  "suggested_fix": "`friend class FIT0186MotorTest;` instead, or expose a `setEncoderCountForTest()` under `#ifdef BB8_DEBUG`."
}
```

```json
{
  "id": "C1-P2-04",
  "title": "OmniKinematics has no `forward()` (forward-kinematics) function — review item is moot",
  "file": "src/drivetrain/OmniKinematics.h",
  "line_range": "9-18",
  "quote": "class OmniKinematics {\npublic:\n    explicit OmniKinematics(const DrivetrainConfig& config) : _config(config) {",
  "risk": "The review prompt asked to verify `forward()` and `inverse()`. The class exposes only `toWheelRPMs` (the inverse map BodyVelocity→wheel RPMs). No forward map exists. Not currently a bug, but documented for completeness — if odometry is added later, the forward map must agree with the matrix in `docs/omni-kinematics-derivation.md` §5."
}
```

## Observations (not findings) — prose, no JSON

- Kinematics matrix is correct against `docs/omni-kinematics-derivation.md` §5. Code at OmniKinematics.h:32-34 expands to `w0 = scale*(-vy - R·ω)`, `w1 = scale*(-0.866·vx + 0.5·vy - R·ω)`, `w2 = scale*(+0.866·vx + 0.5·vy - R·ω)`, which matches the doc exactly.
- Saturation in `toWheelRPMs` correctly preserves direction (scales all 3 by a common factor) — this is the safety-critical property for teleop and it is implemented right (OmniKinematics.h:45-50).
- L298N direction sign is consistent with `setOutput([-1,1])` semantics: positive → fwd pin PWM, rev pin 0; negative → fwd 0, rev PWM (L298NDriver.cpp:30-36). BTS7960 maps positive → TurnLeft, negative → TurnRight (BTS7960Driver.cpp:31-35). Whether `TurnLeft` matches the physical "forward" rotation of motor 0 is a *wiring* concern, not a code concern.
- PID derivative-on-measurement is correctly implemented (PID.h:41) — no spike on setpoint change. Test `test_derivative_on_measurement_no_spike_on_setpoint_change` confirms.
- ESP32Encoder's `getCount()` returns `int64_t` per the library; multi-day operation will not overflow even at max RPM × full quadrature × 86400s.
- The control task's nominal dt (10 ms) is passed straight into PID, while FIT0186Motor's internal dt is from `micros()`. Independent sources, both correct at steady state — observed in CONTEXT §9.6.
- The PID `_lastOutput` is initialized to 0 (PID.h:13), so the sub-threshold-dt freeze at boot returns 0 — C1-P0-02 is only a hazard *after* the PID has saturated at least once.
- Test gap pattern across all suites: nothing tests **adversarial input** (NaN, Inf, huge values, malformed frames reaching kinematics/PID). All tests exercise nominal & boundary, no security/adversarial inputs.
