# Cell 12 — Spec Drift Review

**Anchor:** f8ca95f
**Scope:** `code/body-firmware/docs/` vs `code/body-firmware/src/`
**Lens:** drift between design docs and implementation (constants, behavior, assumptions, sign/convention, units, untested invariants).

Each finding is JSON-in-markdown. IDs `C12-{P0|P1|P2}-NN`. Quotes are verbatim and ≥8 chars.

---

## P0 — Safety-critical drift

### C12-P0-01: ENCODER_CPR in code (2800) is 4× the spec value (700)

```json
{
  "id": "C12-P0-01",
  "title": "fit0186::ENCODER_CPR disagrees with motor design spec (2800 vs 700)",
  "spec_file": "docs/superpowers/specs/2026-04-23-fit0186-motor-design.md",
  "spec_line_range": "176-179",
  "spec_quote": "inline constexpr uint16_t ENCODER_CPR = 700;  // counts per revolution at gearbox output",
  "file": "src/motor/constants/FIT0186.h",
  "line_range": "11-11",
  "quote": "inline constexpr uint16_t ENCODER_CPR = 2800;",
  "risk": "RPM = (deltaCount / encoderCPR) * 60/dt. If CPR is 4× the true value, computed RPM is 4× too low; if CPR is too low, computed RPM is too high. PID measurement is in RPM-space, so the loop is mistuned by a factor of 4. Saturation in OmniKinematics (capped at MAX_RPM=251) will allow demanded RPMs the PID never actually reaches (or vice versa). The hardware reference in the same spec also says '16 CPR motor shaft / 700 CPR gearbox shaft' — consistent with 700, not 2800. Likely the code value (2800 = 700 * 4) is correct for x4-decoded full-quadrature (ESP32Encoder.attachFullQuad), but the spec was never updated to reflect that decoding mode. This is on the hot path of the feedback loop, so P0.",
  "suggested_fix": "Update the spec to clarify that 700 is the gearbox CPR in single-edge counting mode and 2800 = 700×4 is what the FIT0186Motor sees through attachFullQuad. Add a comment in FIT0186.h referencing the derivation."
}
```

### C12-P0-02: setSpeed(0) coast semantics conflict between Motor spec and L298NDriver behavior on brake()

```json
{
  "id": "C12-P0-02",
  "title": "Motor spec says brake() shorts terminals via BTS7960; L298N brake() also writes 255/255 (not documented as supported)",
  "spec_file": "docs/superpowers/specs/2026-04-23-fit0186-motor-design.md",
  "spec_line_range": "104-106",
  "spec_quote": "setSpeed(0) — coast (both H-bridge outputs LOW, motor disconnected)\n- brake() — active braking (both H-bridge outputs HIGH, terminals shorted)",
  "file": "src/motor/driver/L298NDriver.cpp",
  "line_range": "39-42",
  "quote": "void L298NDriver::brake() {\n    analogWrite(_pins.fwd, 255);\n    analogWrite(_pins.rev, 255);\n}",
  "risk": "The motor-design spec is written exclusively against the BTS7960 — the Driver/L298N abstraction does not appear in the spec. The runtime build (`[env:robot]`) excludes BTS7960Driver.cpp and uses L298NDriver instead (main_robot.cpp:58-60). The L298N brake (both pins HIGH = output enable on both half-bridges) is documented in the driver as 'short motor terminals', matching the BTS7960 brake intent, but the spec doesn't describe L298N at all. Risk: changes to brake semantics on one driver won't propagate to the other (already happens — BTS7960 'brake' is `_hbridge.Stop()`, not both-high). Safety: brake() is the failsafe path; divergence between drivers is a real hazard.",
  "suggested_fix": "Either update the motor-design spec to describe both concrete drivers (L298N and BTS7960) including their brake/coast semantics, or amend it to note that the production driver was changed from BTS7960 to L298N after the spec was written."
}
```

### C12-P0-03: Drivetrain `update()` signature drift (spec: no arg; code: takes dt)

```json
{
  "id": "C12-P0-03",
  "title": "Drivetrain::update spec signature has no parameter; code takes float dt",
  "spec_file": "docs/superpowers/specs/2026-05-06-omni-drivetrain-design.md",
  "spec_line_range": "110-112",
  "spec_quote": "    void drive(const BodyVelocity& velocity) override;\n    void update() override;\n    void stop() override;",
  "file": "src/drivetrain/OmniDrivetrain.h",
  "line_range": "25-33",
  "quote": "    void update(float dt) override {\n        for (auto* m : _motors) m->update();\n\n        float s0 = _pids[0]->compute(_targetRPMs[0], _motors[0]->getFilteredRPM(), dt);",
  "risk": "Spec deferred dt handling to the PID class ('PID receives dt externally or computes it internally — that's a PID design decision'). Code: PID::compute takes dt, OmniDrivetrain::update takes dt, and the caller (controlTask in main_robot.cpp:190-191) passes a hard-coded nominal `CONTROL_PERIOD_MS/1000 = 0.010f` rather than measuring actual elapsed time. If vTaskDelayUntil drifts (it's phase-locked, but jitter is observable on the ESP32 with WiFi+I2C), the PID derivative term will be computed against the wrong dt while the motor encoder dt is independently measured from micros(). Two dt sources (FIT0186Motor::update line 38-39 vs OmniDrivetrain::update arg) is also a footgun.",
  "suggested_fix": "Update the omni-drivetrain spec to describe the dt plumbing decision and to clarify that the caller passes nominal dt. Optionally consider computing actual dt in controlTask via micros() and forwarding it, to remove a class of jitter-induced PID error."
}
```

### C12-P0-04: Kinematics expected behavior on pure strafe contradicts derivation sanity check

```json
{
  "id": "C12-P0-04",
  "title": "drivetrain-test-procedure says strafe puts motor 0 at largest magnitude; kinematics derivation says |ω₀|=vy/(rcosα) while |ω₁|=|ω₂|=0.5×vy/(rcosα) — consistent; but spec prose 'motor 0 active, motors 1 and 2 contribute' is ambiguous about sign",
  "spec_file": "docs/superpowers/specs/2026-05-06-omni-drivetrain-design.md",
  "spec_line_range": "193-194",
  "spec_quote": "- `OmniKinematics`: pure strafe → motor 0 active, motors 1 and 2 contribute",
  "file": "src/drivetrain/OmniKinematics.h",
  "line_range": "32-34",
  "quote": "        float w0 = scale * (-SIN_0   * v.vx - COS_0   * v.vy - R * v.omega);\n        float w1 = scale * (-SIN_120 * v.vx - COS_120 * v.vy - R * v.omega);\n        float w2 = scale * (-SIN_240 * v.vx - COS_240 * v.vy - R * v.omega);",
  "risk": "Magnitudes and signs match the derivation in omni-kinematics-derivation.md (motor 0 is 2× the magnitude of motors 1 and 2 on pure strafe). However the verification bullet in the design spec is editorial only ('contribute'). The test (`test_omni_kinematics.cpp`) is what locks the invariant. Drift here is mostly that the spec's verification bullet doesn't actually assert the 2:1 ratio. P0 only because pure-strafe sign errors are catastrophic for teleop.",
  "suggested_fix": "Tighten the spec verification bullet to say 'motor 0 magnitude == 2 × motor 1 magnitude == 2 × motor 2 magnitude on pure strafe, with motor 0 sign opposite to motors 1/2' and confirm a unit test covers it."
}
```

---

## P1 — Non-safety constant / behavior drift

### C12-P1-01: PID tuning values drift between troubleshooting doc and code

```json
{
  "id": "C12-P1-01",
  "title": "drivetrain-test-procedure suggests Kp=1.0, Ki=0.1, Kd=0.01; code uses Kp=0.005, Ki=0.002, Kd=0.0005",
  "spec_file": "docs/drivetrain-test-procedure.md",
  "spec_line_range": "135-135",
  "spec_quote": "| Target RPMs look right but actuals don't converge | PID gains need tuning (Kp=1.0, Ki=0.1, Kd=0.01) |",
  "file": "src/main_robot.cpp",
  "line_range": "66-68",
  "quote": "PID g_pid0(0.005f, 0.002f, 0.0005f, -1.0f, 1.0f, 5.0f);\nPID g_pid1(0.005f, 0.002f, 0.0005f, -1.0f, 1.0f, 5.0f);\nPID g_pid2(0.005f, 0.002f, 0.0005f, -1.0f, 1.0f, 5.0f);",
  "risk": "Code gains are ~200× smaller than the doc's suggested starting point. Doc is a troubleshooting hint, but if a contributor uses it as a reference they will introduce 200× over-aggressive control, which on this drivetrain will saturate within one tick. The PID design spec (2026-05-06 omni drivetrain) explicitly defers tuning, so this is the only doc that mentions numbers — and it is stale.",
  "suggested_fix": "Replace the parenthetical in drivetrain-test-procedure.md with the actual tuned values from main_robot.cpp:66-68, or remove the specific numbers (since deadband=5 RPM is also undocumented)."
}
```

### C12-P1-02: PID compute signature drift (spec: no dt; code: dt is explicit arg)

```json
{
  "id": "C12-P1-02",
  "title": "OmniDrivetrain spec PID interface omits dt; code requires dt",
  "spec_file": "docs/superpowers/specs/2026-05-06-omni-drivetrain-design.md",
  "spec_line_range": "154-160",
  "spec_quote": "class PID {\npublic:\n    float compute(float setpoint, float measurement) -> float;  // output in [-1, 1]\n    void reset();\n};",
  "file": "src/control/PID.h",
  "line_range": "15-15",
  "quote": "    float compute(float setpoint, float measurement, float dt) {",
  "risk": "Spec is an 'assumed interface' for the drivetrain to reason against. The mismatch isn't a bug — it just makes the spec obsolete as documentation of how the drivetrain talks to PID. Future readers will need to cross-reference PID.h.",
  "suggested_fix": "Update the spec snippet to match `float compute(float setpoint, float measurement, float dt)` or add a note that the final interface added explicit dt."
}
```

### C12-P1-03: Spec says deadband is 'TBD'; code hard-codes 5.0 RPM

```json
{
  "id": "C12-P1-03",
  "title": "PID deadband of 5 RPM is undocumented; PID spec excludes tuning",
  "spec_file": "docs/superpowers/specs/2026-05-06-omni-drivetrain-design.md",
  "spec_line_range": "164-171",
  "spec_quote": "## What This Spec Does NOT Cover\n\n- Acceleration ramping (PID handles smoothing naturally)\n- IMU integration / heading correction (DrivetrainController)\n- Roll/pitch stabilization (DrivetrainController)\n- FreeRTOS task setup and loop timing (main.cpp concern)\n- PID tuning values",
  "file": "src/main_robot.cpp",
  "line_range": "66-68",
  "quote": "PID g_pid0(0.005f, 0.002f, 0.0005f, -1.0f, 1.0f, 5.0f);",
  "risk": "5 RPM deadband (the 6th arg) silently zeros the integral and output whenever setpoint==0 AND |measurement|<5 RPM. This produces a discontinuity (no PWM at all when commanding zero, even though the wheel may be coasting at <5 RPM). Not in any spec, but it is observable behavior — an operator commanding 'stop' through the staleness ramp will see PID outputs go to exactly zero at the bottom of the ramp rather than producing a small holding/damping output.",
  "suggested_fix": "Add a 'PID deadband: 5 RPM' line to either the drivetrain spec or the controller-discussion doc."
}
```

### C12-P1-04: Spec describes only 'no-load 251 RPM'; code uses it as MAX_RPM saturation limit

```json
{
  "id": "C12-P1-04",
  "title": "DrivetrainConfig.maxRPM described as 'motor no-load RPM'; OmniKinematics treats it as a hard saturation ceiling",
  "spec_file": "docs/superpowers/specs/2026-05-06-omni-drivetrain-design.md",
  "spec_line_range": "47-52",
  "spec_quote": "struct DrivetrainConfig {\n    float wheelRadius;   // meters\n    float robotRadius;   // center to wheel contact point, meters\n    float tiltAngle;     // wheel tilt from vertical, radians\n    float maxRPM;        // motor no-load RPM (251 for FIT0186)\n};",
  "file": "src/drivetrain/OmniKinematics.h",
  "line_range": "45-50",
  "quote": "        if (maxAbs > _config.maxRPM) {\n            float factor = _config.maxRPM / maxAbs;\n            for (int i = 0; i < 3; i++) {\n                rpms[i] *= factor;\n            }\n        }",
  "risk": "Using no-load RPM as the saturation ceiling means under load the motors will never achieve the demanded RPM (load curve falls off quickly from no-load), and PID will permanently see a positive error. Anti-windup (PID.h:32-37) prevents integrator runaway but the output is pegged at outputMax. Spec doesn't distinguish 'no-load RPM' (a motor spec sheet value) from 'practical max RPM under load' (the kinematics-saturation constant). On hardware this is the right knob to expose, but the doc never says so.",
  "suggested_fix": "Rename `maxRPM` to `saturationRPM` (or add a comment 'set conservatively below no-load to leave PID headroom') and document in RobotConstants.h."
}
```

### C12-P1-05: Control rate documented in CONTEXT and code, but spec only says '100 Hz target'

```json
{
  "id": "C12-P1-05",
  "title": "Spec '100 Hz target' is approximate; code is exact 10ms via vTaskDelayUntil",
  "spec_file": "docs/superpowers/specs/2026-05-06-omni-drivetrain-design.md",
  "spec_line_range": "129-129",
  "spec_quote": "Called at a fixed rate (100 Hz target) from a FreeRTOS task on Core 1:",
  "file": "src/drivetrain/RobotConstants.h",
  "line_range": "15-16",
  "quote": "constexpr uint32_t STALENESS_TIMEOUT_MS = 200;  // ramp-to-zero window on silence\nconstexpr uint32_t CONTROL_PERIOD_MS    = 10;   // 100 Hz tick",
  "risk": "Editorial. 100 Hz target vs 10ms period is the same number. P1 because controlTask passes the nominal 0.010 directly to PID rather than measuring actual elapsed; if the spec said 'PID dt is the nominal CONTROL_PERIOD_MS' it would be load-bearing. It doesn't.",
  "suggested_fix": "Either accept this as editorial or add a 'PID dt = CONTROL_PERIOD_MS/1000, not measured' note to the spec."
}
```

### C12-P1-06: Spec _motors is not declared but code's Drivetrain base owns _motors[3]

```json
{
  "id": "C12-P1-06",
  "title": "Spec OmniDrivetrain class layout shows kinematics+pids only; code inherits _motors[3] from Drivetrain base",
  "spec_file": "docs/superpowers/specs/2026-05-06-omni-drivetrain-design.md",
  "spec_line_range": "114-121",
  "spec_quote": "private:\n    void setMotorSpeeds(float s0, float s1, float s2);\n\n    OmniKinematics _kinematics;\n    PID* _pids[3];\n    float _targetRPMs[3] = {};\n};",
  "file": "src/drivetrain/Drivetrain.h",
  "line_range": "27-29",
  "quote": "protected:\n    Motor* _motors[3];\n};",
  "risk": "Spec's class diagram doesn't show where motors live. Code factored them into the base. Functionally fine, but spec readers may think OmniDrivetrain holds its own motor refs. Editorial.",
  "suggested_fix": "Update OmniDrivetrain spec class sketch to show `_motors` is inherited from `Drivetrain<TVelocity>`."
}
```

### C12-P1-07: Spec '`stop` brake' description omits clearing target RPMs

```json
{
  "id": "C12-P1-07",
  "title": "Spec stop() resets PIDs + brakes motors; code additionally clears _targetRPMs",
  "spec_file": "docs/superpowers/specs/2026-05-06-omni-drivetrain-design.md",
  "spec_line_range": "139-144",
  "spec_quote": "Brakes all motors and resets PID state:\n\n```cpp\nfor (auto* m : _motors) m->brake();\nfor (auto* p : _pids) p->reset();\n```",
  "file": "src/drivetrain/OmniDrivetrain.h",
  "line_range": "39-43",
  "quote": "    void stop() override {\n        for (auto* m : _motors) m->brake();\n        for (auto* p : _pids) p->reset();\n        for (int i = 0; i < 3; i++) _targetRPMs[i] = 0.0f;\n    }",
  "risk": "Code adds `_targetRPMs[i] = 0.0f` which the spec omits. This matters because if `stop()` is called and then `update()` is called before the next `drive()`, the PID setpoint will be 0 instead of the last commanded value. Code behavior is more correct; spec is stale.",
  "suggested_fix": "Add the target-RPM zeroing to the spec snippet."
}
```

### C12-P1-08: Controller-discussion mandates tilt limiting; code has only PassthroughDrivetrainController

```json
{
  "id": "C12-P1-08",
  "title": "controller-discussion.md says 'tilt limiting is mandatory'; runtime controller is passthrough and ignores IMU",
  "spec_file": "docs/controller-discussion.md",
  "spec_line_range": "16-23",
  "spec_quote": "## Why Tilt Limiting is Mandatory\n\nThe FIT0186 motors produce ~177N combined stall force (3 motors × 1.77 N·m stall torque / 0.03m wheel radius). The drive platform weighs ~1.4kg (~13.7N). The force-to-weight ratio is ~13:1, meaning the motors can trivially push the platform up the inside of the sphere and flip it.",
  "file": "src/drivetrain/controller/PassthroughDrivetrainController.h",
  "line_range": "17-19",
  "quote": "    void update(const BodyVelocity& command, const IMUReading& /*imuData*/) override {\n        _drivetrain.drive(command);\n    }",
  "risk": "The controller spec is explicit: tilt limiting is mandatory due to the 13:1 force-to-weight ratio. The runtime controller is a no-op that forwards the command unchanged and discards IMU data. CONTEXT.md notes this is a known gap (#11 in §9), but the controller-discussion doc reads as a hard requirement — there is no doc-side acknowledgement that the current runtime controller is unsafe-by-design. The IMU `read()` is still called every tick (paying the I2C cost) but the result is discarded. Risk: anyone running the robot with a teleop client will hit this limit on aggressive commands.",
  "suggested_fix": "Add a 'Status: not yet implemented; PassthroughDrivetrainController is a stub' header to controller-discussion.md, or convert this doc into a tracked TODO with explicit warning that the production controller does no tilt limiting."
}
```

### C12-P1-09: Staleness window not described in the omni drivetrain spec

```json
{
  "id": "C12-P1-09",
  "title": "200ms staleness ramp is implemented in main_robot but absent from any spec",
  "spec_file": "docs/superpowers/specs/2026-05-06-omni-drivetrain-design.md",
  "spec_line_range": "164-171",
  "spec_quote": "## What This Spec Does NOT Cover\n\n- Acceleration ramping (PID handles smoothing naturally)",
  "file": "src/main_robot.cpp",
  "line_range": "175-182",
  "quote": "            const uint32_t age = now - last_fresh_ms;\n            if (age < STALENESS_TIMEOUT_MS) {\n                const float scale = 1.0f - static_cast<float>(age) /\n                                            static_cast<float>(STALENESS_TIMEOUT_MS);\n                drive_cmd = { last_known.vx * scale,\n                              last_known.vy * scale,\n                              last_known.omega * scale };\n            }",
  "risk": "Spec disclaims acceleration ramping; the code introduces a different kind of ramping (a watchdog-driven decay to zero on command silence). The spec exists for the drivetrain layer and the ramp lives in main_robot, so this is partly out of scope — but there is no doc whatsoever for the 200ms behavior, the linear shape, or the rationale (all three are CONTEXT.md inventions). New contributors can't infer this from the code alone.",
  "suggested_fix": "Add a short 'Command staleness / dead-window policy' section to either controller-discussion.md or a new teleop-loop spec."
}
```

### C12-P1-10: Spec says `[env:native]` adds `-DBB8_DEBUG` for all builds; production envs don't

```json
{
  "id": "C12-P1-10",
  "title": "Motor design spec example platformio adds -DBB8_DEBUG unconditionally; production envs lack it (BB8_ASSERT is a no-op on hardware)",
  "spec_file": "docs/superpowers/specs/2026-04-23-fit0186-motor-design.md",
  "spec_line_range": "269-291",
  "spec_quote": "build_flags =\n    -std=gnu++17\n    -Wall\n    -Wextra\n    -Isrc/config\n    -Isrc/motor\n[...]\n    -DBB8_DEBUG",
  "file": "platformio.ini",
  "line_range": "8-22",
  "quote": "build_flags =\n    -std=gnu++17\n    -Wall\n    -Wextra\n    -Isrc/config\n    -Isrc/motor\n    -Isrc/motor/constants",
  "risk": "Spec said 'Remove `-DBB8_DEBUG` for release builds'. The actual platformio has `-DBB8_DEBUG` only on `[env:native]` (line 83), never on `[env:robot]` etc. So on hardware, every `BB8_ASSERT` is `((void)0)`. That includes asserts in OmniKinematics (positive radii, sane tilt angle), L298NDriver::setOutput (range check on `[-1,1]`), FIT0186Motor (positive CPR), PID (finite setpoint/measurement, non-negative dt). Spec said this would happen but only in release — production = release here. P1 because it changes the runtime failure mode (silent UB vs panic with backtrace).",
  "suggested_fix": "Spec is accurate (it says release removes BB8_DEBUG) but CONTEXT.md observes nobody is set up to flip in a debug hardware build. Either add a `[env:robot_debug]` env that defines BB8_DEBUG, or note in the spec that the production envs are 'release' by default."
}
```

### C12-P1-11: Spec says wheel angles are 'CW from above'; OmniKinematics treats CW with -sin/-cos formula

```json
{
  "id": "C12-P1-11",
  "title": "Spec gives angles in 'clockwise from above'; derivation explains CW-angle convention with -sin/-cos; code's SIN_120=+0.866 matches +sin(120°) (CCW math)",
  "spec_file": "docs/omni-kinematics-derivation.md",
  "spec_line_range": "52-55",
  "spec_quote": "In our frame (x-forward, y-left, z-up), CW angles correspond to negative rotation. The wheel at CW angle θ_i has position direction (cos θ, −sin θ) and rolling direction (−sin θ, −cos θ). Projecting the contact-point velocity onto this direction:\n\n```\nv_wheel_i = -sin(θ_i) × vx - cos(θ_i) × vy - R × ω\n```",
  "file": "src/drivetrain/OmniKinematics.h",
  "line_range": "22-25",
  "quote": "        constexpr float SIN_120 =  0.86602540378f;\n        constexpr float COS_120 = -0.5f;\n        constexpr float SIN_240 = -0.86602540378f;\n        constexpr float COS_240 = -0.5f;",
  "risk": "Subtle. The spec defines θ_i as the CW angle from front and uses sin(θ_i) in the formula. sin(120° CW) seen as a normal math angle is sin(-120°) = -0.866. But sin(+120°) = +0.866, and the code uses +0.866. Reading the derivation literally, the code should use SIN_120 = -0.866 for CW. The two cancel out because in the derivation 'CW angles correspond to negative rotation' is folded into the v_wheel_i formula's signs (rolling direction is (-sin θ, -cos θ) for CW θ), but anyone tracing the code against the spec will be confused about which convention sin/cos are evaluated in. CONTEXT.md says 'Motor 1: back-right (120° clockwise)'. Code's `SIN_120 = +0.866` is the standard-math sin(120°) — i.e. it's actually treating the angle as CCW from front in the math, while the spec calls it CW. The kinematics works because the (−sin θ × vx − cos θ × vy) formula was derived in the CW frame, so plugging in standard-math sin(120°) gives the right answer for a wheel physically at 120° CW. But this is brittle — a casual change to the IK form (e.g. switching to (+sin, +cos) convention) would silently flip directions.",
  "suggested_fix": "Add a comment in OmniKinematics.h immediately above the SIN_120/SIN_240 declarations: `// θ is CW from front but evaluated as standard math sin/cos; the negative signs in the formula handle the CW convention. See omni-kinematics-derivation.md.`"
}
```

---

## P2 — Editorial drift

### C12-P2-01: Spec says `update()` returns void; code's `void update(float dt) override` — already covered, but spec layout drifts in arrow direction also

```json
{
  "id": "C12-P2-01",
  "title": "OmniDrivetrain spec setMotorSpeeds is a separate helper; in code it's an inline private one-liner",
  "spec_file": "docs/superpowers/specs/2026-05-06-omni-drivetrain-design.md",
  "spec_line_range": "146-148",
  "spec_quote": "### `setMotorSpeeds(s0, s1, s2)`\n\nHelper that sets each motor to its respective speed.",
  "file": "src/drivetrain/OmniDrivetrain.h",
  "line_range": "46-50",
  "quote": "    void setMotorSpeeds(float s0, float s1, float s2) {\n        _motors[0]->setSpeed(s0);\n        _motors[1]->setSpeed(s1);\n        _motors[2]->setSpeed(s2);\n    }",
  "risk": "None — purely structural.",
  "suggested_fix": "None needed."
}
```

### C12-P2-02: Spec for motor-design lists Motor 'getRPM()/getFilteredRPM() return cached values'; code matches except FIT0186Motor's _encoder is public for testing

```json
{
  "id": "C12-P2-02",
  "title": "FIT0186Motor::_encoder is public for testing; spec doesn't mention this hack",
  "spec_file": "docs/superpowers/specs/2026-04-23-fit0186-motor-design.md",
  "spec_line_range": "211-243",
  "spec_quote": "Concrete `Motor` implementation. Owns a `Driver&` reference and an `ESP32Encoder` instance.",
  "file": "src/motor/FIT0186Motor.h",
  "line_range": "70-72",
  "quote": "    // Public for testing — test code sets _encoder._count directly\n    ESP32Encoder _encoder;",
  "risk": "Encapsulation leak in service of a test. Doesn't affect runtime.",
  "suggested_fix": "Either friend the test fixture or add a setter for testing. Or accept as-is and note in spec."
}
```

### C12-P2-03: Motor-design spec wheel-radius example (0.03m) disagrees with code's RobotConstants (0.046225m)

```json
{
  "id": "C12-P2-03",
  "title": "controller-discussion uses 0.03m wheel radius in stall-force calculation; RobotConstants.h says 0.046225m",
  "spec_file": "docs/controller-discussion.md",
  "spec_line_range": "17-17",
  "spec_quote": "The FIT0186 motors produce ~177N combined stall force (3 motors × 1.77 N·m stall torque / 0.03m wheel radius).",
  "file": "src/drivetrain/RobotConstants.h",
  "line_range": "9-9",
  "quote": "constexpr float WHEEL_RADIUS  = 0.046225f;  // 9.245 cm diameter / 2, in meters",
  "risk": "The controller-discussion's 'force-to-weight ratio 13:1' argument is sized on a 30mm wheel; the real wheel is 46.225mm. Recomputed stall force = 3 × 1.77 / 0.046225 = ~115N (still ~8:1 vs 13.7N, still concerning, but materially different from 13:1). The doc's safety argument doesn't change qualitatively, but the numbers don't reflect reality.",
  "suggested_fix": "Update controller-discussion.md to use 0.046225m and recompute the ratio."
}
```

### C12-P2-04: Untested invariant — kinematics tilt-angle near pi/2 BB8_ASSERT never fires in production

```json
{
  "id": "C12-P2-04",
  "title": "OmniKinematics asserts tilt is not near pi/2, but BB8_ASSERT is a no-op on hardware",
  "spec_file": "docs/omni-kinematics-derivation.md",
  "spec_line_range": "73-74",
  "spec_quote": "At α = 0 (vertical wheels, flat floor), cos(0) = 1 and this reduces to the standard equation. As α → 90° (wheels horizontal), cos(α) → 0 and the required wheel speed → ∞, which is the physical limit where the wheels can no longer drive the sphere.",
  "file": "src/drivetrain/OmniKinematics.h",
  "line_range": "15-16",
  "quote": "        BB8_ASSERT(std::abs(cosf(config.tiltAngle)) > 1e-6f,\n                   \"OmniKinematics: tiltAngle too close to pi/2\");",
  "risk": "On production builds BB8_ASSERT is a no-op (see C12-P1-10). The 1/cos(α) divide by ~0 produces inf which propagates into RPM targets → saturation kicks in → all wheels capped at MAX_RPM. So in practice this is bounded, but the assert that the derivation relies on is silent on hardware.",
  "suggested_fix": "Promote this check to a runtime guard (`if (std::abs(cosAlpha) < 1e-6f) return {0,0,0};`) or fail loud during construction (Serial.print + ESP.restart)."
}
```

### C12-P2-05: Untested invariant — '0 = coast' in spec applies to L298N but not BTS7960Driver

```json
{
  "id": "C12-P2-05",
  "title": "Motor spec setSpeed(0) = coast; BTS7960Driver setOutput(0) calls Disable() (not coast in the same sense)",
  "spec_file": "docs/superpowers/specs/2026-04-23-fit0186-motor-design.md",
  "spec_line_range": "104-104",
  "spec_quote": "- `setSpeed(0)` — coast (both H-bridge outputs LOW, motor disconnected)",
  "file": "src/motor/driver/BTS7960Driver.cpp",
  "line_range": "20-23",
  "quote": "    if (value == 0.0f) {\n        _hbridge.Disable();\n        return;\n    }",
  "risk": "BTS7960's `Disable()` (drives the L_EN/R_EN low) tri-states the H-bridge — terminals high impedance. The L298N coast (both PWM at 0) leaves the H-bridge enabled with both outputs low — terminals shorted to GND through the low-side MOSFETs. These produce different motor behavior: L298N's '0 = coast' actually short-circuit-brakes through the GND rail; BTS7960's '0 = disable' truly coasts (freewheel via body diodes). Spec says 'coast'; depending on which driver is in the build, the behavior differs. P2 because BTS7960 is excluded from all envs at HEAD, so only L298N's behavior matters in production.",
  "suggested_fix": "Spec the precise expected behavior of `setSpeed(0)` per driver, or unify by having both implementations produce the same observable state."
}
```

### C12-P2-06: IMU test procedure SDA/SCL pin doc disagrees with main_robot.cpp

```json
{
  "id": "C12-P2-06",
  "title": "imu-test-procedure says SDA=26, SCL=25; main_robot.cpp uses SDA=21, SCL=22",
  "spec_file": "docs/imu-test-procedure.md",
  "spec_line_range": "11-17",
  "spec_quote": "| BNO055 Pin | ESP32 Pin |\n|------------|-----------|\n| VIN        | 3.3V      |\n| GND        | GND       |\n| SDA        | GPIO 26   |\n| SCL        | GPIO 25   |",
  "file": "src/main_robot.cpp",
  "line_range": "53-54",
  "quote": "constexpr int I2C_SDA = 21;\nconstexpr int I2C_SCL = 22;",
  "risk": "Operator following the test procedure will wire the IMU to the wrong pins and `g_imu.begin()` will fail, triggering `ESP.restart()` (main_robot.cpp:222-223). Reboot loop until rewired. The most recent commit (f8ca95f 'remap motor and IMU pins to match new harness') updated the code; the doc lagged.",
  "suggested_fix": "Update imu-test-procedure.md table to SDA=21, SCL=22."
}
```

---

## Summary

| Severity | Count |
|----------|-------|
| P0       | 4 |
| P1       | 11 |
| P2       | 6 |
| **Total**| **21** |

**Highest-priority themes**

1. **`ENCODER_CPR` discrepancy (700 vs 2800)** — direct multiplier on every RPM reading; spec needs to acknowledge x4 quadrature decoding.
2. **Controller spec calls tilt limiting 'mandatory'; runtime controller is a stub.** No tilt safety in the loop.
3. **`Drivetrain::update()` and `PID::compute()` signatures** drifted from spec (both gained `dt`).
4. **PID gains and deadband are undocumented**; troubleshooting doc cites stale values.
5. **Pin docs lag the harness remap** (motor 2, encoder 1, IMU I2C).
6. **`BB8_ASSERT` is a no-op in every production env** — every defensive check in IK, drivers, PID is dead on hardware.
