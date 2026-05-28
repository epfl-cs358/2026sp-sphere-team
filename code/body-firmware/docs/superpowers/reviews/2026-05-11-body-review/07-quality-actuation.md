# Cell 7 — Code Quality & Architecture (Actuation)

Lens: Code Quality & Architecture. Scope: motor, drivers, drivetrain, PID.
Anchored at f8ca95f. Severity: P0/P1/P2.

```json
[
  {
    "id": "C7-P1-01",
    "title": "Dead constants in FIT0186 hardware-spec header",
    "severity": "P1",
    "category": "dead-config",
    "file": "src/motor/constants/FIT0186.h",
    "line": "12-13",
    "quote": "    inline constexpr float GEAR_RATIO = 43.8f;\n    inline constexpr uint16_t NO_LOAD_RPM = 251;",
    "evidence": "Project-wide grep finds zero non-doc references to `fit0186::GEAR_RATIO` or `fit0186::NO_LOAD_RPM`. The numeric 251 is hardcoded again as `MAX_RPM = 251.0f` in RobotConstants.h:12 instead of being sourced from `fit0186::NO_LOAD_RPM`. ENCODER_CPR is the only constant actually used (pins.h:20-22).",
    "impact": "Two sources of truth for the motor's no-load RPM. A future motor swap that updates fit0186::NO_LOAD_RPM will silently not affect MAX_RPM, leading to saturation limits that mismatch reality.",
    "recommendation": "Either delete the unused constants and rename the header (it currently advertises a spec sheet it doesn't deliver), or have RobotConstants::MAX_RPM = `fit0186::NO_LOAD_RPM`. Pick one source."
  },
  {
    "id": "C7-P1-02",
    "title": "RAD_TO_RPM conversion duplicated between production code and test",
    "severity": "P1",
    "category": "unit-consistency",
    "file": "src/drivetrain/OmniKinematics.h:36 + test/test_omni_kinematics/test_omni_kinematics.cpp:15",
    "line": "OmniKinematics.h:36",
    "quote": "        constexpr float RAD_TO_RPM = 60.0f / (2.0f * static_cast<float>(M_PI));",
    "evidence": "Same literal expression `60.0f / (2.0f * static_cast<float>(M_PI))` appears in test_omni_kinematics.cpp:15 as the inverse helper, defined locally rather than imported. No central `units` or `conversions` header exists. The encoder->RPM conversion in FIT0186Motor.h:46-47 uses a different idiomatic form (`60.0f / dtSeconds`) — also rad/s-adjacent math expressed independently.",
    "impact": "Three independent rad↔RPM idioms in a codebase whose entire control authority pipeline is RPM-space. A future units bug (e.g. someone writes `2π/60` instead of `60/2π`) is undetectable until live.",
    "recommendation": "Create `src/util/units.h` with `radsToRpm` / `rpmToRads` (and `rpmToRadsPerSec`) `constexpr` inline functions. Have OmniKinematics and tests both call them. Bonus: strong types (see C7-P2-04)."
  },
  {
    "id": "C7-P1-03",
    "title": "Driver abstraction leaks: brake/zero semantics diverge across drivers",
    "severity": "P1",
    "category": "abstraction-boundary",
    "file": "src/motor/driver/L298NDriver.cpp + src/motor/driver/BTS7960Driver.cpp",
    "line": "L298NDriver.cpp:24-28,39-42 + BTS7960Driver.cpp:20-22,38-40",
    "quote": "void L298NDriver::brake() {\n    analogWrite(_pins.fwd, 255);\n    analogWrite(_pins.rev, 255);\n}",
    "evidence": "`Driver.h:14` documents `0.0 = coast` and `brake()` as 'Active braking'. L298N honors this: 0/0 PWM coasts, 255/255 actively brakes (short). BTS7960Driver::setOutput(0) calls `_hbridge.Disable()` (high-impedance disable, not 'coast' in the same circuit sense), and `brake()` calls `Stop()` (library defines this as PWM=0 + Disable on some forks — not necessarily a short). Additionally BTS7960 re-`Enable`s on every nonzero call (cpp:25); L298N has no enable concept.",
    "impact": "The PID hot path issues sub-1.0 floats every 10 ms. If you ever switch from L298N to BTS7960 (note: `BTS7960Driver.cpp` is excluded from every env, so this is untested), the same setpoint sequence will produce different deceleration profiles. The interface contract is not enforced.",
    "recommendation": "Tighten the `Driver` contract — either (a) define three explicit operating modes (`drive(float)`, `coast()`, `brake()`) and remove the magic-zero coast, or (b) add a `BB8_ASSERT` regression test that both drivers translate `setOutput(0)` to the same electrical state. Document the contract in `Driver.h` precisely (currently the comment just says 'coast' which BTS7960 violates)."
  },
  {
    "id": "C7-P2-04",
    "title": "API ergonomics: Motor::setSpeed/Driver::setOutput take unchecked float",
    "severity": "P2",
    "category": "api-ergonomics",
    "file": "src/motor/Motor.h + src/motor/driver/Driver.h + src/drivetrain/BodyVelocity.h",
    "line": "Motor.h:18, Driver.h:15, BodyVelocity.h:3-6",
    "quote": "    // Set motor output. -1.0 = full reverse, 0.0 = coast, 1.0 = full forward.\n    virtual void setSpeed(float speed) = 0;",
    "evidence": "Both `Motor::setSpeed` and `Driver::setOutput` accept a raw `float` whose valid domain is `[-1, 1]`. The only enforcement is `BB8_ASSERT`, which is a no-op in every production env (the context doc §7 confirms `-DBB8_DEBUG` is set only in `[env:native]`). `BodyVelocity` similarly carries three units (m/s, m/s, rad/s) as bare floats; signing convention for omega is in a comment only.",
    "impact": "A caller passing a PID output before clamping, or accidentally an RPM value instead of normalized output, compiles and silently drives the motor with garbage. The PID output range is parameterized to `[-1, 1]` in main_robot.cpp:66-68, so this works by convention, not by type.",
    "recommendation": "Introduce a `NormalizedOutput` strong type (a `struct { float v; }` with a private constructor + `clamped(float)` factory) used at the Motor/Driver boundary. Use compile-time tagged units (`RadiansPerSecond`, `RPM`, `MetersPerSecond`) on the drivetrain API. Costs nothing at runtime; catches the entire class of unit-confusion bugs in CI. Already partially possible since `Drivetrain` is templated on `TVelocity`."
  },
  {
    "id": "C7-P2-05",
    "title": "Drivetrain interface leaks 3-motor implementation into the base",
    "severity": "P2",
    "category": "interface-leak",
    "file": "src/drivetrain/Drivetrain.h",
    "line": "13-14, 27-28",
    "quote": "    Drivetrain(Motor& m0, Motor& m1, Motor& m2)\n        : _motors{&m0, &m1, &m2} {}\n[...]\nprotected:\n    Motor* _motors[3];",
    "evidence": "The 'generic' `Drivetrain<TVelocity>` interface hardcodes exactly three motors in its constructor and stores them in a `Motor* _motors[3]`. A differential drive (2 motors) or mecanum (4) would not fit. Per the lens question (Q7), this means a swap to a differential impl requires editing the base, not just adding a sibling subclass.",
    "impact": "Locks the interface to omni geometry. Future hardware variants (the doc trail mentions a possible tilt-stabilizer / different bot) can't reuse the abstraction.",
    "recommendation": "Move motor storage into the concrete classes. The base should be `virtual void drive(const TVelocity&) = 0; virtual void update(float dt) = 0; virtual void stop() = 0;` with no constructor and no member array. If a `getMotorCount()` accessor is needed for telemetry, expose it virtually."
  },
  {
    "id": "C7-P2-06",
    "title": "PID is reusable in principle; signature/docs imply motor-only",
    "severity": "P2",
    "category": "reusability",
    "file": "src/control/PID.h",
    "line": "8-13, 20",
    "quote": "    PID(float kp, float ki, float kd, float outputMin, float outputMax,\n        float deadband = 0.0f)",
    "evidence": "Math-wise `PID::compute(setpoint, measurement, dt)` is unit-agnostic — kp/ki/kd carry the implied units, deadband sits in measurement units, and output is clamped to caller-supplied range. Could drive heading hold (deg setpoint → rate output) unmodified. The class itself has no Motor coupling. However the lens-required check turned up no doc-level statement that this is a general-purpose PID; the only callers are 3 motor-RPM loops, so the test surface is narrow.",
    "impact": "Low. The component is reusable; this is a documentation gap that may discourage reuse.",
    "recommendation": "Add a one-line header comment in PID.h stating 'unit-agnostic; caller chooses setpoint/measurement units and gains accordingly'. Mention the dt<1e-4 early-return as an explicit contract (currently silent return of `_lastOutput`)."
  },
  {
    "id": "C7-P2-07",
    "title": "L298N vs BTS7960 driver: 80% structural duplication, no shared base",
    "severity": "P2",
    "category": "duplication",
    "file": "src/motor/driver/L298NDriver.cpp + src/motor/driver/BTS7960Driver.cpp",
    "line": "L298NDriver.cpp:19-37 / BTS7960Driver.cpp:17-36",
    "quote": "    uint8_t pwm = static_cast<uint8_t>(std::abs(value) * 255.0f);",
    "evidence": "Both drivers: (1) repeat the same `BB8_ASSERT(value >= -1.0f && value <= 1.0f, ...)` (literal copy), (2) repeat `uint8_t pwm = static_cast<uint8_t>(std::abs(value) * 255.0f)`, (3) repeat the sign→direction branch. The float-to-8bit-PWM mapping is duplicated and could easily drift (e.g. one driver later switching to 10-bit ledc PWM). Neither uses `std::lround`, so both quantize via truncation.",
    "impact": "Maintenance hazard. Already drifted: BTS7960 has a second redundant `if (pwm == 0)` check after the early `value == 0.0f` check; L298N has no such redundancy. Float-equality against 0 is also distinct between the two.",
    "recommendation": "Extract `static uint8_t normalizedToPwm(float value)` (and the assert) into `Driver.h` or a `driver/detail.h`. Each concrete driver only owns the H-bridge wiring step. Bonus: this also fixes the floating-point `== 0.0f` vs `pwm == 0` divergence."
  },
  {
    "id": "C7-P2-08",
    "title": "Inconsistent RPM unit suffix on parameters & members",
    "severity": "P2",
    "category": "naming-consistency",
    "file": "src/motor/Motor.h + src/drivetrain/OmniDrivetrain.h + src/drivetrain/DrivetrainConfig.h",
    "line": "Motor.h:18 + OmniDrivetrain.h:54 + DrivetrainConfig.h:7",
    "quote": "    virtual void setSpeed(float speed) = 0;",
    "evidence": "`Motor::setSpeed(float speed)` takes a normalized `[-1,1]` (not actually a speed — name conflates output-fraction and RPM). Meanwhile `_targetRPMs[3]` (OmniDrivetrain.h:54), `getFilteredRPM()`, `_rawRPM`, `maxRPM` (DrivetrainConfig.h:7), `getTargetRPMs()` correctly carry the `RPM` suffix. `BodyVelocity.vx/vy/omega` carry no unit suffix (just comments).",
    "impact": "`setSpeed` is the worst offender: a caller might pass an RPM in and expect a speed. The compiler doesn't catch it (see C7-P2-04). All other actuation code consistently suffixes RPM, which makes the omission stand out.",
    "recommendation": "Rename `Motor::setSpeed(float speed)` → `Motor::setOutput(float normalized)` to match `Driver::setOutput`. Then the layer cake reads consistently: Driver/Motor speak normalized output, Drivetrain speaks RPM, BodyVelocity speaks SI. Touches the public Motor interface but the only callers are OmniDrivetrain and main_motor_test."
  },
  {
    "id": "C7-P2-09",
    "title": "ESP32Encoder member is public 'for testing'",
    "severity": "P2",
    "category": "encapsulation",
    "file": "src/motor/FIT0186Motor.h",
    "line": "70-71",
    "quote": "    // Public for testing — test code sets _encoder._count directly\n    ESP32Encoder _encoder;",
    "evidence": "The encoder is moved out of `private:` solely to enable tests to poke its `_count`. This is the only WHY comment in the actuation code and it correctly documents non-obvious intent — but the solution itself is wrong: it leaks an entire hardware-driven member into the public surface of every consumer.",
    "impact": "Any code that holds a `FIT0186Motor&` can now reach into the encoder and call e.g. `attachFullQuad` a second time. Not a runtime risk today (no such callers) but the test seam is sitting in the production header.",
    "recommendation": "Add a `protected: setEncoderCountForTest(int64_t)` (or a `friend class FIT0186MotorTest;` forward-decl). Move `_encoder` back to private. The WHY comment is good practice; the technique is the issue."
  },
  {
    "id": "C7-P2-10",
    "title": "Heavy includes in motor header (Arduino.h, ESP32Encoder.h)",
    "severity": "P2",
    "category": "header-hygiene",
    "file": "src/motor/FIT0186Motor.h",
    "line": "14-15",
    "quote": "#include <ESP32Encoder.h>\n#include <Arduino.h>",
    "evidence": "FIT0186Motor.h is the only header in the actuation subsystem pulling in `Arduino.h` (drags the entire Arduino-ESP32 core into every TU that includes it transitively) and `ESP32Encoder.h`. Because the class is a template, both headers propagate to every consumer (`OmniDrivetrain.h` → `Drivetrain.h` users → main_*.cpp). Arduino.h is needed for `micros()` only.",
    "impact": "Firmware, so build-time impact is minor — but `Arduino.h` also redefines macros (e.g. `min`, `max`, `B0`) that have bitten C++ projects before. The note in the lens question accepts that 'build times matter less' for firmware, so this is low priority.",
    "recommendation": "Move `update()` body to a `FIT0186Motor.inl` or `.cpp` (explicit instantiation of `FilterN=5`), or pImpl the encoder. Replace `micros()` use with a forward-declared free function in a slim 'time.h' wrapper if hard isolation is wanted."
  },
  {
    "id": "C7-P2-11",
    "title": "Comments that restate code; missing WHY on PID dt-floor and OmniDrivetrain::stop unwiring",
    "severity": "P2",
    "category": "comment-hygiene",
    "file": "src/motor/Motor.h + src/control/PID.h + src/drivetrain/OmniDrivetrain.h",
    "line": "Motor.h:14,17,20,23,26 + PID.h:20 + OmniDrivetrain.h:39-43",
    "quote": "    // Refresh encoder state and recompute RPM. Call at a fixed interval.\n    virtual void update() = 0;",
    "evidence": "Motor.h's inline comments largely restate the method name (`begin`, `setSpeed`, `brake`, `getRPM`, `getFilteredRPM` all have a sentence that paraphrases the identifier). PID.h:20 has `if (dt < 1e-4f) return _lastOutput;` with no comment explaining why 1e-4 was picked (which would be the only valuable comment in the file). OmniDrivetrain::stop() has no doc that it is unreachable from the runtime loop (context §6.11). The good comment is in FIT0186Motor.h:70 — keep that style.",
    "impact": "Per the project rule ('only leave doc'), most actuation comments are noise; one important WHY is missing (the 1e-4 dt-floor — is this 100 µs? Is it tied to micros() resolution? To the encoder noise floor?).",
    "recommendation": "Strip the paraphrase comments in Motor.h, Driver.h, Drivetrain.h. Add a one-line WHY at PID.h:20 (e.g. 'protect derivative term against control-task jitter; chosen to be << 1 sample period'). Optionally note in OmniDrivetrain::stop that it is currently only called from PassthroughDrivetrainController::stop which itself is unwired."
  },
  {
    "id": "C7-P2-12",
    "title": "DrivetrainConfig uses bare float for tiltAngle without enforced unit",
    "severity": "P2",
    "category": "unit-consistency",
    "file": "src/drivetrain/DrivetrainConfig.h",
    "line": "6",
    "quote": "    float tiltAngle;     // wheel tilt from vertical, radians",
    "evidence": "All four fields are bare floats. `tiltAngle` says 'radians' in a comment; the call site `RobotConstants.h:11` writes `30.0f * static_cast<float>(M_PI) / 180.0f` correctly. But OmniKinematics.h:27 does `cosf(_config.tiltAngle)` — a degrees value here would compile and silently misbehave (cos(30deg-as-radians) ≈ -0.99 vs intended 0.866).",
    "impact": "If anyone constructs a `DrivetrainConfig{...}` without going through `RobotConstants::drivetrainConfig()` (tests, future ports), passing 30.0f instead of 30°→rad is a silent ~22% saturation bug.",
    "recommendation": "Either: (a) rename field to `tiltAngleRadians`, or (b) add `Radians` strong-type wrapper. (a) is the cheap fix. The assert at OmniKinematics.h:15 catches only the pi/2-singular case, not the deg/rad confusion."
  }
]
```

## Summary

The actuation subsystem is competently structured: clear `Driver → Motor → Drivetrain → DrivetrainController` layers, PID is unit-agnostic, kinematics is pure functional and saturation-aware. Two real issues stand out: **(a)** the `Driver` contract is loose enough that L298N and BTS7960 produce electrically different behavior for the same float input (C7-P1-03), and **(b)** every unit is a bare `float`, which (combined with disabled `BB8_ASSERT` in production envs) leaves no safety net against unit-confusion or out-of-range bugs (C7-P2-04, C7-P2-08, C7-P2-12). Dead constants in `FIT0186.h` (C7-P1-01) and duplicated rad↔RPM math (C7-P1-02) point to a missing `util/units.h`. Driver code is ~80% duplicated and already drifting (C7-P2-07). Header hygiene and comment hygiene are mostly fine; one good WHY comment lives in `FIT0186Motor.h:70`, several others paraphrase the code (C7-P2-11). The `Drivetrain` base hardcodes 3 motors, blocking a clean differential-drive swap (C7-P2-05).
