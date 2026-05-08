# OmniDrivetrain — Design Spec

## Context

The body-firmware has a complete motor stack (FIT0186Motor + L298N/BTS7960 drivers + encoder feedback) and a BNO055 IMU. The next layer up is the Drivetrain — mapping a body-frame velocity command to individual wheel speeds via inverse kinematics and per-wheel PID control.

This spec covers only the Drivetrain layer. Heading correction, tilt stabilization, and IMU integration belong to `DrivetrainController` (designed separately).

## Architecture

Two new components plus supporting types:

```
BodyVelocity ──► OmniKinematics ──► target RPMs ──► OmniDrivetrain (PID per wheel) ──► Motor::setSpeed()
                 (pure math)                         (orchestrator)
```

`OmniKinematics` is stateless and independently testable. `OmniDrivetrain` owns the kinematics instance, 3 PID instances, and wires everything together.

## Wheel Layout

Three omniwheels at 120-degree spacing, tilted inward at an angle from vertical (contact the inside of the sphere):

- Motor 0: front (0°) — still when moving straight forward
- Motor 1: back-right (120° clockwise from above)
- Motor 2: back-left (240° clockwise from above)

The tilt angle is a runtime parameter in `DrivetrainConfig`.

## New Types

### `BodyVelocity` (`src/drivetrain/BodyVelocity.h`)

```cpp
struct BodyVelocity {
    float vx = 0;     // forward speed, m/s (positive = forward)
    float vy = 0;     // strafe speed, m/s (positive = left)
    float omega = 0;  // rotation rate, rad/s (positive = CCW from above)
};
```

Body frame: x-forward, y-left, z-up (right-handed). Matches BNO055 IMU default frame.

### `DrivetrainConfig` (`src/drivetrain/DrivetrainConfig.h`)

```cpp
struct DrivetrainConfig {
    float wheelRadius;   // meters
    float robotRadius;   // center to wheel contact point, meters
    float tiltAngle;     // wheel tilt from vertical, radians
    float maxRPM;        // motor no-load RPM (251 for FIT0186)
};
```

All geometry parameters are runtime values so the mechanical design can evolve without recompiling the kinematics.

## OmniKinematics (`src/drivetrain/OmniKinematics.h`)

Stateless utility. Pure math, no hardware dependencies.

```cpp
class OmniKinematics {
public:
    explicit OmniKinematics(const DrivetrainConfig& config);

    std::array<float, 3> toWheelRPMs(const BodyVelocity& velocity) const;

private:
    DrivetrainConfig _config;
};
```

### Inverse kinematics

Standard 3-omniwheel IK with tilt correction. Full derivation in [`docs/omni-kinematics-derivation.md`](../../omni-kinematics-derivation.md).

For wheels at angles θ₀=0°, θ₁=120°, θ₂=240°, tilted at angle α from vertical:

```
ω_i = (1 / (r * cos(α))) * [-sin(θ_i) * vx - cos(θ_i) * vy - R * omega]
```

Where `r` = wheel radius, `R` = robot radius, `α` = tilt angle. Positive omega = CCW from above.

Expanded per wheel (motor 0 at front):

```
ω₀ = (1 / (r * cos(α))) * [-vy - R*omega]
ω₁ = (1 / (r * cos(α))) * [-0.866*vx + 0.5*vy - R*omega]
ω₂ = (1 / (r * cos(α))) * [0.866*vx + 0.5*vy - R*omega]
```

Convert to RPM: `RPM_i = ω_i * 60 / (2π)`.

### Saturation scaling

If any wheel's target RPM exceeds `maxRPM`, all three are scaled proportionally to preserve the commanded trajectory direction. No individual wheel clipping.

## OmniDrivetrain (`src/drivetrain/OmniDrivetrain.h`)

Concrete implementation of `Drivetrain<BodyVelocity>`.

```cpp
class OmniDrivetrain : public Drivetrain<BodyVelocity> {
public:
    OmniDrivetrain(Motor& m0, Motor& m1, Motor& m2,
                   const DrivetrainConfig& config,
                   PID& pid0, PID& pid1, PID& pid2);

    void drive(const BodyVelocity& velocity) override;
    void update() override;
    void stop() override;

private:
    void setMotorSpeeds(float s0, float s1, float s2);

    OmniKinematics _kinematics;
    PID* _pids[3];
    float _targetRPMs[3] = {};
};
```

### `drive(velocity)`

Runs `_kinematics.toWheelRPMs(velocity)` and stores the 3 target RPMs. Does not touch motors directly — that happens in `update()`.

### `update()`

Called at a fixed rate (100 Hz target) from a FreeRTOS task on Core 1:

1. Refresh encoder state: `for (auto* m : _motors) m->update();`
2. Per-wheel PID: `output_i = _pids[i]->compute(_targetRPMs[i], _motors[i]->getFilteredRPM())`
3. Apply: `setMotorSpeeds(output0, output1, output2)`

PID outputs are in [-1, 1], mapping directly to `Motor::setSpeed()`.

### `stop()`

Brakes all motors and resets PID state:

```cpp
for (auto* m : _motors) m->brake();
for (auto* p : _pids) p->reset();
```

### `setMotorSpeeds(s0, s1, s2)`

Helper that sets each motor to its respective speed. Exists because each wheel gets a different value from the PID output.

## PID

Generic PID class, designed separately. The Drivetrain uses 3 instances as black boxes. Interface assumed:

```cpp
class PID {
public:
    float compute(float setpoint, float measurement) -> float;  // output in [-1, 1]
    void reset();
};
```

PID receives `dt` externally or computes it internally — that's a PID design decision, not a Drivetrain concern. Anti-windup strategy is also internal to PID.

## What This Spec Does NOT Cover

- Acceleration ramping (PID handles smoothing naturally)
- IMU integration / heading correction (DrivetrainController)
- Roll/pitch stabilization (DrivetrainController)
- FreeRTOS task setup and loop timing (main.cpp concern)
- PID tuning values
- PID class design

## Serial Test Harness (`src/main_drivetrain_test.cpp`)

Same pattern as `main_motor_test.cpp`. Serial input format:

```
<vx> <vy> <omega_degrees>
```

- `0.5 0.0 0.0` — forward
- `0.0 0.5 0.0` — strafe right
- `0.0 0.0 45` — rotate (45 deg/s)
- `stop` — brake all

Omega input is in degrees/s for ergonomics. Converted to radians before calling `drive()`. Prints actual RPM per wheel to serial for verification.

## Verification

### Native tests (`pio test -e native`)

- `OmniKinematics`: pure forward → motor 0 target ≈ 0, motors 1 and 2 equal and opposite sign
- `OmniKinematics`: pure strafe → motor 0 active, motors 1 and 2 contribute
- `OmniKinematics`: pure rotation → all 3 motors equal magnitude
- `OmniKinematics`: saturation scaling preserves direction when exceeding maxRPM
- `OmniKinematics`: tilt angle = 0 matches standard flat-floor equations
- `OmniDrivetrain`: `drive()` stores target RPMs, `update()` calls PID and sets motor speeds
- `OmniDrivetrain`: `stop()` brakes all motors and resets PIDs

### Hardware verification (serial test harness)

- Forward command: motor 0 still, motors 1 and 2 spin in opposite directions
- Strafe command: all 3 motors active
- Rotation command: all 3 motors spin same direction
- `stop` command: all motors brake
- RPM readback approximately matches commanded velocity
