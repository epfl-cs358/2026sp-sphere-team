# Omni-Wheel Inverse Kinematics Derivation

## Setup

A 3-omniwheel drivetrain inside a spherical shell. Wheels are equally spaced at 120° and tilted inward at angle α from vertical to contact the sphere's inner surface.

### Coordinate Frame (body-frame, right-handed)

- **x**: forward (direction the robot faces)
- **y**: left
- **z**: up

All velocities are in the body frame. Positive omega = counter-clockwise when viewed from above (right-hand rule around z-up).

This matches the BNO055 IMU's default coordinate frame, so no axis remapping is needed.

### Wheel Positions (viewed from above, CCW from body +x)

Angles are measured **counter-clockwise from the body +x axis (forward)** when viewed from above, matching `DrivetrainConfig::wheelAngles` consumed by `OmniKinematics`.

| Motor | Angle θ (CCW from +x) | Physical position |
|-------|-----------------------|-------------------|
| 0     | 180°                  | Back              |
| 1     | 300°                  | Front-right       |
| 2     |  60°                  | Front-left        |

Each wheel's axis of rotation points toward the center of the robot. The wheel rolls in the direction perpendicular to its axis.

Source of truth for motor index ↔ physical position: `src/main_motor_test.cpp:5-7` and `RobotConstants.h:29`.

### Parameters

| Symbol | Meaning |
|--------|---------|
| r      | Wheel radius (m) |
| R      | Robot radius — distance from center to wheel contact point (m) |
| α      | Tilt angle from vertical (rad) |
| vx     | Forward body velocity (m/s), positive = forward |
| vy     | Strafe body velocity (m/s), positive = left |
| ω      | Rotation rate (rad/s), positive = CCW from above |

### Tilt correction assumption

The `1/cos(α)` factor assumes the wheel radius `r` already accounts for the sphere contact geometry — i.e., `r` is the effective wheel radius at the contact point, not necessarily the physical wheel radius. This is a valid first-order approximation. If the exact sphere contact geometry differs, `r` should be adjusted accordingly.

## Derivation

### Step 1: Flat-floor 3-omniwheel IK

For a wheel at angle θ_i (measured CCW from body +x when viewed from above), its contact point sits at position `R × (cos θ_i, sin θ_i)`. The wheel's rolling direction is perpendicular to that radial vector, tangent to the circle. We pick the tangent `(-sin θ_i, cos θ_i)` so that positive wheel angular velocity drives the body CCW (matching `+ω = CCW from above`).

The velocity at the wheel contact point has two parts:
1. **Translation**: the robot's linear velocity (vx, vy)
2. **Rotation**: tangential velocity from the body spinning at ω = `R × ω` along the same tangent direction

Projecting the contact-point velocity onto the rolling direction `(-sin θ_i, cos θ_i)`:

```
v_wheel_i = -sin(θ_i) × vx + cos(θ_i) × vy + R × ω
```

The `-sin(θ_i) × vx + cos(θ_i) × vy` term projects the linear velocity onto the rolling direction. The `+R × ω` term adds the rotational contribution: positive ω (CCW) makes all wheels spin in the same positive direction. This matches `OmniKinematics::toWheelRPMs` in `src/drivetrain/OmniKinematics.h:30`.

The wheel angular velocity is:

```
ω_i = v_wheel_i / r
```

### Step 2: Tilt correction

When wheels are tilted at angle α from vertical, the effective radius of contact with the sphere changes. The wheel's force is projected onto the horizontal plane by a factor of cos(α). This means the wheel must spin faster to achieve the same horizontal velocity:

```
ω_i = v_wheel_i / (r × cos(α))
```

At α = 0 (vertical wheels, flat floor), cos(0) = 1 and this reduces to the standard equation. As α → 90° (wheels horizontal), cos(α) → 0 and the required wheel speed → ∞, which is the physical limit where the wheels can no longer drive the sphere.

### Step 3: Full equations

Combining steps 1 and 2:

```
ω_i = (1 / (r × cos(α))) × [-sin(θ_i) × vx + cos(θ_i) × vy + R × ω]
```

### Step 4: Expand per wheel

Using CCW angles θ₀ = 180°, θ₁ = 300°, θ₂ = 60°:

**Motor 0 (back, θ = 180°):**
```
sin(180°) = 0,  cos(180°) = -1

ω₀ = (1 / (r × cos(α))) × [-0 × vx + (-1) × vy + R × ω]
ω₀ = (1 / (r × cos(α))) × [-vy + R × ω]
```

**Motor 1 (front-right, θ = 300°):**
```
sin(300°) = -√3/2 ≈ -0.866,  cos(300°) = 1/2 = 0.5

ω₁ = (1 / (r × cos(α))) × [-(-0.866) × vx + 0.5 × vy + R × ω]
ω₁ = (1 / (r × cos(α))) × [0.866 × vx + 0.5 × vy + R × ω]
```

**Motor 2 (front-left, θ = 60°):**
```
sin(60°) = √3/2 ≈ 0.866,  cos(60°) = 1/2 = 0.5

ω₂ = (1 / (r × cos(α))) × [-0.866 × vx + 0.5 × vy + R × ω]
```

### Step 5: Matrix form

```
┌ ω₀ ┐         1          ┌  0     -1    R ┐   ┌ vx ┐
│ ω₁ │ = ───────────── ×  │  √3/2   1/2  R │ × │ vy │
└ ω₂ ┘   r × cos(α)       └ -√3/2   1/2  R ┘   └ ω  ┘
```

### Step 6: Convert to RPM

```
RPM_i = ω_i × 60 / (2π)
```

## Sanity Checks

### Pure forward (vx > 0, vy = 0, ω = 0)

```
ω₀ = 0                            ← motor 0 (back) is still
ω₁ = +0.866 × vx / (r × cos(α))  ← positive (front-right)
ω₂ = -0.866 × vx / (r × cos(α))  ← negative (front-left)
```

The two front wheels (motors 1, 2) spin at equal magnitude, opposite directions. Forward motion comes from their combined horizontal thrust. Motor 0 (back) contributes nothing — its rolling axis is purely lateral, so it cannot push the shell forward. ✓

### Pure strafe left (vx = 0, vy > 0, ω = 0)

```
ω₀ = -vy / (r × cos(α))          ← negative (back wheel spins one way)
ω₁ = +0.5 × vy / (r × cos(α))   ← positive (front-right)
ω₂ = +0.5 × vy / (r × cos(α))   ← positive (front-left, same as motor 1)
```

Motor 0 (back) takes a single full-magnitude contribution in the negative direction; motors 1 and 2 (front pair) each take half-magnitude in the positive direction. The three contributions sum to net leftward thrust on the shell, matching the `+y = LEFT` REP-103 convention. ✓

### Pure CCW rotation (vx = 0, vy = 0, ω > 0)

```
ω₀ = +R × ω / (r × cos(α))
ω₁ = +R × ω / (r × cos(α))
ω₂ = +R × ω / (r × cos(α))
```

All three motors spin at equal speed in the same positive direction, driving the body CCW from above. ✓

### Tilt angle = 0

All equations reduce to standard flat-floor IK (cos(0) = 1). ✓

## References

- [ROS2 Mobile Robot Kinematics — Omnidirectional Wheels](https://control.ros.org/rolling/doc/ros2_controllers/doc/mobile_robot_kinematics.html)
- [Modern Robotics Ch. 13.2 — Omnidirectional Wheeled Mobile Robots](https://modernrobotics.northwestern.edu/nu-gm-book-resource/13-2-omnidirectional-wheeled-mobile-robots-part-1-of-2/)
- Borisov et al., "Nonholonomic dynamics and control of a spherical robot with an internal omniwheel platform" (2016)
- SpheriDrive: A spherical robot with an innovative 3-wheeled platform (2025)
