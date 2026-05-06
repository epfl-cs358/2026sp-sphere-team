# Omni-Wheel Inverse Kinematics Derivation

## Setup

A 3-omniwheel drivetrain inside a spherical shell. Wheels are equally spaced at 120° and tilted inward at angle α from vertical to contact the sphere's inner surface.

### Coordinate Frame (body-frame, right-handed)

- **x**: forward (direction the robot faces)
- **y**: right
- **z**: down

All velocities are in the body frame. Positive omega = clockwise when viewed from above.

### Wheel Positions (viewed from above, clockwise)

| Motor | Angle (CW from front) | Position |
|-------|----------------------|----------|
| 0     | 0°                   | Front    |
| 1     | 120°                 | Back-right |
| 2     | 240°                 | Back-left |

Each wheel's axis of rotation points toward the center of the robot. The wheel rolls in the direction perpendicular to its axis.

### Parameters

| Symbol | Meaning |
|--------|---------|
| r      | Wheel radius (m) |
| R      | Robot radius — distance from center to wheel contact point (m) |
| α      | Tilt angle from vertical (rad) |
| vx     | Forward body velocity (m/s), positive = forward |
| vy     | Strafe body velocity (m/s), positive = right |
| ω      | Rotation rate (rad/s), positive = CW from above |

### Tilt correction assumption

The `1/cos(α)` factor assumes the wheel radius `r` already accounts for the sphere contact geometry — i.e., `r` is the effective wheel radius at the contact point, not necessarily the physical wheel radius. This is a valid first-order approximation. If the exact sphere contact geometry differs, `r` should be adjusted accordingly.

## Derivation

### Step 1: Flat-floor 3-omniwheel IK

For a wheel at angle θ_i (measured clockwise from front), the wheel's rolling direction is perpendicular to the line from center to wheel. The component of the robot's velocity along that rolling direction determines the wheel speed.

The velocity at the wheel contact point has two parts:
1. **Translation**: the robot's linear velocity (vx, vy)
2. **Rotation**: tangential velocity from the robot spinning = R × ω

In our frame (x-forward, y-right, z-down), for wheel i at angle θ_i (CW from front), projecting onto the wheel's rolling direction:

```
v_wheel_i = sin(θ_i) × vx + cos(θ_i) × vy + R × ω
```

The `sin(θ_i) × vx + cos(θ_i) × vy` term projects the linear velocity onto the rolling direction. The `+R × ω` term adds the rotational contribution: positive ω (CW) makes all wheels spin in the same positive direction.

The sign differences from the z-up convention:
- `cos(θ_i) × vy` is now `+` (y-right instead of y-left)
- `R × ω` is now `+` (positive ω = CW instead of CCW)

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
ω_i = (1 / (r × cos(α))) × [sin(θ_i) × vx + cos(θ_i) × vy + R × ω]
```

### Step 4: Expand per wheel

Using clockwise angles θ₀ = 0°, θ₁ = 120°, θ₂ = 240°:

**Motor 0 (front, θ = 0°):**
```
sin(0°) = 0,  cos(0°) = 1

ω₀ = (1 / (r × cos(α))) × [0 × vx + 1 × vy + R × ω]
ω₀ = (1 / (r × cos(α))) × [vy + R × ω]
```

**Motor 1 (back-right, θ = 120°):**
```
sin(120°) = √3/2 ≈ 0.866,  cos(120°) = -1/2 = -0.5

ω₁ = (1 / (r × cos(α))) × [0.866 × vx + (-0.5) × vy + R × ω]
ω₁ = (1 / (r × cos(α))) × [0.866 × vx - 0.5 × vy + R × ω]
```

**Motor 2 (back-left, θ = 240°):**
```
sin(240°) = -√3/2 ≈ -0.866,  cos(240°) = -1/2 = -0.5

ω₂ = (1 / (r × cos(α))) × [-0.866 × vx + (-0.5) × vy + R × ω]
ω₂ = (1 / (r × cos(α))) × [-0.866 × vx - 0.5 × vy + R × ω]
```

### Step 5: Matrix form

```
┌ ω₀ ┐         1          ┌  0      1    R ┐   ┌ vx ┐
│ ω₁ │ = ───────────── ×  │  √3/2  -1/2  R │ × │ vy │
└ ω₂ ┘   r × cos(α)       └ -√3/2  -1/2  R ┘   └ ω  ┘
```

### Step 6: Convert to RPM

```
RPM_i = ω_i × 60 / (2π)
```

## Sanity Checks

### Pure forward (vx > 0, vy = 0, ω = 0)

```
ω₀ = 0                           ← motor 0 is still (correct: front wheel)
ω₁ = +0.866 × vx / (r × cos(α)) ← positive
ω₂ = -0.866 × vx / (r × cos(α)) ← negative
```

Motors 1 and 2 spin at equal magnitude, opposite directions. Forward motion comes from their combined horizontal thrust. Motor 0 contributes nothing. ✓

### Pure strafe right (vx = 0, vy > 0, ω = 0)

```
ω₀ = +vy / (r × cos(α))         ← positive (motor 0 spins forward)
ω₁ = -0.5 × vy / (r × cos(α))  ← negative (motors 1 and 2 spin backward)
ω₂ = -0.5 × vy / (r × cos(α))  ← negative (same magnitude as motor 1)
```

Motor 0 spins forward, motors 1 and 2 spin backward at half the magnitude. Net force is rightward. ✓

### Pure CW rotation (vx = 0, vy = 0, ω > 0)

```
ω₀ = +R × ω / (r × cos(α))
ω₁ = +R × ω / (r × cos(α))
ω₂ = +R × ω / (r × cos(α))
```

All three motors spin at equal speed in the same positive direction. ✓

### Tilt angle = 0

All equations reduce to standard flat-floor IK (cos(0) = 1). ✓

## References

- [ROS2 Mobile Robot Kinematics — Omnidirectional Wheels](https://control.ros.org/rolling/doc/ros2_controllers/doc/mobile_robot_kinematics.html)
- [Modern Robotics Ch. 13.2 — Omnidirectional Wheeled Mobile Robots](https://modernrobotics.northwestern.edu/nu-gm-book-resource/13-2-omnidirectional-wheeled-mobile-robots-part-1-of-2/)
- Borisov et al., "Nonholonomic dynamics and control of a spherical robot with an internal omniwheel platform" (2016)
- SpheriDrive: A spherical robot with an innovative 3-wheeled platform (2025)
