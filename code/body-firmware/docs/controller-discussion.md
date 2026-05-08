# DrivetrainController Design Discussion

## Context

The `DrivetrainController<TCommand, TIMUData, TVelocity>` abstract class exists but has no concrete implementation. This document captures design decisions for the omni-drivetrain controller that sits between user input and `OmniDrivetrain`.

## Role

The controller translates joystick commands + IMU feedback into `BodyVelocity` commands for the drivetrain. It handles:

1. **Yaw stabilization** — PID on heading to correct drift from the omniwheels
2. **Tilt monitoring** — IMU pitch/roll used to detect when the drive platform is climbing the sphere wall
3. **Tilt limiting** — cut motors when the platform tilts too far, re-engage when it settles back

## Why Tilt Limiting is Mandatory

The FIT0186 motors produce ~177N combined stall force (3 motors × 1.77 N·m stall torque / 0.03m wheel radius). The drive platform weighs ~1.4kg (~13.7N). The force-to-weight ratio is ~13:1, meaning the motors can trivially push the platform up the inside of the sphere and flip it.

Supporting evidence:
- BB-8 builders report tumbling/turtling events during turns and control overcorrections
- Omniwheel sphere-bot academic tests show controllability loss beyond ~15° tilt
- The IK tilt correction `1/cos(α)` degrades as the platform climbs — wheel speed demand grows while effective traction drops
- Adam Green's BB-8 build documented the internal mass spinning around inside the ball faster than the ball itself, until IMU-based pitch control was added

## Tilt Behavior

When the robot accelerates, the drive platform climbs the inside of the sphere in the direction of acceleration. This is inherent to the design — the wheels push the sphere, reaction force pushes the platform, and inertia causes it to ride up the wall.

Past a certain angle:
- The IK contact geometry breaks down (wheels lose effective traction)
- The platform risks flipping upside down inside the sphere

## Proposed Controller Modes

### Active

Tilt within safe range. Yaw PID active, joystick input translated to `BodyVelocity`, motors driven normally.

### Recovery

Tilt exceeds the traction threshold. Motors coasted/braked, joystick input ignored. Wait for gravity to pull the platform back to the bottom of the sphere. Re-engage when tilt drops below a lower re-engage threshold (hysteresis to prevent oscillating between modes).

### Stopped

Explicit stop command. All motors braked, PIDs reset.

## Two Thresholds

| Threshold | Purpose | Approximate range | Action |
|-----------|---------|-------------------|--------|
| Traction limit | IK breaks down, driving is unproductive | ~15-20° (tune on hardware) | Coast motors, enter recovery |
| Safety limit | Hard ceiling to prevent flip | ~30-35° (conservative, below flip point) | Brake immediately |

The traction limit is derived partly from the sphere/wheel geometry (sphere radius, wheel radius, tilt angle α) and partly tuned empirically. The safety limit is a hard cap that should never be reached during normal operation.

Both thresholds should use hysteresis — the re-engage angle should be several degrees below the cutoff angle.

## PID Channels

| Channel | Setpoint | Measurement | Output | Purpose |
|---------|----------|-------------|--------|---------|
| Yaw | Commanded heading (or heading hold) | IMU yaw | `BodyVelocity.omega` correction | Heading stabilization |
| Pitch | 0° (upright) | IMU pitch | `BodyVelocity.vx` correction | Dampen forward/back tilt |
| Roll | 0° (upright) | IMU roll | `BodyVelocity.vy` correction | Dampen left/right tilt |

The pitch/roll PIDs aren't "balancing" the robot (it's naturally stable via gravity) — they're damping oscillations and keeping the platform centered so the drivetrain stays effective.

## Open Questions

- Exact sphere inner diameter and wheel tilt angle α — needed to derive the theoretical traction limit
- Should pitch/roll PID corrections be additive with joystick input or should they modulate it?
- Should the yaw PID hold absolute heading or just dampen rotation rate?
- What IMU data rate does the BNO055 deliver, and does it match the 100Hz control loop?

## References

- `DrivetrainController.h` — abstract base class in `src/drivetrain/controller/`
- `OmniDrivetrain.h` — the drivetrain this controller feeds into
- `BNO055IMU.h` — IMU implementation in `src/imu/`
- Adam Green's BB-8 build: https://github.com/adamgreen/bb-8
- BB-8 Builders Club: https://bb8builders.club
