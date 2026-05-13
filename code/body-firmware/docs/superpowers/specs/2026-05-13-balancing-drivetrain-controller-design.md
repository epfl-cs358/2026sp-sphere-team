# BalancingDrivetrainController — Design Spec

## Context

Body-firmware has a complete drivetrain (OmniDrivetrain + per-wheel PID) and a
BNO055 IMU on the platform. Today's teleop routes the user's `BodyVelocity`
directly to `drivetrain.drive()` via `PassthroughDrivetrainController` — no IMU
feedback, no balancing.

This spec replaces the passthrough controller with a Segway-style balancing
controller that:
- Derives a tilt setpoint from the user's `BodyVelocity` command and uses
  IMU feedback to drive the platform to that tilt.
- When idle (`cmd = 0`), holds the platform level; the resulting wheel motion
  decelerates the sphere.
- Faults safely when tilt exceeds a 60° envelope.
- Runs only under explicit arm/disarm/kill control.
- Exposes live PID tuning over RemoteSerial, persisted to NVS.

OTA (`OtaSafeMode`) and OmniDrivetrain are unchanged.

## Architecture

```
joystick ──► WebSocket ──► CommandFrameParser ─┬─► CommandLatch<BodyVelocity> ─► control task @100Hz
                                               │
                                               └─► ArmingState transitions (arm/disarm/kill)

control task:
   if (ArmingState::isArmed() && !OtaSafeMode::isUpdating())
       controller.update(cmd, imu, dt)
   else
       controller.stop(); drive_cmd = 0

controller.update():
   cfg = configSlot.load()
   (gx, gy, gz) = quatToBodyGravity(imu.orientation)
   tiltMagSin   = sqrt(gx² + gy²)
   schmitt fault on tiltMagSin against cfg.envelopeEnter/ExitSin
   pitch_target = clamp(cfg.tiltPerVelocity · cmd.vx, ±cfg.maxTiltSetpoint)
   roll_target  = clamp(cfg.tiltPerVelocity · cmd.vy, ±cfg.maxTiltSetpoint)
   pitch_actual = atan2(gx, -gz)        roll_actual  = atan2(gy, -gz)
   vx_out = pitchPid.compute(pitch_target, pitch_actual, gyro_pitch_rate, dt)
   vy_out = rollPid.compute( roll_target,  roll_actual,  gyro_roll_rate,  dt)
   clamp(vx_out, vy_out, cfg.maxOutputVelocity)
   drivetrain.drive({vx_out, vy_out, cmd.omega})
```

Two independent safety layers gate the controller:
- `ArmingState` — explicit operator control (DISARMED / ARMED / KILLED).
- `OtaSafeMode::isUpdating()` — existing OTA halt (unchanged).

A third, internal-only safety (Schmitt tilt-fault) sits inside the controller
itself and halts driving when tilt exceeds the safe envelope.

## New types

### `BalanceConfig` — `src/drivetrain/BalanceConfig.h`

```cpp
struct BalanceConfig {
    // Command → tilt-setpoint mapping
    float tiltPerVelocity;     // rad per (m/s)
    float maxTiltSetpoint;     // rad; clamp on commanded tilt

    // Pitch PID (forward/back)
    float pitchKp, pitchKi, pitchKd;
    // Roll PID (left/right) — separate so platform asymmetries can be tuned
    float rollKp,  rollKi,  rollKd;

    // Output safety
    float maxOutputVelocity;   // m/s; clamp on |vx_out|, |vy_out|

    // Tilt-fault Schmitt thresholds — stored as sin(angle) for cheap compare
    float envelopeEnterSin;    // sin(60°) ≈ 0.866
    float envelopeExitSin;     // sin(55°) ≈ 0.819

    // Gyro axis sign overrides. Default +1; bring-up may set to -1 if the
    // BNO055 axis convention disagrees with our pitch/roll sign assumptions.
    float gyroPitchSign;       // multiplies imu.gyro.y when computing pitch rate
    float gyroRollSign;        // multiplies imu.gyro.x when computing roll rate
};
```

Defaults from `RobotConstants::balanceConfig()`:

```cpp
inline BalanceConfig balanceConfig() {
    return {
        .tiltPerVelocity   = 0.30f,   // 1 m/s → ~17° tilt
        .maxTiltSetpoint   = 0.35f,   // ~20°; leaves ample headroom below 60° fault envelope
        .pitchKp = 1.50f, .pitchKi = 0.0f, .pitchKd = 0.15f,
        .rollKp  = 1.50f, .rollKi  = 0.0f, .rollKd  = 0.15f,
        .maxOutputVelocity = 1.00f,   // m/s; just below physical max (~1.05)
        .envelopeEnterSin  = 0.866f,  // sin(60°)
        .envelopeExitSin   = 0.819f,  // sin(55°)
        .gyroPitchSign     = 1.0f,
        .gyroRollSign      = 1.0f,
    };
}
```

All values are placeholders — actual values come from bench tuning over
RemoteSerial. Defaults are conservative enough to compile, arm, and not
destroy hardware on first run.

### `TiltSetpoint` (internal only)

Used inside the controller for readability. Not a wire-level type.

```cpp
struct TiltSetpoint { float pitch; float roll; };  // rad, body-frame
```

### `ArmingState` — `src/safety/ArmingState.{h,cpp}`

```cpp
namespace ArmingState {
    enum class State : uint8_t { Disarmed, Armed, Killed };

    void  begin();           // initialise to Disarmed
    State get();             // lock-free atomic load
    bool  isArmed();         // shorthand

    void arm();              // Disarmed → Armed (no-op if Killed)
    void disarm();           // Armed    → Disarmed (no-op if Killed)
    void kill();             // any      → Killed
    void clearKill();        // Killed   → Disarmed only
}
```

Stored as `std::atomic<State>` (`uint8_t` underlying; lock-free on ESP32 /
Xtensa LX6). Memory ordering: writers use `store(release)`, readers
(control task hot path) use `load(acquire)`. Single-writer invariant
documented: all transition functions are only called from Core 0
(RemoteSerial dispatcher, WebSocket dispatcher, OtaSafeMode::onStart). No
`compare_exchange` needed.

Each transition logs to `Serial` and `RemoteSerial`. Boot default
`Disarmed` — robot cannot move until first `arm`.

## BalancingDrivetrainController

`src/drivetrain/controller/BalancingDrivetrainController.{h,cpp}`

```cpp
class BalancingDrivetrainController
    : public DrivetrainController<BodyVelocity, IMUReading, BodyVelocity>
{
public:
    BalancingDrivetrainController(Drivetrain<BodyVelocity>& drive,
                                  IMU<IMUReading>& imu,
                                  std::atomic<const BalanceConfig*>& configSlot);

    void update(const BodyVelocity& cmd,
                const IMUReading& imu,
                float dt) override;
    void stop() override;

private:
    PID  _pitchPid;
    PID  _rollPid;
    bool _inFault = false;
    std::atomic<const BalanceConfig*>& _configSlot;
};
```

### `update(cmd, imu, dt)` algorithm

1. Snapshot config: `const BalanceConfig& cfg = *_configSlot.load(std::memory_order_acquire);`
2. Derive body-frame gravity from the quaternion (see "Quaternion → gravity").
3. `tiltMagSin = sqrt(gx² + gy²)`.
4. Schmitt fault state machine on `tiltMagSin` vs
   `(envelopeEnterSin, envelopeExitSin)`. If in fault, emit zero and return.
   On fault entry, reset both balance PIDs.
5. `pitch_target = clamp(cfg.tiltPerVelocity · cmd.vx, ±cfg.maxTiltSetpoint)`;
   roll symmetric.
6. `pitch_actual = atan2(gx, -gz)`; roll symmetric.
7. PID with explicit rate (gyro as derivative source):
   ```
   vx_out = _pitchPid.compute(pitch_target, pitch_actual, gyro_pitch_rate, dt)
   vy_out = _rollPid.compute( roll_target,  roll_actual,  gyro_roll_rate,  dt)
   ```
   Mapping committed:
   - `gyro_pitch_rate = SIGN_PITCH * imu.gyro.y` (pitch = rotation about body-y; right-hand rule with thumb along +y)
   - `gyro_roll_rate  = SIGN_ROLL  * imu.gyro.x` (roll  = rotation about body-x)

   `SIGN_PITCH` and `SIGN_ROLL` are `+1` by default and verified at bring-up
   with this procedure (documented in the bench-harness):

   1. Place platform level. Confirm `gx ≈ 0`, `gy ≈ 0`, `gz ≈ -1`.
   2. Tilt platform forward slowly. Confirm `pitch_actual` increases monotonically and `imu.gyro.y` is non-zero with consistent sign during the tilt.
   3. Numerically integrate `imu.gyro.y` over the motion and compare to the change in `pitch_actual`. If they have opposite signs, set `SIGN_PITCH = -1`.
   4. Repeat for roll with `imu.gyro.x`.

   The two signs are constants in `BalanceConfig` (`gyroPitchSign`, `gyroRollSign`)
   so they're configurable via RemoteSerial without reflash if bring-up reveals
   an inverted axis.
8. Clamp `vx_out`, `vy_out` to `±cfg.maxOutputVelocity`.
9. `omega_out = cmd.omega` (passthrough).
10. `drivetrain.drive({vx_out, vy_out, omega_out})`.

### `stop()`

- `drivetrain.stop()` — brakes wheels and resets per-wheel PIDs.
- Reset `_pitchPid` and `_rollPid`.
- `_inFault = false`.

Called only on hard shutdown (e.g. KILLED state). Distinct from internal
tilt-fault behaviour, which performs a *soft* halt (zero `BodyVelocity` to
`drivetrain.drive()`, no brake) so per-wheel PIDs ramp wheels to zero
gracefully and the controller can resume when tilt re-enters the envelope.

### Quaternion → body-frame gravity

The orientation quaternion `q = (w, x, y, z)` rotates body-frame vectors into
world frame, i.e. `v_world = R(q) · v_body`. To express world-frame gravity
`(0, 0, -1)` in body frame: `g_body = R(q)^T · (0, 0, -1)`, which gives:

```cpp
static void quatToBodyGravity(const Quat& q, float& gx, float& gy, float& gz) {
    // Body-frame components of gravity (world −z, magnitude 1).
    // Reduces to (0, 0, −1) when q = identity (upright platform).
    gx =  2.0f * (q.w * q.y - q.x * q.z);
    gy = -2.0f * (q.w * q.x + q.y * q.z);
    gz = -(q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
}
```

Verify the formula at compile-time with a quaternion unit test (identity →
`(0, 0, −1)`; 30° pitch-forward rotation → `gx > 0`, `gz ≈ −cos 30°`).

Avoids the BNO055 Euler "automatic orientation detection" discontinuity (per
Adafruit / Bosch guidance: read the quaternion, derive Euler-like quantities
in software at the last moment).

Body-frame axis sign convention is verified empirically on first bring-up:
tilt the platform forward by hand, confirm `gx > 0`. If the BNO055 axis
silkscreen is rotated relative to the platform's "forward" direction, the
fix is to swap or negate components *at this single call site*, not anywhere
downstream.

## `DrivetrainController` base class change

The existing base signature is `update(const TCommand&, const TIMUData&)` —
no `dt`. The balancing controller needs `dt` for PID. Update the base:

```cpp
template <typename TCommand, typename TIMUData, typename TVelocity>
class DrivetrainController {
public:
    virtual void update(const TCommand& cmd, const TIMUData& imu, float dt) = 0;
    virtual void stop() = 0;
    ...
};
```

`PassthroughDrivetrainController::update` is updated to accept and ignore
`dt` (`void update(..., float /*dt*/)`). All call sites — currently just
`main_robot.cpp` and the `test_passthrough_controller` test — are updated
to pass `dt` (already computed locally in the control task for
`drivetrain.update(dt)`).

## PID extension

`src/control/PID.h` — add a rate-aware overload.

```cpp
class PID {
public:
    // Existing — finite-difference derivative computed internally.
    float compute(float setpoint, float measurement, float dt);

    // New — caller supplies rate = d(measurement)/dt (e.g. gyro).
    float compute(float setpoint, float measurement, float rate, float dt);
    ...
};
```

The 3-arg version becomes a thin wrapper:

```cpp
float PID::compute(float setpoint, float measurement, float dt) {
    float rate = _firstCompute ? 0.0f : (measurement - _prevMeasurement) / dt;
    return compute(setpoint, measurement, rate, dt);
}
```

The 4-arg version is the canonical implementation. Anti-windup, deadband,
output clamping, sign convention of `derivative = -rate` (derivative on
measurement, same as before) all unchanged.

Per-wheel PIDs in `OmniDrivetrain` continue calling the 3-arg version.
Balance PIDs call the 4-arg version with gyro components.

## Tilt-fault Schmitt

State: `bool _inFault` (initially false).

| Current state | Condition                            | Action                                                |
| ------------- | ------------------------------------ | ----------------------------------------------------- |
| not in fault  | `tiltMagSin > envelopeEnterSin`      | enter fault: reset balance PIDs, emit zero, log       |
| not in fault  | `tiltMagSin ≤ envelopeEnterSin`      | normal operation                                      |
| in fault      | `tiltMagSin > envelopeExitSin`       | stay in fault, emit zero                              |
| in fault      | `tiltMagSin ≤ envelopeExitSin`       | exit fault, log, resume normal operation              |

The 5° dead band between 60° (enter) and 55° (exit) prevents chatter under
IMU noise at the boundary. PID reset on entry prevents wind-up during the
fault from kicking the wheels when the controller re-engages.

The fault is recoverable — it auto-clears when the platform returns to the
safe envelope. Distinct from `KILLED`, which is latched and requires explicit
operator action.

## Arming + control-task integration

Three behaviours, gated solely by `ArmingState`. OTA composes by *driving*
the state (see "OTA composition" below), not by a separate check in the
hot path.

| State        | drive_cmd                  | `controller.update()`    | Drivetrain      |
| ------------ | -------------------------- | ------------------------ | --------------- |
| `ARMED`      | user (`CommandLatch`)      | called normally          | runs            |
| `DISARMED`   | hard-zeroed                | **skipped**              | runs (PIDs hold motors at zero) |
| `KILLED`     | n/a                        | replaced by `stop()`     | hard brake      |

Rationale: `DISARMED` is "robot off" — no balance assistance, motors held
electrically at zero by the per-wheel PIDs. The platform pendulums passively.
This is the safe default and is what we want during OTA flash. `ARMED` is
the only state in which the balance loop is active. `KILLED` adds an
explicit hard brake on top.

This collapses the previous "balance-while-disarmed" idea — operationally
simpler and aligns with the "disarm during flash" guarantee.

```cpp
const ArmingState::State state = ArmingState::get();

if (state == ArmingState::State::Killed) {
    g_controller->stop();              // brakes wheels, resets balance PIDs
    drive_cmd = { 0.0f, 0.0f, 0.0f };  // for downstream visibility / logging
} else if (state == ArmingState::State::Armed) {
    g_controller->update(drive_cmd, imu_reading, dt);
} else {
    // DISARMED: hard-zero command, skip controller; per-wheel PIDs hold motors.
    drive_cmd = { 0.0f, 0.0f, 0.0f };
    g_drivetrain.drive(drive_cmd);
    // Note: we do NOT call controller.stop() here, so the balance controller
    // retains its internal state. On re-arming, the first update() resets the
    // PIDs via the fault-exit cleanup path if needed.
}
g_drivetrain.update(dt);
```

`ArmingState::begin()` is called in `setup()` before the control task spawns.
Boot default is `DISARMED`.

### OTA composition

`OtaSafeMode::onStart()` calls `ArmingState::disarm()`. From then on, the
control task naturally takes the DISARMED branch and motors are held at
zero throughout the flash window. No separate `OtaSafeMode::isUpdating()`
check is needed in the control task — the safety guarantee comes from the
arming state alone.

If the flash succeeds, the chip reboots and starts again in DISARMED.
If the flash fails and the chip keeps running in degraded mode, it remains
DISARMED until explicit operator action — exactly what we want.

### Safety precedence

When multiple safety conditions overlap, the order is:

```
KILLED        > brake immediately, skip everything
DISARMED      > hard-zero, skip controller
tilt-fault    > internal soft-halt + balance PID reset (only reachable in ARMED)
normal ARMED  > run controller
```

KILLED dominates. Tilt-fault is internal to the controller and only reachable
in ARMED — by definition, DISARMED never enters tilt-fault because the
controller isn't called.

### Auto-disarm on producer silence

If `last_fresh_ms` age exceeds `STALE_DISARM_MS` (2000 ms, defined in
`RobotConstants.h`), the control task calls `ArmingState::disarm()`. The
existing 200 ms ramp-to-zero handles short-term silence; this is the
long-term backstop. Recovery requires the operator to send a fresh `arm`
from the UI.

## Wire protocol — `CommandFrameParser`

The existing parser is strict CSV (`vx,vy,omega` parsed via `strtof`).
Extension method: **leading-sigil dispatch**. The parser inspects the
first byte:

- `'v'` (or any digit, `+`, `-`, `.` for backward compatibility) → velocity
  frame, rest of the line is the existing CSV format.
- `'c'` → control frame, rest of the line is a control verb.

Grammar:

```
v<vx>,<vy>,<omega>      # velocity command (the leading 'v' is optional for
                        # backward-compatibility with the current format).
c:arm                   # arming control
c:disarm
c:kill
c:clearkill
```

`CommandFrameParser` returns a tagged union (`VelocityFrame` |
`ControlFrame`). Velocity frames push to `CommandLatch<BodyVelocity>`;
control frames invoke `ArmingState` transitions directly via a small
dispatcher in `WebSocketCommandProducer`.

Webapp + stream-client both update to emit the prefixed format. Backward
compatibility: legacy unprefixed `<vx>,<vy>,<omega>` lines still parse as
velocity (first byte test treats digit / sign / dot as "velocity-like").

### Webapp UI implications

- Dedicated **Arm** / **Disarm** toggle, separate from joystick.
- Prominent **Kill** button (red, larger).
- A read-back channel reflecting current `ArmingState` so the UI can mirror it.
- Joystick continues sending `v` frames at the existing cadence; firmware
  ignores them unless armed.

Webapp work is in `code/webapp` and outside the firmware spec, but the wire
format is part of this PR's coordination.

## RemoteSerial tuning

`src/drivetrain/BalanceTuner.{h,cpp}` registers commands with `RemoteSerial`:

```
balance show                  # full BalanceConfig dump
balance show pids             # PID gains only
balance set <key> <value>     # mutate one field with range check
balance reset                 # revert to RobotConstants defaults (live, not persisted)
balance save                  # persist current live values to NVS
balance status                # pitch, roll, vx_out, vy_out, fault, time-in-fault, armed
arm | disarm | kill | clearkill | armstate
```

### Atomic config swap (lock-free hot path)

Two static `BalanceConfig` buffers + `std::atomic<const BalanceConfig*>`
slot. Controller reads `slot.load(memory_order_acquire)` once per
`update()`. On `balance set`:

1. Tuner copies the current live config into the spare buffer.
2. Applies the mutation.
3. Validates (kp ≥ 0, envelopeExitSin < envelopeEnterSin, maxTiltSetpoint
   sane, etc.) — rejects invalid values with a clear error.
4. `slot.store(spare_ptr, memory_order_release)`. Active and spare buffers
   swap roles.
5. Tuner sleeps one control period (`vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS + 1))`)
   before accepting another `set`. Guarantees the controller has read the
   new pointer before the previous-live buffer is reused for the next
   mutation. RemoteSerial is human-paced so this is invisible to operators.

No heap, no mutex in the hot path. `BalanceConfig` is trivially-copyable
so writes within the spare buffer are well-defined.

### Persistence

`Preferences` namespace `"balance"`. Stored blob layout:

```
[uint16_t version][BalanceConfig payload]
```

Current `version = 1`. On boot, load the blob and validate:
- Total size matches `sizeof(uint16_t) + sizeof(BalanceConfig)`.
- Version equals current.

If validation fails (absent / size mismatch / version mismatch), fall back
to `RobotConstants::balanceConfig()` and log. On `balance save`, write
version + current live `BalanceConfig`. Adding fields to `BalanceConfig`
in the future requires bumping `version` and writing a one-time migration
path (load old layout, populate new fields with defaults).

Same `Preferences` API as the BNO055 calibration save in `BNO055IMU::saveCalibration`.

## Wire-in to `main_robot.cpp`

Changes (additive — OTA, WiFi, command latch, drivetrain init unchanged):

- Allocate two-buffered `BalanceConfig` storage + `std::atomic<const
  BalanceConfig*>` slot. Load persisted config into one buffer at boot.
- Construct `BalancingDrivetrainController` (replacing
  `PassthroughDrivetrainController`).
- `ArmingState::begin()` before control task spawn.
- Register `BalanceTuner` and arming-state handlers with `RemoteSerial`.
- Control task gates on `(armed && !ota)`; calls `controller.stop()` on the
  gated-off path.
- Add auto-disarm: if command-fresh age > `STALE_DISARM_MS`, call
  `ArmingState::disarm()`.

`PassthroughDrivetrainController` stays in the tree for use by
`main_drivetrain_test.cpp`.

## Bench harness — `main_balance_test.cpp`

Stripped-down `main_robot.cpp` for bench PID tuning without the webapp.

- No WebSocket, no `WebSocketCommandProducer`, no `CommandLatch`.
- Commands injected via RemoteSerial: `cmd <vx> <vy> <omega>` writes a
  static `BodyVelocity` consumed by the control task.
- Everything else identical: BNO055, drivetrain, balance controller,
  OtaSafeMode, ArmingState, BalanceTuner.

New `platformio.ini` envs: `[env:balance_test]` and `[env:balance_test_ota]`.

## Tests

### `test/test_balancing_controller/test_balancing_controller.cpp`

Native, mock IMU and mock drivetrain.

| Case                                                                  | Assert                                          |
| --------------------------------------------------------------------- | ----------------------------------------------- |
| Zero command, level platform                                          | Emits zero `BodyVelocity`                       |
| Zero command, platform tilted forward                                 | Emits `vx_out` with sign correcting toward level |
| Zero command, platform tilted left                                    | Emits `vy_out` with sign correcting toward level |
| Forward command, level                                                | Nonzero `vx_out`; `vy_out = 0`; `omega_out = 0`  |
| Forward command, platform already at `pitch_target`                   | `vx_out → 0` (error neared zero)                 |
| Enter fault at `envelopeEnterSin`                                     | Emits zero; balance PIDs reset (verify via a pre-loaded integral) |
| In fault, `tiltMagSin` between exit and enter                         | Stays in fault                                   |
| Exit fault at `envelopeExitSin`                                       | Resumes normal operation                         |
| `quatToBodyGravity` over grid of `(pitch, roll)`                      | Matches analytic ground truth                    |
| Output clamp at `maxOutputVelocity`                                   | Output saturates correctly, no overflow          |
| `stop()`                                                              | Calls `drivetrain.stop()`; resets balance PID state |

### `test/test_pid/` — extend existing

| Case                                                                  | Assert                                          |
| --------------------------------------------------------------------- | ----------------------------------------------- |
| 4-arg `compute` with rate = finite-diff                               | Equals 3-arg `compute` result                    |
| 4-arg `compute` with externally supplied rate                         | Uses supplied rate, ignores prev-measurement     |
| 4-arg version respects anti-windup, deadband, clamping                | Same behaviour as 3-arg                          |

### `test/test_balance_config/` (new)

| Case                                                                  | Assert                                          |
| --------------------------------------------------------------------- | ----------------------------------------------- |
| Valid `balance set <key> <value>`                                     | Slot pointer updates; subsequent `show` reflects |
| Invalid value (negative kp, etc.)                                     | Rejected with error string; slot unchanged       |
| Persistence round-trip                                                | Save → reload → equal                            |
| Boot with missing/corrupt NVS blob                                    | Falls back to defaults                           |

### `test/test_arming_state/` (new)

| Case                                                                  | Assert                                          |
| --------------------------------------------------------------------- | ----------------------------------------------- |
| Boot state                                                            | Disarmed                                         |
| Disarmed → Armed via `arm()`                                          | get() == Armed                                   |
| Armed → Disarmed via `disarm()`                                       | get() == Disarmed                                |
| Any → Killed via `kill()`                                             | get() == Killed                                  |
| `arm()` while Killed                                                  | No-op; remains Killed                            |
| `clearKill()` from non-Killed                                         | No-op                                            |
| `clearKill()` from Killed                                             | get() == Disarmed                                |

### Hardware verification (bench)

Run `main_balance_test`, observe over Serial / RemoteSerial:

1. `arm`. Place robot on soft surface.
2. Hand-tilt platform forward 10° → wheels drive backward to restore level.
3. Hand-tilt platform left 10° → wheels drive right.
4. `cmd 0.5 0 0` → platform tilts forward; sphere rolls forward.
5. `cmd 0 0 0` → platform returns to level; sphere decelerates.
6. Tilt past 60° → drive halts; fault transition logs. Return below 55° →
   drive resumes.
7. `kill` → drive halts immediately. State = Killed.
8. `clearkill` then `arm` → drive resumes.

## Tuning guide

### Procedure (do in order)

1. **P only.** Ki = Kd = 0. Robot on soft surface. Increase `pitchKp` (and
   matching `rollKp`) until restoring action is clear, with light overshoot.
2. **Add D.** Increase `pitchKd` until overshoot is damped without
   high-frequency wobble. (Wobble = D too high, amplifying gyro noise.)
3. **Iterate P/D.** With damping, P can usually go higher. Repeat 1–2.
4. **I, sparingly.** Only if the robot settles slightly off-vertical. Start at
   ~5 % of Kp; increase only if needed.
5. **`tiltPerVelocity` last.** Subjective: full stick = full attack
   acceleration without exceeding `maxTiltSetpoint`. Tune by feel.

### Symptoms → adjustments

| Observation                                              | Likely cause                              | Adjust                                          |
| -------------------------------------------------------- | ----------------------------------------- | ----------------------------------------------- |
| Mushy / slow response to commands                        | Kp too low                                | ↑ `pitchKp` / `rollKp`                          |
| Slow oscillation (~1–2 Hz) around upright                | Kp too high or Kd too low                 | ↓ Kp or ↑ Kd                                    |
| High-frequency motor buzz                                | Kd too high (amplifying gyro noise)       | ↓ Kd                                            |
| Persistent off-vertical lean (steady-state error)        | Ki too low                                | ↑ Ki (start at ~5% of Kp)                       |
| Drifts away over time / wind-up                          | Ki too high or anti-windup not engaging   | ↓ Ki; verify anti-windup integral clamp         |
| Weak feel even at full stick                             | `tiltPerVelocity` low, or output clamping | ↑ `tiltPerVelocity`; check `maxOutputVelocity`  |
| Alarming lean on full stick                              | `maxTiltSetpoint` too high                | ↓ `maxTiltSetpoint`                             |
| Fault trips during normal aggressive driving             | Envelope too tight (or real problem)      | First check the physics — shouldn't see 60° normally. If real, ↑ `envelopeEnterSin` slightly. |
| Fault chatters at the edge                               | Hysteresis gap too small                  | Widen the gap between enter and exit            |
| Kick when recovering from fault                          | Balance PIDs not reset on entry           | Bug — verify enterFault() calls pid.reset()     |
| Drifts sideways while driving forward                    | Pitch/roll PID asymmetry, or mechanical   | Try `rollKp` ≠ `pitchKp` slightly; if persists, suspect hardware |

## What this spec does NOT cover

- Webapp UI implementation (button styling, event wiring).
- Exact wire format for `CommandFrameParser` — coordinated with the existing
  parser convention.
- Final PID tuning values — placeholders only; bench-tuned in operation.
- Position-hold / ground odometry — controller is tilt-only.
- Yaw-rate closed loop / heading hold — passthrough only.
- Outer feedforward term — explicitly excluded; can be added later as a
  one-line addition with `ff_gain = 0` default.
- IMU calibration drift handling beyond a log warning.
