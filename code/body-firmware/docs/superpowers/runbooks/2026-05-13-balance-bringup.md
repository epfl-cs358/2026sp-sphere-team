# Balancing drivetrain controller — bench bring-up runbook

**Date:** 2026-05-13
**Audience:** operator on the bench during first power-on of the balancing controller.

This runbook is the operator's reference during bench bring-up. It is kept
separate from the design spec
(`docs/superpowers/specs/2026-05-13-balancing-drivetrain-controller-design.md`)
so that bench notes, gotchas, and field-tuned values can be appended here as
they accumulate without churning the design document. Update freely.

The harness used throughout is `main_balance_test` (PlatformIO env
`balance_test`). All `balance ...` commands below are sent over RemoteSerial.

---

## 1. Bench-verification sequence

Run `main_balance_test`. Observe Serial / RemoteSerial output. Execute in
order:

1. `arm`. Place robot on soft surface.
2. Hand-tilt platform forward 10° → wheels drive backward to restore level.
3. Hand-tilt platform left 10° → wheels drive right.
4. `cmd 0.5 0 0` → platform tilts forward; sphere rolls forward.
5. `cmd 0 0 0` → platform returns to level; sphere decelerates.
6. Tilt past 60° → drive halts; fault transition logs. Return below 55° →
   drive resumes.
7. `kill` → drive halts immediately. State = Killed.
8. `clearkill` then `arm` → drive resumes.

If any step fails, jump to the [Axis diagnosis](#3-axis-diagnosis) section
before re-tuning gains.

---

## 2. Sign-convention check (`gyroPitchSign` / `gyroRollSign`)

`gyroPitchSign` and `gyroRollSign` default to `+1` in `BalanceConfig` and are
verified at bring-up with the following sub-procedure. Both signs are
configurable via RemoteSerial without reflash if bring-up reveals an inverted
axis.

1. Place platform level. Confirm `gx ≈ 0`, `gy ≈ 0`, `gz ≈ -1`.
2. Tilt platform forward slowly. Confirm `pitch_actual` increases monotonically
   and `imu.gyro.y` is non-zero with consistent sign during the tilt.
3. Numerically integrate `imu.gyro.y` over the motion and compare to the change
   in `pitch_actual`. If they have opposite signs, set
   `gyroPitchSign = -1` via:

   ```
   balance set gyroPitchSign -1.0
   balance save
   ```

4. Repeat steps 2–3 for roll, using `imu.gyro.x` and `gyroRollSign`.

The signs live as `float` knobs in `BalanceConfig`, so a flip is live and
persistent (after `balance save`) — no reflash needed.

---

## 3. Axis diagnosis

> Tilt the platform forward by hand and watch the wheels.
>
> - **Wheels drive in the wrong direction on forward tilt** (sign-inverted
>   axis) → set `gyroPitchSign = -1.0` via
>   `balance set gyroPitchSign -1.0`. Live, no reflash.
> - **Wheels drive *sideways* on forward tilt** (rotated mount: chip-Y points
>   along body-X, etc.) → the two `BalanceConfig` sign knobs cannot fix this.
>   Either physically remount the BNO055 so chip-X aligns with body-forward,
>   or call `_bno.setAxisRemap(...)` in `BNO055IMU::begin()` and reflash.
>   Re-run the bring-up procedure after.
>
> Repeat the diagnosis for roll on body-X.

The distinction matters: the two `BalanceConfig` sign knobs are
one-dimensional; they cannot rotate a coordinate frame. If chip and body axes
are *rotated* relative to each other (not just sign-inverted), the fix is
mechanical or in IMU driver setup — not in `BalanceConfig`.

---

## 4. Tuning guide

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

---

## 5. Persistence smoke test

Confirm NVS-backed `BalanceConfig` round-trips through a reboot:

```
balance set pitchKp 2.0
balance save
# power-cycle the board (or trigger a software reset)
balance show pids
```

**Assert:** `pitchKp == 2.0` in the `balance show pids` output. If the value
reverts to the default, NVS save/load is broken — do not proceed with bench
verification.

---

## 6. Configuration footgun list

- **`maxTiltSetpoint` must stay strictly below `asin(envelopeEnterSin)`.** The
  setpoint must never command a tilt that would itself trip the fault
  envelope. The tuner enforces this invariant on every `balance set`. If the
  operator sees a `rejected` message on a `set` command, this is almost
  certainly the invariant they violated — either lower `maxTiltSetpoint`, or
  raise `envelopeEnterSin` first and then retry the `maxTiltSetpoint` change.
- The envelope hysteresis pair must satisfy
  `envelopeExitSin < envelopeEnterSin`. The tuner rejects updates that would
  invert the gap. Widen the gap if the fault chatters at the edge.
- All PID gains (`pitchKp`, `pitchKi`, `pitchKd`, `rollKp`, `rollKi`,
  `rollKd`) must be `≥ 0`. Negative values are rejected — sign convention is
  expressed exclusively through `gyroPitchSign` / `gyroRollSign`.
