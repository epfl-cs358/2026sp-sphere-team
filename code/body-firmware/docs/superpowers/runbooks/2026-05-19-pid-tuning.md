# PID Tuning — 2026-05-19

Bench-operator reference for tuning the balance controller's pitch/roll PIDs on the BB-8 sphere. Seven phases. Phases 1–2 are background you read once; phases 3–6 are the procedure you run at the bench; phase 7 is the wire-protocol reference card.

Each procedural step shows **Expected** behavior and an **Observed** blank for the operator to fill in. Conceptual sections are plain prose.

Cross-reference: do the bringup runbook (`2026-05-19-balance-bench-bringup.md`) first. This document assumes Phases A–E of bringup have already passed (OTA, sign-direction, kinematics convention, re-arm windup, fault envelope) — tuning gains on top of an unresolved sign error is unproductive.

---

## 1. Context

The balance controller's job is to keep the sphere upright by leaning into commanded velocity. There are two independent balance loops:

- **Pitch loop** (fore-aft tilt) → drives the body-frame `vx` output.
- **Roll loop** (left-right tilt) → drives the body-frame `vy` output.

Yaw (`omega`) is **cascaded** as of Commit 3+4: an inner gyro-Z rate PID closes the loop on yaw rate, and an outer heading-hold P loop captures the integrated heading when the stick is in the deadband and feeds a (clamped, LP-filtered) rate setpoint to the inner loop. Yaw can be reverted to pre-Commit-3 passthrough by zeroing both `yawRateKp` and `yawRateKi`. Phase F of the bringup runbook brings up both loops.

### Sphere sign convention (not Segway)

On a Segway, the wheels drive **toward** the lean to push the platform back upright. On the BB-8 sphere the geometry is inverted: the body sits inside the shell, and a forward lean of the body requires the wheels to push the **shell** in the opposite direction so the body's center of mass swings back over the contact patch. The recent fix in `src/drivetrain/OmniKinematics.h` (vx coefficient `-sin` → `+sin`) is what makes the `+vx` body-frame command produce a forward roll of the shell. The Phase B sign check in the bringup runbook locks this down.

### Safety envelope and arming

- The controller has a **Schmitt-triggered fault gate**. If the tilt magnitude (in `sin` space) exceeds `envelopeEnterSin` (default `sin 60° ≈ 0.866`), the controller latches a fault, resets both balance PIDs, and commands zero velocity until tilt comes back below `envelopeExitSin` (default `sin 55° ≈ 0.819`). The 5° gap is hysteresis — without it the gate would chatter at the threshold.
- `arm` / `disarm` / `kill` / `clearkill` / `armstate` verbs control the arming FSM. PIDs are reset on both arming edges (Disarmed→Armed and Armed→Disarmed) so accumulated I/D state cannot kick the wheels on re-engage.
- OTA upload force-disarms before flashing.
- The producer-silence auto-disarm killswitch has been **removed**. Armed state persists until an explicit verb, OTA, or fault trip. Short-term silence (<200 ms) still ramps `cmd` to zero via `STALENESS_TIMEOUT_MS` but does not disarm.
- The `kill` verb hard-brakes and latches Killed; only `clearkill` releases.

### Where to run this

**Robot must be ON THE GROUND or in a wheel-loaded cradle.** Free-spinning wheels hide both the windup jolt and the balance D-term effects — a robot that "looks fine on the bench stand" can be wildly mis-tuned on the ground. The fault envelope (60°/55°) is your friend during tuning: a robot that's about to tip will gate itself off rather than fall.

---

## 2. Parameter glossary

The full set is defined in `src/drivetrain/BalanceConfig.h`. Compiled defaults live in `RobotConstants::balanceConfig()` (`src/drivetrain/RobotConstants.h`). Live values are settable over WebSerial via `balance set <key> <value>` and persisted to NVS with `balance save`.

For each parameter: units, what it physically does, current default, and the observable symptoms of "too low" / "too high".

### `tiltPerVelocity` — command-to-lean gain

- **Units:** rad per (m/s).
- **Default:** `0.30` (1 m/s commanded → ~17° lean setpoint).
- **What it does:** Maps the operator's velocity command (`cmd.vx`, `cmd.vy`) into a target lean angle. The balance loop then drives wheels to achieve that lean. This is the *forward path*: high `tiltPerVelocity` means "aggressively lean for fast travel"; low means "stay near upright even when commanded fast".
- **Too low:** Robot won't move much even at full stick — the controller barely leans, so it barely accelerates.
- **Too high:** Robot tips trying to achieve the requested speed — the commanded lean exceeds the controller's catch authority. Caps at `maxTiltSetpoint` (next param) before that happens.

### `maxTiltSetpoint` — hard cap on commanded lean

- **Units:** rad.
- **Default:** `0.35` (~20°).
- **What it does:** Clips the output of `tiltPerVelocity * cmd.v` so the *requested* lean never exceeds this value, regardless of stick input. Keeps the commanded lean comfortably below the 60° fault envelope.
- **Tuner invariant:** `BalanceTuner::_set` rejects values `>= asin(envelopeEnterSin)`. You cannot set this to a value that would itself trip the fault gate.
- **Too low:** Maximum forward speed is throttled — you can't ask for an aggressive lean even if the robot could handle it.
- **Too high:** A burst of stick input requests a near-tipover lean; the controller is asked to balance at a setpoint that leaves no recovery margin.

### `pitchKp` — proportional restoring force (fore-aft)

- **Units:** m/s output per rad of pitch error.
- **Default:** `1.50`.
- **What it does:** This is the "spring". When the robot is leaning forward by `+X` rad (pitch_actual > pitch_target), the error is negative, so the PID emits `Kp * error` m/s of `vx` — wheels roll **backward**, which pushes the shell back so the body falls back toward upright. Stiffer Kp = stronger restoring force per unit of error.
- **Too low:** Robot tips and falls — the wheels barely move when tilted, no restoring force.
- **Too high:** Robot oscillates fore-aft. The wheels overshoot every correction; you see a clear ringing at the system's natural frequency (typically 1–5 Hz on a sphere this size).

### `pitchKi` — integrated pitch error

- **Units:** m/s output per (rad·s) of accumulated pitch error.
- **Default:** `0.0`.
- **What it does:** Sums error over time and pushes the output to eliminate any persistent offset. For a balance robot with a direct tilt measurement from the IMU, Kp alone usually gets you to zero steady-state — there's no sensor drift to integrate away. Useful only if there's a real physical bias (e.g. weight unevenly placed, wheel friction asymmetry) that Kp's proportional law can't fully cancel.
- **Anti-windup:** `PID::compute` clamps the integral to `[outputMin / Ki, outputMax / Ki]` so it cannot saturate the output indefinitely. The clamp prevents *runaway*, not lag — a wound-up integral will still take time to bleed off once the disturbance is gone.
- **Too low (e.g. 0):** Robot lists persistently in one direction at a small angle (1–3°). Kp wants to push but the proportional output equals the disturbance and equilibrium settles offset from upright.
- **Too high:** Slow-period oscillation (longer than Kp ringing), or a "leans further and further in one direction" before the anti-windup clamp kicks in and the controller eventually re-centers. Often trips the fault envelope.

### `pitchKd` — pitch rate damping

- **Units:** m/s output per (rad/s) of pitch rate.
- **Default:** `0.15`.
- **What it does:** Resists *angular velocity*. The PID computes `Kd * (-rate)` so a positive pitch rate (falling forward) produces a negative `vx` term, pushing back.
- **Critical implementation detail:** Read `BalancingDrivetrainController.cpp` lines 76–80. The D-term is fed the **raw gyro reading** (`cfg.gyroPitchSign * imuData.gyro.y`), **not** the numerical derivative of the pitch measurement. This matters because:
    - Gyro rate is a direct, low-noise hardware measurement. Numerical d/dt of pitch would amplify IMU quantization noise.
    - There's no `Kd * d(error)/dt` term that fights the setpoint when it changes — only the measurement's velocity is damped. Setpoint changes are smooth ramps, not derivative kicks.
    - **Kd here behaves like a tachometer brake.** "How fast am I rotating?" → directly proportional opposing wheel command. Tune it that way.
- **Too low:** Robot oscillates after each Kp correction — the spring rings without enough viscous damping.
- **Too high:** Wheels chatter / whine at IMU noise frequency (you can hear it as a high-pitched buzz from the motors). The D-term is amplifying gyro hash directly into wheel command.

### `rollKp`, `rollKi`, `rollKd` — same, for left-right axis

- **Defaults:** `1.50 / 0.0 / 0.15` — identical to pitch.
- **Why separate from pitch:** Mechanical asymmetries (wheel placement, mass distribution, friction) can make the roll axis subtly different from pitch. They're tuned independently so you can match each axis to its own dynamics. In practice, on a near-symmetric sphere, the roll gains usually end up within ~20% of the pitch gains.

### `maxOutputVelocity` — PID output clamp

- **Units:** m/s.
- **Default:** `1.00` (physical drivetrain max is ~1.05 m/s).
- **What it does:** Clips `|vx_out|` and `|vy_out|` after the PID computes them. Applied both inside `PID::compute` and as a redundant safety clamp in the balance controller. Also sets the anti-windup integral cap (`integralMax = outputMax / Ki`).
- **Too low:** Controller saturates during large tilts and can't catch them. Visible as "robot starts tipping faster than the wheels can spin up to counter it".
- **Too high:** Wheel command exceeds physical capability. The omni kinematics layer then has to clamp per-wheel commands unevenly (one wheel saturates before the others), distorting the commanded body velocity.

### `envelopeEnterSin` / `envelopeExitSin` — fault gate

- **Units:** dimensionless (`sin` of the tilt angle).
- **Defaults:** `0.866` (`sin 60°`) / `0.819` (`sin 55°`).
- **What they do:** Schmitt trigger on tilt magnitude. Enter the fault state when `|tilt| > envelopeEnterSin`; exit only when `|tilt| <= envelopeExitSin`. In fault, the controller commands zero velocity and resets both balance PIDs.
- **Why `sin` not radians:** Compares the body-gravity vector's horizontal projection magnitude directly — no `asin` needed every tick.
- **Tuner invariant:** `_set` enforces `envelopeExitSin < envelopeEnterSin`. The gap (5° default) prevents flapping.
- **Don't reduce the gap below ~3°** — small hysteresis windows flap on IMU noise alone.

### `gyroPitchSign`, `gyroRollSign` — IMU axis sign override

- **Units:** dimensionless (±1).
- **Defaults:** `+1.0` each.
- **What they do:** Multiplies the raw gyro reading before it's fed into the PID's D-term. If the BNO055's axis convention disagrees with our pitch/roll sign assumption, the D-term will fight the wrong direction — instead of damping motion, it *adds* to it. That's positive feedback. The robot will tip away from upright on contact, or oscillate divergently from the moment it's armed.
- **How to identify a wrong sign:** Phase B of the bringup runbook is the canonical procedure. If during gain-tuning Phase A (below) the robot "doubles down" — tilts forward and the wheels also drive forward, making it tip faster — flip the corresponding sign to `-1`.

### `yawRateKp`, `yawRateKi` — inner gyro-Z rate PID

- **Units:** dimensionless (output is rad/s; input is rad/s).
- **Defaults:** `0.0` both. **Ship-safe**: with both zero, the inner PID is bypassed and `cmd.omega` flows straight through (pre-Commit-3 passthrough behavior).
- **What they do:** Close the loop on yaw rate. Setpoint is the outer-loop output (heading hold) or `cmd.omega` directly (passthrough). Measurement is `gyroYawSign * gyro.z`. Output is the body-frame `omega` written to the drivetrain, clamped to `±OMEGA_MAX` (3.5 rad/s ≈ 200 dps).
- **Anti-windup:** `PID::compute` clamps the integral to `[outputMin/Ki, outputMax/Ki]` (same as pitch/roll). Saturation bits surface as `kEvent_YAW_RATE_I_SATURATED` and `kEvent_YAW_RATE_OUT_SATURATED`.
- **Too low:** Bot doesn't follow yaw stick well; manual perturbations not damped (loop is effectively open).
- **Too high:** Yaw chatter / motor whine, or the inner loop saturates and the wheels jitter at the saturation boundary.
- **No `Kd`**: dropped at `c551fb2`. The measurement is already a rate; a derivative-of-rate term amplifies gyro noise without adding control authority.

### `headingKp` — outer heading-hold P gain

- **Units:** dimensionless (output is rad/s of rate-setpoint; input is rad of heading error).
- **Default:** `1.0` (engaged by default). Set to `0` to disable heading hold while leaving the inner yaw-rate loop active.
- **What it does:** When `|cmd.omega| < HEADING_DEADBAND` (0.05 rad/s), the controller snapshots the current `∫gyro.z` as the held setpoint and emits `headingKp * (setpoint - integrator)` as the rate command into the inner loop. The output is clamped to `±OMEGA_MAX` (no integral term means no windup to worry about). The rate command is low-pass filtered with τ = 0.15 s before reaching the inner PID — this prevents a snap on stick release.
- **Latching behavior**: stick out of deadband invalidates the held setpoint (telemetry shows `heading_setpoint = NaN`); returning to deadband re-latches at the *current* integrator value (so "wherever the operator was pointing at the moment of release" becomes the new hold). The event `kEvent_HEADING_LATCHED` fires on each falling edge.
- **Tilt-fault behavior**: `_headingIntegrator` is **not** zeroed on fault entry — it must keep tracking truth so that on fault exit the held heading is still accurate. The LP filter `_omegaTargetFiltered` is reset on fault entry so it restarts clean.
- **Too low:** Bot drifts under disturbance; pushed by hand → doesn't quite recover original heading.
- **Too high:** Bot oscillates around the held heading, or yaw command saturates on small disturbances and the wheels grind at the clamp.

### `gyroYawSign` — yaw-axis IMU sign override

- **Units:** dimensionless (±1).
- **Default:** `+1.0`.
- **What it does:** Multiplies `gyro.z` before it feeds both the inner yaw-rate PID and the heading integrator. Wrong sign = positive feedback on yaw (bot spins faster instead of correcting). Identified the same way as `gyroPitchSign`/`gyroRollSign` — Phase F1 of the bringup runbook.

---

## 3. Pre-flight checks

Before touching any gains, verify the bench is in a known good state. **All Expected rows must match before proceeding to Phase 4.**

### 3a. Choose your env

| Env | When to use | Input path |
|---|---|---|
| `balance_test_ota` | Direct velocity injection over WebSerial (`cmd <vx> <vy> <omega>`) — fastest for gain sweeps where you want to fire test commands without the webapp loop. | WebSerial `cmd` verb |
| `robot_ota` | Production teleop path. Velocity comes from the webapp's WebSocket producer on port 80. Use for final validation. | webapp `/control` |

For this runbook, use **`balance_test_ota`** for Phases 4A–4D (fast iteration), then re-flash to `robot_ota` for end-to-end verification.

```sh
pio run -e balance_test_ota -t upload
```

### 3b. Connect WebSerial

**Full WebSerial at `http://bb8-robot.local:81/webserial`. NOT WebSerial Lite.** Lite buffers and replays input on reconnect, which surfaces as duplicate `ok:` and spurious `error: unknown balance verb` lines mid-session.

### 3c. Verify IMU and arming state

| # | Action | Expected | Observed |
|---|--------|----------|----------|
| 1 | Power on the robot, watch boot log | `[imu] offsets restored from NVS` then `[cal] warm boot — gyro settled, ready` within ~2 s | `[   ]` |
| 2 | If you see a *cold-boot* IMU path instead | Run the 6-orientation accel dance per the bringup runbook IMU note; do not tune on an uncalibrated IMU | `[   ]` |
| 3 | Send `balance show` | All 13 fields print; values match `RobotConstants::balanceConfig()` | `[   ]` |
| 4 | If they don't match | `balance reset` then `balance save` to restore compiled defaults to NVS | `[   ]` |
| 5 | Send `armstate` | `[arming] Disarmed` | `[   ]` |

Gate: every Observed must match Expected. If the IMU is cold or `balance show` returns surprise values, fix that before tuning gains.

---

## 4. Gain tuning procedure

Tune **pitch first, in order Kp → Kd → Ki**. Then repeat the same order for roll. Pitch is usually the dominant failure mode — if you can't stabilize fore-aft, you can't usefully observe left-right dynamics.

This is the Ziegler-Nichols-by-feel approach: drive Kp up until the system rings (this is your `Ku`, ultimate gain, with period `Tu`), then back off and add damping.

### Phase 4A — find `pitchKp` upper bound

Isolate Kp by zeroing Ki and Kd. Robot must be **on the ground or in a wheel-loaded cradle**.

```sh
balance set pitchKi 0
balance set pitchKd 0
balance set rollKp 0
balance set rollKi 0
balance set rollKd 0
balance set pitchKp 0.5
```

| # | Action | Expected | Observed |
|---|--------|----------|----------|
| 1 | `arm` | `[arming] -> Armed`. Robot likely falls over (Kp=0.5 is below working range). Catch it. | `[   ]` |
| 2 | `disarm`, double Kp: `balance set pitchKp 1.0`, re-arm | Repeat. Note any sign of restoring motion (wheels push against the lean). | `[   ]` |
| 3 | Continue doubling: 2.0, 4.0, 8.0… | At some Kp the robot oscillates fore-aft at a clear period. This is `Ku`. Time the period `Tu` in seconds with a stopwatch (or count cycles per 5 s). | Ku = `[   ]`, Tu = `[   ]` s |
| 4 | `disarm` | `[arming] -> Disarmed` | `[   ]` |
| 5 | Compute working Kp ≈ 0.5 · Ku (P-only) or 0.6 · Ku (PD target) | Set `balance set pitchKp <value>`. | Working pitchKp = `[   ]` |

**Failure modes during Phase 4A:**

- *Robot keeps falling even at Kp = 4 or higher* → suspect wrong gyro sign. The controller is generating positive feedback. **Disarm**, flip the sign: `balance set gyroPitchSign -1` (then `balance save` if you want to persist it). Re-run Phase 4A from Kp = 0.5. If after a sign flip the robot now stabilizes at a reasonable Kp, the original IMU axis assumption was inverted on this hardware; record this in Findings.
- *Wheels chatter at Kp = 0.1, before any tilt* → mechanical issue (loose wheel, bad encoder, motor wiring). Not a tuning problem. Check the drivetrain before continuing.
- *Robot tilts but wheels drive in the same direction* → kinematics sign error (the OmniKinematics.h vx fix didn't land, or roll is involved). Cross-reference Phase C of the bringup runbook. Don't try to fix this with Kp.

### Phase 4B — add `pitchKd` to damp the ringing

With `pitchKp` set from Phase 4A:

```sh
balance set pitchKd 0.05
```

| # | Action | Expected | Observed |
|---|--------|----------|----------|
| 1 | `arm`, observe robot at rest | Mild residual oscillation, but smaller amplitude than Kp-only. | `[   ]` |
| 2 | If still ringing: `disarm`, raise to 0.10, 0.20, 0.30… re-arm each time | Each step damps faster. Goal: oscillations die in 1–2 cycles. | `[   ]` |
| 3 | Disturb the robot by hand (light push fore-aft) | Robot returns to upright in 1–2 swings; no sustained ringing. | Working pitchKd = `[   ]` |
| 4 | If you hear a high-pitched whine from motors | Kd is too high — D-term is amplifying gyro noise. Lower Kd until the whine stops. | `[   ]` |
| 5 | `disarm` | `[arming] -> Disarmed` | `[   ]` |

**Heuristic:** for the default Kp ≈ 1.5 region, Kd typically lands somewhere in `[0.10, 0.30]`. Outside that range, sanity-check the IMU and gyro sign.

### Phase 4C — add `pitchKi` only if needed

For most balance setups, **leave Ki at 0**. Direct tilt measurement makes integral action mostly unnecessary.

The symptom that *does* call for Ki: with Kp and Kd tuned, the robot consistently sits leaning ~1–3° in one direction. The proportional restoring force is exactly cancelled by some physical disturbance (uneven mass, motor friction, wheel asymmetry). Only Ki can drive that residual to zero.

```sh
balance set pitchKi 0.05
```

| # | Action | Expected | Observed |
|---|--------|----------|----------|
| 1 | `arm`, wait 5 s with no command input | Standing tilt converges to ~0° (no persistent lean). | `[   ]` |
| 2 | If still leaning after 10 s | Raise to 0.10. Do not exceed 0.20 — at that point you're masking a mechanical issue. | `[   ]` |
| 3 | If robot starts slow-period drifting/oscillating | Ki is too high. Halve it. | `[   ]` |
| 4 | `disarm` | `[arming] -> Disarmed` | `[   ]` |

### Phase 4D — repeat for the roll axis

Restore roll Kp/Kd to their tuned-pitch values as a starting point (the robot is nearly symmetric):

```sh
balance set rollKp <pitchKp>
balance set rollKd <pitchKd>
balance set rollKi 0
```

Re-run Phases 4A–4C, but tilt the robot **left-right** by hand instead of fore-aft. Expect rollKp/Kd within ~20% of the pitch values.

| # | Action | Expected | Observed |
|---|--------|----------|----------|
| 1 | Find `rollKp` upper bound (4A-style sweep) | rollKu ≈ pitchKu within ~20% | Working rollKp = `[   ]` |
| 2 | Damp with `rollKd` | 1–2 cycle settling under hand disturbance | Working rollKd = `[   ]` |
| 3 | Add `rollKi` only if persistent list | Standing tilt converges to 0° in roll axis | Working rollKi = `[   ]` |
| 4 | `disarm` | `[arming] -> Disarmed` | `[   ]` |

### Phase 4E — sign-direction sanity check (re-affirm)

Before declaring the tune done, run a quick sign confirmation. This is the existing Phase B of the bringup runbook condensed.

| # | Action | Expected | Observed |
|---|--------|----------|----------|
| 1 | `arm`, tilt the robot forward by hand a few degrees | Wheels roll the shell **backward** (sphere model). | `[   ]` |
| 2 | Tilt left-side-down | Shell rolls to the **right** (sphere model). | `[   ]` |
| 3 | `disarm` | `[arming] -> Disarmed` | `[   ]` |

If either row inverts, the gyro sign for that axis is wrong **or** the kinematics fix didn't land. Stop tuning and resolve sign first — gains tuned over the wrong sign are garbage.

---

## 5. Persistence

Tuned gains live in two places:

- **In-RAM live config:** updated immediately on every `balance set` via the two-buffer atomic swap (`BalanceTuner::_publish`). The controller picks up the new pointer on the next tick (one `CONTROL_PERIOD_MS` of latency).
- **NVS-persisted config:** survives reboots. Written by `balance save`, restored by `BalanceConfigStorage::load` at boot (key `"balance"` in namespace `bal_cfg`).

### Saving the tune

```sh
balance show
balance save
```

`balance show` prints all 13 fields so you can sanity-check before committing. `balance save` writes the current live config to NVS — survives reboot, survives reflash (until someone wipes NVS).

### Reverting to compiled defaults

```sh
balance reset
balance save
```

`balance reset` overwrites the live config with `RobotConstants::balanceConfig()` but **does not save to NVS automatically**. If you reboot after `balance reset` without `balance save`, the previously-saved NVS values come back. Pair them.

### Reading current saved state

```sh
balance show          # all 13 fields
balance show pids     # Kp/Ki/Kd for pitch and roll only
```

---

## 6. Troubleshooting

| Symptom | Likely Cause | Try |
|---|---|---|
| Robot falls on `arm`, wheels barely move | `pitchKp` too low | Raise pitchKp by 2× until restoring motion appears |
| Robot oscillates fore-aft, doesn't settle | `pitchKp` too high, `pitchKd` too low | Lower pitchKp 20%; raise pitchKd in 0.05 steps |
| High-pitched whine from motors at rest | `pitchKd` or `rollKd` too high (amplifying IMU noise) | Lower Kd until whine stops |
| Persistent ~1–3° lean in one direction at rest | Physical imbalance or gyro bias | Try Ki = 0.05; if no improvement, recalibrate IMU (6-face accel dance) |
| Robot doubles down on lean (tips faster when displaced) | Wrong `gyroPitchSign`/`gyroRollSign`, or kinematics vx fix not in firmware | Flip `gyroPitchSign` (or `gyroRollSign`) to `-1`; verify OmniKinematics.h fix landed |
| Robot drives forward then suddenly stops | OTA disarm hook fired, OR fault envelope tripped at 60° | Send `armstate`; if `Killed`, run `clearkill`; if `Disarmed`, re-arm |
| `balance set` doesn't take effect | Tuner returned an error you missed | Run `balance show` to inspect; check for `error:` lines in WebSerial scrollback |
| `balance set maxTiltSetpoint 0.9` rejected | Tuner invariant: `maxTiltSetpoint < asin(envelopeEnterSin)` | Either lower the value or raise `envelopeEnterSin` first |
| `balance set envelopeExitSin 0.9` rejected | Tuner invariant: `envelopeExitSin < envelopeEnterSin` | Lower the value or raise `envelopeEnterSin` first |
| Robot oscillates with a long period (>2 s) | `pitchKi` too high — integrator overshoot | Halve pitchKi, or set back to 0 |
| Wheels jitter only when the robot is moving fast | `maxOutputVelocity` set higher than physical max → kinematics saturating one wheel at a time | Drop `maxOutputVelocity` to 1.0 or below |
| Tune was good yesterday, bad today | NVS may have been wiped (firmware reflash with chip-erase), or someone ran `balance reset` without `save` | `balance show`, compare to your recorded Findings, re-`balance set` + `balance save` |

---

## 7. Reference — wire protocol

All verbs are sent as plain text lines over WebSerial at `http://bb8-robot.local:81/webserial`. Responses come back as `ok: ...` or `error: ...` lines.

### Balance tuner verbs

| Verb | Effect |
|---|---|
| `balance show` | Print all 13 BalanceConfig fields with current live values |
| `balance show pids` | Print only the 6 PID gain fields (pitchKp/Ki/Kd, rollKp/Ki/Kd) |
| `balance set <key> <value>` | Set a single field; validates and hot-swaps via two-buffer atomic |
| `balance reset` | Overwrite live config with compiled defaults (`RobotConstants::balanceConfig()`); does not auto-save |
| `balance save` | Write current live config to NVS (key `balance`, namespace `bal_cfg`) |
| `balance status` | Print current arming state and placeholder pitch/roll (placeholders pending tuner ↔ controller wire-in) |

### Settable keys (for `balance set`)

`tiltPerVelocity`, `maxTiltSetpoint`, `pitchKp`, `pitchKi`, `pitchKd`, `rollKp`, `rollKi`, `rollKd`, `maxOutputVelocity`, `envelopeEnterSin`, `envelopeExitSin`, `gyroPitchSign`, `gyroRollSign`, `yawRateKp`, `yawRateKi`, `headingKp`, `gyroYawSign`.

### Tuner-enforced invariants

- All gain fields (Kp/Ki/Kd) must be `>= 0`.
- `maxOutputVelocity >= 0`.
- `envelopeExitSin < envelopeEnterSin`.
- `maxTiltSetpoint < asin(envelopeEnterSin)`.

Violations emit `error: ...` and leave the live config unchanged.

### Arming verbs

| Verb | Effect |
|---|---|
| `arm` | FSM: → Armed (no-op if already Armed). Resets wheel and balance PIDs on the Disarmed→Armed edge. |
| `disarm` | FSM: → Disarmed (no-op if already Disarmed). Resets wheel and balance PIDs on the Armed→Disarmed edge. |
| `kill` | FSM: → Killed (latched hard-brake). Only `clearkill` releases. |
| `clearkill` | FSM: Killed → Disarmed. |
| `armstate` | Print current arming state. |

### Convenience verbs (env-dependent)

| Verb | Env | Effect |
|---|---|---|
| `cmd <vx> <vy> <omega>` | `balance_test_ota` only | Inject a body-frame velocity command directly (bypasses webapp WebSocket) |
| `stop` | `drivetrain_test_ota` | Brake all wheels |

### Related runbook

- `code/body-firmware/docs/superpowers/runbooks/2026-05-19-balance-bench-bringup.md` — bringup gating (OTA, sign-direction, kinematics, re-arm windup, fault envelope). Run that first. This document picks up after Phase E of bringup.

---

## Findings

Use this section to record the final tuned values and any per-robot observations. Persist these somewhere outside the firmware too — NVS can be wiped.

### Tuned gains

| Parameter | Value | Notes |
|---|---|---|
| `tiltPerVelocity` | `[       ]` | Default 0.30; raise if too sluggish, lower if too aggressive |
| `maxTiltSetpoint` | `[       ]` | Default 0.35 |
| `pitchKp` | `[       ]` | Ku observed = `[   ]`, Tu = `[   ]` s |
| `pitchKi` | `[       ]` | Leave 0 unless persistent lean observed |
| `pitchKd` | `[       ]` | Default 0.15; raise to 0.20–0.30 if oscillating |
| `rollKp` | `[       ]` | rollKu observed = `[   ]`, Tu = `[   ]` s |
| `rollKi` | `[       ]` | Leave 0 unless persistent lean observed |
| `rollKd` | `[       ]` | Expect within ~20% of pitchKd |
| `maxOutputVelocity` | `[       ]` | Default 1.00 |
| `envelopeEnterSin` | `[       ]` | Default 0.866 (sin 60°) |
| `envelopeExitSin` | `[       ]` | Default 0.819 (sin 55°) |
| `gyroPitchSign` | `[ +1 / -1 ]` | Flipped during tuning: `[ yes / no ]` |
| `gyroRollSign` | `[ +1 / -1 ]` | Flipped during tuning: `[ yes / no ]` |
| `yawRateKp` | `[       ]` | Default 0.0 (passthrough); set non-zero to enable inner yaw loop |
| `yawRateKi` | `[       ]` | Default 0.0; raise if steady-state yaw drift visible |
| `headingKp` | `[       ]` | Default 1.0; raise for stiffer hold, lower for less yaw chatter |
| `gyroYawSign` | `[ +1 / -1 ]` | Flipped during Phase F1: `[ yes / no ]` |

### Bench notes

- IMU calibration path on this session: `[ cold-boot / warm-boot ]`
- Sign flip required during Phase 4A: `[ pitch / roll / both / neither ]`
- Observed natural period Tu (pitch): `[      ]` s
- Observed natural period Tu (roll): `[      ]` s
- Anomalies / mechanical issues noticed:

```
[                                                                          ]
[                                                                          ]
[                                                                          ]
```

- `balance save` issued after tune: `[ yes / no ]`
- Date of tune: `2026-05-_  `
- Operator: `[                       ]`
