# Balance Bench Bring-Up — 2026-05-19

End-to-end bench protocol covering OTA verification, sign-direction tilt check, arming-state cycle, re-arm windup, and fault envelope. Five gating phases — do not progress to phase N+1 until phase N has all "Expected"s matched.

Each step shows **Expected** behavior and an **Observed** blank for the operator to fill in.

---

## Rollback prologue (do this BEFORE Phase A)

Before flashing this branch:

1. **Record the SHA currently on the robot.** Read it from the boot banner of the previous flash, or note whatever was last uploaded (`git log -1` on the previously-deployed branch). Write it here:

   - Previously-deployed SHA: `[                                ]`

2. **Confirm the safe-mode SoftAP is reachable** from your laptop. The firmware's `BringUp` module spins up SoftAP `bb8-robot-recovery` whenever WiFi-STA association fails. Hold the chip's reset button for 5 s to force that path; your laptop must see SSID `bb8-robot-recovery` in the network list. If the new firmware misbehaves on bench, this is the recovery path:

   ```sh
   git checkout <previous-sha>
   pio run -e robot_ota -t upload
   ```

   - SoftAP visible from laptop: `[   ]`

If either prerequisite fails, fix it before flashing — do not proceed without a rollback option.

---

## Phase A — power-on + comms verification (~1–2 min)

**Use the full WebSerial endpoint at `http://bb8-robot.local:81/webserial`, NOT WebSerial Lite.** Lite re-emits buffered input on reconnect, which surfaces as duplicate `ok:` lines from the balance tuner and spurious `error: unknown balance verb` errors during Phase B.

Logging note: `ArmingState` is the single source of truth for transition logs. A successful transition prints exactly one line, formatted `[arming] -> <State>`. Re-issuing the same verb (e.g. `arm` while already Armed) is a FSM no-op and prints nothing — use `armstate` to query.

IMU note: BNO055 runs in **IMUPLUS** mode (gyro + accel fusion, no magnetometer). `isFullyCalibrated()` only checks `gyro==3 && accel==3` — `sys` and `mag` stay at 0 forever and that's expected (the `[cal]` lines still print all four, ignore the last two). Cold-boot expectation: hold chip still (gyro→3 in ~2s), then place on each of 6 cube faces ~10s each (accel→3, ~60s total). Once `[cal] fully calibrated — offsets saved` appears, offsets are persisted to NVS and every subsequent boot fast-paths to `[cal] warm boot — gyro settled, ready` within ~1s. Wipe NVS or swap chips → cold boot again.

| # | Action | Expected | Observed |
|---|--------|----------|----------|
| 1 | `pio run -e robot_ota -t upload` | Console: `BB-8 main_robot starting.` and `[arming] boot state: Disarmed` | `[   ]` |
| 2 | Open WebSerial at `http://bb8-robot.local:81/webserial`; send `armstate` | `[arming] Disarmed` | `[   ]` |
| 3 | While WebSerial is still connected, kick a second OTA upload | `[ota] update starting` and `[arming] -> Disarmed` (only if currently Armed; silent no-op otherwise — both confirm the OTA→disarm hook fired) | `[   ]` |
| 4a | Send `arm` | `[arming] -> Armed` | `[   ]` |
| 4b | Send `kill` | `[arming] -> Killed` | `[   ]` |
| 4c | Send `clearkill` | `[arming] clearKill -> Disarmed` | `[   ]` |
| 4d | Send `arm` | `[arming] -> Armed` | `[   ]` |
| 4e | Send `disarm` | `[arming] -> Disarmed` | `[   ]` |

Gate: do not proceed unless every Observed matches Expected.

---

## Phase B — sign-direction check (robot OFF the ground, wheels free)

Purpose: bench data that resolves **B1** (balance controller sign question — three independent analyses disagreed). Defer the controller change until this phase produces a recorded observation.

Setup:

```
balance set pitchKp 0.05
balance set pitchKi 0
balance set pitchKd 0
balance set rollKp 0
balance set rollKi 0
balance set rollKd 0
balance save
arm
```

| # | Action | Expected (decision matrix) | Observed |
|---|--------|----------------------------|----------|
| 1 | Tilt platform front-down / back-up by hand | Wheels roll shell **BACKWARD** → current sign matches sphere/BB-8 spec, **no flip needed**. Wheels roll shell **FORWARD** → Segway model, **flip pitchKp**. | `[   ]` |
| 2 | Tilt left-side-down | Shell rolls to the **RIGHT** → correct sign (sphere). Shell rolls to the **LEFT** → **flip rollKp**. | `[   ]` |
| 3 | `disarm` | `[arming] -> Disarmed` | `[   ]` |

Record the decision (no flip / flip pitch / flip roll / flip both) in the **Findings** section at the bottom — that data feeds the follow-up B1 PR.

---

## Phase C — turn/strafe convention (`drivetrain_test` env)

Purpose: bench data that resolves **B7** (kinematics L/R question). The teleop pages, firmware kinematics, and IMU all agree on `+vy = LEFT, +ω = CCW` REP-103 on paper; physical observation needed.

```
pio run -e drivetrain_test_ota -t upload
```

Connect WebSerial. Each row sends a body-frame velocity command `vx vy omega`.

| # | Command | Expected (REP-103) | Observed |
|---|---------|--------------------|----------|
| 1 | `0.3 0 0` | Shell rolls **forward** | **2026-05-19**: rolled **BACKWARD** — pre-fix vx sign was inverted. **Fixed in OmniKinematics.h (vx coefficient `-sin` → `+sin`).** Re-verify after re-flash. |
| 2 | `0 0.3 0` | Shell strafes **LEFT** | **2026-05-19**: strafed LEFT — correct. |
| 3 | `0 0 30`  | Shell yaws **LEFT (CCW from above)** | **2026-05-19**: yawed CCW — correct. |
| 4 | `stop`    | Wheels brake | `[   ]` |

Record any further disagreements in the **Findings** section — they feed the follow-up B7 PR. Re-flashing after the vx fix is required before Phase D/E continue.

---

## Phase D — re-arm windup verification (`robot_ota` env)

**REQUIRED**: robot **ON THE GROUND** (or under wheel load — e.g. shell mass on the platform). Windup-jolt (B3) is invisible on free-spinning wheels.

Note: the producer-silence auto-disarm backstop (B4) has been **removed**. Armed state now persists until an explicit `disarm`, `kill`, OTA upload, or fault-envelope trip. Short-term silence (<200ms) still ramps drive_cmd to zero via `STALENESS_TIMEOUT_MS`, but the controller stays armed and the wheels hold-zero rather than auto-disarming. If the operator closes the producer tab while armed, the robot remains armed — explicit safety responsibility is on the operator (or the kill switch).

```
pio run -e robot_ota -t upload
```

| # | Action | Expected | Observed |
|---|--------|----------|----------|
| 1 (**B3 verification — robot ON GROUND**) | Open webapp `/control`, connect, click **Arm**, send forward velocity (hold `W`) for ~1 s, click **Disarm**, **immediately** click **Arm** again, send zero velocity (no keys). | Wheels do **NOT** jolt on re-arm — PID integrators were reset on disarm edge. Pre-fix the accumulated I-term would slam wheels on re-arm. | `[   ]` |
| 2 | Arm via WebSerial, wait 10+ seconds without sending any velocity. Send `armstate`. | `[arming] Armed` (no auto-disarm — backstop is gone). | `[   ]` |

Gate: B3 is the safety-critical fix shipping in this PR. Must observably match Expected before moving on.

---

## Phase E — fault envelope (robot on the ground or in a cradle)

Setup:

```
balance reset
balance save
arm
```

| # | Action | Expected | Observed |
|---|--------|----------|----------|
| 1 | Manually tilt past 60° (enter envelope) | Wheels stop: `_drivetrain.drive({0, 0, 0})`, fault latched | `[   ]` |
| 2 | Return below 55° (exit envelope, Schmitt hysteresis) | Wheels resume balancing | `[   ]` |
| 3 | `disarm` | `[arming] -> Disarmed` | `[   ]` |

---

## Phase F — yaw + heading hold (robot on the ground or in a cradle)

This phase brings up the cascaded yaw controller and the heading-hold outer loop. Run after Phase E. Both loops are off-by-default in firmware shipped before Commit 3 (`yawRateKp=0`, `headingKp=0`); from Commit 4 onward `headingKp=1.0` is the compiled default but `yawRateKp` is still zero — without a non-zero `yawRateKp` the inner PID is bypassed and the outer loop is moot.

Setup:

```
balance reset
balance save
arm
```

### Phase F1 — yaw-rate sign sanity (cradle, wheels free)

| # | Action | Expected | Observed |
|---|--------|----------|----------|
| 1 | `balance set yawRateKp 0.5` | `ok: yawRateKp=0.5` | `[   ]` |
| 2 | Manually spin the chassis CCW (looking from above) | Wheels drive opposite (CW) to null the spin | `[   ]` |
| 3 | Manually spin CW | Wheels drive CCW | `[   ]` |
| 4 | If wheels reinforce the spin (positive feedback): `balance set gyroYawSign -1` | `ok: gyroYawSign=-1` | `[   ]` |
| 5 | Repeat steps 2–3; confirm corrective behavior | Wheels counteract spin | `[   ]` |

### Phase F2 — yaw-rate gain tuning

| # | Action | Expected | Observed |
|---|--------|----------|----------|
| 1 | Operator stick at zero, gently perturb yaw by hand | Crisp recentering with no oscillation | `[   ]` |
| 2 | If oscillating, lower `yawRateKp` 20%; if sluggish, raise 20% | Settling within ~0.5 s | `[   ]` |
| 3 | If steady-state heading drift visible, `balance set yawRateKi 0.1` and re-perturb | Drift eliminated, no slow oscillation | `[   ]` |
| 4 | `balance save` | `ok: saved` | `[   ]` |

### Phase F3 — heading hold (stick centered)

| # | Action | Expected | Observed |
|---|--------|----------|----------|
| 1 | Confirm `headingKp` non-zero (default `1.0`); leave stick centered | Bot holds heading; push by hand → wheels return to original yaw | `[   ]` |
| 2 | Stick to non-zero omega for 2 s, release | Bot turns under stick; on release, holds the **new** heading (no snap-back to pre-stick yaw) | `[   ]` |
| 3 | Repeat 5× re-latch cycles | No drift, no oscillation between cycles | `[   ]` |
| 4 | Try arming while shaking the chassis (gyro loud) | `arm` is **refused**; `kEvent_PREARM_REJECTED` fires in telemetry | `[   ]` |
| 5 | Hold still 500 ms; send `arm` | `[arming] -> Armed`; heading hold engages from tick 1 | `[   ]` |
| 6 | If hold is too aggressive or oscillating, lower `headingKp` 20%; if drift accumulates, raise 20% | Bot holds at rest within ±2° over 30 s | `[   ]` |
| 7 | `balance save` | `ok: saved` | `[   ]` |

---

## Findings

Use this section to record the bench observations that resolve the deferred questions. The data here directly feeds the follow-up PRs.

### B1 — balance controller sign (from Phase B)

- Pitch sign needs flip: `[ yes / no ]`
- Roll sign needs flip: `[ yes / no ]`
- Notes: `[                                                                  ]`

### B7 — drivetrain L/R / yaw convention (from Phase C)

- `0.3 0 0` produced: **BACKWARD** (2026-05-19) — pre-fix
- `0 0.3 0` produced: **LEFT** — correct
- `0 0 30` produced:  **CCW** — correct
- Disagreements vs REP-103: **vx inverted at the kinematics level** (vy and ω correct)
- Suspected wiring/sign issue: **kinematics formula vx coefficient sign**. Fixed in `src/drivetrain/OmniKinematics.h` (changed `-_sin[i] * v.vx` → `+_sin[i] * v.vx`). Bench re-verification pending.
- Note: this is the upstream fix for B7. B1 (controller pitch/roll sign) is now well-defined against the corrected kinematics — re-run Phase B after re-flash to decide whether `pitchKp`/`rollKp` sign flips are also required.

### General bench notes

```
[                                                                          ]
[                                                                          ]
[                                                                          ]
```
