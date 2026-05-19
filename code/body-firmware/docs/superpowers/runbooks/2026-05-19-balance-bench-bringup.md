# Balance Bench Bring-Up — 2026-05-19

End-to-end bench protocol covering OTA verification, sign-direction tilt check, arming-state cycle, producer-silence backstop, re-arm windup, and fault envelope. Five gating phases — do not progress to phase N+1 until phase N has all "Expected"s matched.

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

| # | Action | Expected | Observed |
|---|--------|----------|----------|
| 1 | `pio run -e robot_ota -t upload` | Console: `BB-8 main_robot starting.` and `[arming] boot state: Disarmed` | `[   ]` |
| 2 | Open WebSerial at `http://bb8-robot.local:81/webserial`; send `armstate` | `[arming] Disarmed` | `[   ]` |
| 3 | While WebSerial is still connected, kick a second OTA upload | `[ota] update starting` and `[arming] Disarmed` (no-op since already disarmed — confirms the OTA→disarm hook is wired) | `[   ]` |
| 4a | Send `arm` | `[arming] Armed` | `[   ]` |
| 4b | Send `kill` | `[arming] Killed` | `[   ]` |
| 4c | Send `clearkill` | `[arming] Disarmed` | `[   ]` |
| 4d | Send `arm` | `[arming] Armed` | `[   ]` |
| 4e | Send `disarm` | `[arming] Disarmed` | `[   ]` |

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
| 3 | `disarm` | `[arming] Disarmed` | `[   ]` |

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
| 1 | `0.3 0 0` | Shell rolls **forward** | `[   ]` |
| 2 | `0 0.3 0` | Shell strafes **LEFT** | `[   ]` |
| 3 | `0 0 30`  | Shell yaws **LEFT (CCW from above)** | `[   ]` |
| 4 | `stop`    | Wheels brake | `[   ]` |

Record any disagreements in the **Findings** section — they feed the follow-up B7 PR.

---

## Phase D — producer-silence backstop + re-arm windup (`robot_ota` env)

**REQUIRED**: robot **ON THE GROUND** (or under wheel load — e.g. shell mass on the platform). Windup-jolt (B3) is invisible on free-spinning wheels. The producer-silence backstop (B4) is observable either way, but co-locate them here to amortize the setup.

**ALSO REQUIRED**: before doing anything else in this phase, open a **second browser window** at `http://bb8-robot.local:81/webserial`. The `armstate` query relies on WebSerial; if you only have the webapp tab open and close it, you lose the channel needed to verify the post-tab-close state.

```
pio run -e robot_ota -t upload
```

| # | Action | Expected | Observed |
|---|--------|----------|----------|
| 1 | Open webapp `/control`, connect, click **Arm** (or press `1`), hold `W` for 1 s to send velocity, then close the webapp tab. Wait 3 s. Send `armstate` via the separate WebSerial window. | `[arming] Disarmed` (auto-disarmed by staleness timeout) | `[   ]` |
| 2 (**B4 verification**) | Reopen webapp, connect, click **Arm**, **IMMEDIATELY** close the tab without sending any velocity. Wait 3 s. Send `armstate` via WebSerial. | Post-fix: `[arming] Disarmed`. Pre-fix bug would report `[arming] Armed` indefinitely. | `[   ]` |
| 3 (**B3 verification — robot ON GROUND**) | Reopen webapp, connect, click **Arm**, send forward velocity (hold `W`) for ~1 s, click **Disarm**, **immediately** click **Arm** again, send zero velocity (no keys). | Wheels do **NOT** jolt on re-arm — PID integrators were reset on disarm edge. Pre-fix the accumulated I-term would slam wheels on re-arm. | `[   ]` |

Gate: B3 and B4 are the safety-critical fixes shipping in this PR. Both must observably match Expected before moving on.

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
| 3 | `disarm` | `[arming] Disarmed` | `[   ]` |

---

## Findings

Use this section to record the bench observations that resolve the deferred questions. The data here directly feeds the follow-up PRs.

### B1 — balance controller sign (from Phase B)

- Pitch sign needs flip: `[ yes / no ]`
- Roll sign needs flip: `[ yes / no ]`
- Notes: `[                                                                  ]`

### B7 — drivetrain L/R / yaw convention (from Phase C)

- `0.3 0 0` produced: `[                                ]`
- `0 0.3 0` produced: `[                                ]`
- `0 0 30` produced:  `[                                ]`
- Disagreements vs REP-103: `[                                              ]`
- Suspected wiring/sign issue: `[                                            ]`

### General bench notes

```
[                                                                          ]
[                                                                          ]
[                                                                          ]
```
