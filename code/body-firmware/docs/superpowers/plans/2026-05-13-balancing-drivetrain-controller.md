# BalancingDrivetrainController Implementation Plan

**Spec:** `code/body-firmware/docs/superpowers/specs/2026-05-13-balancing-drivetrain-controller-design.md`
**PR shape:** one PR, two logical phases. Each phase ends with a reviewer-subagent pass.
**Final-resting plan path (post-approval):** copy this file to `code/body-firmware/docs/superpowers/plans/2026-05-13-balancing-drivetrain-controller.md`. Plan-mode only allows writing to `~/.claude/plans/`.

---

## Context

The robot today routes joystick `BodyVelocity` straight to `OmniDrivetrain` via `PassthroughDrivetrainController` — no IMU feedback, no balancing, no operator arm/disarm. The spec turns the platform into a Segway-style balancer: tilt is the inner loop, IMU drives feedback, and an explicit `ArmingState` (DISARMED / ARMED / KILLED) gates the controller. OTA flashes disarm automatically. Tuning happens live over RemoteSerial with NVS persistence.

The reviewer found a clean seam: the wire-protocol / safety scaffolding ships independently of the balancing controller. Phase 1 lands that scaffolding as a no-op (`PassthroughDrivetrainController` stays wired, system boots DISARMED). Phase 2 lands the balancing controller + tuner + bench harness and swaps the wire-in. Both phases live in one PR so the wire format and the controller that consumes it can't drift.

## Pre-flight assumptions (resolved during STEP 0)

| Question | Decision | Rationale |
| --- | --- | --- |
| Where does `OtaSafeMode::onStart() → ArmingState::disarm()` live? | Inline call inside the existing `ArduinoOTA.onStart` lambda at `OtaSafeMode.cpp:112-115`. | Spec's plain reading; minimal change. Creates `network/ → safety/` include dep (acceptable). |
| Keep `OtaSafeMode::isUpdating()` hot-path check in control task? | Remove the check, keep the API. | Per spec §"OTA composition": arming-state alone provides the guarantee. API stays for status read-back. |
| How to express the tagged Velocity-or-Control parser result? | Extend `FrameParseResult` with a `FrameKind kind` and `ControlVerb control` field. | No `std::variant` precedent in this codebase; one struct stays cheap and easy to read. |
| `RemoteSerial` per-verb registration? | None — `RemoteSerial::onMessage()` is a *single* callback. `main_balance_test.cpp` and `main_robot.cpp` each install one dispatcher that calls `ArmingHandlers::handle(line)` and falls through to `BalanceTuner::handle(line)`. | Matches existing `main_drivetrain_test.cpp` pattern. No new infra. |
| BNO055 axis-remap currently applied? | No. Gyro reads pass through raw. The spec's `gyroPitchSign` / `gyroRollSign` knobs in `BalanceConfig` are the empirical sign-flip lever; bench tuning procedure documents the verification. | Spec already handles this — no plan adjustment needed. |
| `BalanceConfig` factory location? | `RobotConstants::balanceConfig()` in existing `src/drivetrain/RobotConstants.h`. | File already has `drivetrainConfig()`; matches the pattern. |

## Multi-agent workflow

The plan executes in four wave-types per phase: **research → parallel impl → sequential integration → verification**. Implementation agents should not need to make judgment calls — every prompt includes exact files, function signatures, test cases, and verify commands. Anything ambiguous is resolved up-front in research, not by the impl agent.

### Wave 0 — research (already complete)

Performed before this plan was written, in STEP 0 of the planning task. The "Pre-flight assumptions" table at the top of this file records the decisions. No further research wave is required before execution. If, mid-execution, an impl agent reports back "I had to guess about X," the orchestrator dispatches a one-off Explore agent for X and updates the affected step before the next wave.

### Subagent prompt template (use verbatim per step; substitute placeholders)

```
You are implementing Step <N.M> of the BalancingDrivetrainController plan.

Plan file: /Users/alessandro/.claude/plans/task-write-the-fuzzy-octopus.md
Spec:      code/body-firmware/docs/superpowers/specs/2026-05-13-balancing-
           drivetrain-controller-design.md

Read Step <N.M> in full. It is self-contained — it lists every file to
touch, every test case to write, and the commit message. Do not invent
behaviour or refactor beyond the step's scope. If a fact in the step
disagrees with what you find on disk, stop and report — do not improvise.

Workflow (strict TDD):
  1. Create / extend the test file(s) named in "Tests first". Use exactly
     the test names and assertions given.
  2. Run the verify command from "Done when" (typically
     `pio test -e native -f <suite>`). Confirm tests fail with the expected
     error (compile-fail or assertion-fail). Capture the output.
  3. Create / modify the implementation files named in "Implementation"
     using exactly the signatures and code shapes given.
  4. Re-run the verify command. Confirm all listed tests pass. Capture
     the output.
  5. Stage ONLY the files listed in the step's "Commit" block. Never
     `git add -A` / `git add .` — other agents may have unrelated work
     in this worktree.
  6. Commit with the exact subject from the step. No body unless the step
     says otherwise. No `Co-Authored-By`. No testing summaries.

Hard constraints:
- Do not touch files outside the step's stage-list. Surface anything else
  you notice in your report instead of fixing it.
- Do not skip the failing-tests-first step. The orchestrator will reject
  reports that don't include the red→green transition.
- Do not rebase, force-push, or rewrite history.

Report back (under 200 words):
- Commit SHA produced
- The exact verify command(s) you ran + truncated pass/fail output
- Any step text that disagreed with the current code (do not act on the
  disagreement; surface it)
```

### Wave 1 — parallel implementation (research-free, leaf nodes)

Steps with no dependencies on other steps fan out together in one `Agent` message. The orchestrator sends N `Agent` tool-use blocks in a single response, then waits for all N reports before applying their commits.

| Wave | Steps (parallel) | Predicate |
| --- | --- | --- |
| Phase 1 — Wave 1 | **1.1** PID 4-arg overload · **1.2** ArmingState · **1.3** Parser sigil dispatch | none |
| Phase 2 — Wave 1 | **2.1** Controller `dt` param · **2.2** BalanceConfig + factory · **2.3** quatToBodyGravity utility | Phase 1 complete |

Three parallel agents per phase. Each agent owns one focused TDD unit. Build verify: `pio test -e native` (each agent runs its own focused `-f <suite>` subset).

### Wave 2 — sequential integration (wires the leaves together)

Steps that depend on Wave-1 outputs. Run inside-out: leaves of the dep DAG first, root last.

| Wave | Steps | Depends on |
| --- | --- | --- |
| Phase 1 — Wave 2a | **1.4** WS control-frame dispatch · **1.5** OTA → disarm hook | 1.2, 1.3 / 1.2 (parallel within 2a) |
| Phase 1 — Wave 2b | **1.6** `main_robot` arming gate | 1.2, 1.4, 1.5 |
| Phase 2 — Wave 2a | **2.4** BalancingDrivetrainController · **2.5** NVS persistence | 1.1, 2.1, 2.2, 2.3 / 2.2 (parallel within 2a) |
| Phase 2 — Wave 2b | **2.6** BalanceTuner | 1.2, 2.2, 2.5 |
| Phase 2 — Wave 2c | **2.7** bench harness · **2.8** `main_robot` swap · **2.9** runbook | 2.4, 2.6 (all three parallel) |

Wave 2a is parallel within itself; Wave 2b and 2c block on the wave before them. Verify per step.

### Wave 3 — verification (review + build + hardware)

| Phase | Step | Type | Action |
| --- | --- | --- | --- |
| Phase 1 — end | **1.7** reviewer subagent | software | Dispatch general-purpose agent with the Phase-1 reviewer prompt from Step 1.7. Block on its report. Fix any flagged blockers as fixup commits before starting Phase 2. |
| Phase 2 — end | **2.10** final reviewer subagent | software | Dispatch general-purpose agent with the full-PR reviewer prompt from Step 2.10. Block on its report. Fix any blockers before opening the PR. |
| Phase 2 — end | (operator) bench checklist | hardware | The human runs the checklist from the runbook (`docs/superpowers/runbooks/2026-05-13-balance-bringup.md`) on the actual robot. No agent can do this. Open the PR only after the bench checklist passes. |

The reviewer agents do **not** modify code. They audit a frozen diff. Their output is a findings table; the orchestrator decides which findings warrant fixup commits.

### Orchestrator pseudocode

```python
# Phase 1
for agent in parallel([dispatch(step) for step in [1.1, 1.2, 1.3]]):
    apply(agent.commit)
for agent in parallel([dispatch(step) for step in [1.4, 1.5]]):
    apply(agent.commit)
apply(dispatch(1.6).commit)
review_report = dispatch(1.7)
apply_fixups_if_any(review_report)

# Phase 2
for agent in parallel([dispatch(step) for step in [2.1, 2.2, 2.3]]):
    apply(agent.commit)
for agent in parallel([dispatch(step) for step in [2.4, 2.5]]):
    apply(agent.commit)
apply(dispatch(2.6).commit)
for agent in parallel([dispatch(step) for step in [2.7, 2.8, 2.9]]):
    apply(agent.commit)
review_report = dispatch(2.10)
apply_fixups_if_any(review_report)

# Hardware (operator-driven, not agentic)
run_bench_checklist()
open_pr()
```

## Conventions (apply to every commit)

- One atomic commit per step. Imperative subject. No body unless a non-obvious tradeoff needs documenting.
- **Never `git add -A` / `git add .`.** Stage only the files this step touches.
- No `Co-Authored-By` trailer. No testing summaries in commit messages or PR body.
- Tests first, run them, see them fail, write implementation, run again, see them pass. Then commit.
- After any C/C++ change: `pio test -e native` must pass before commit. PlatformIO compile check for the affected env (`robot`, `balance_test`) runs at the end of each phase.

---

# Phase 1 — Foundations & Safety (no behavioural change to drivers)

End-state of Phase 1: the robot still uses `PassthroughDrivetrainController`. `ArmingState` exists and gates the control task. Boot default is `DISARMED` so the robot won't move until the operator sends `c:arm` over the new wire format. OTA flashes disarm automatically.

---

### Step 1.1 — `PID` 4-arg overload (rate-aware)

**Inputs:** `src/control/PID.h`, `test/test_pid/test_pid.cpp`, spec §"PID extension".

**Tests first** (append to `test/test_pid/test_pid.cpp`):
- `test_4arg_with_finite_diff_rate_matches_3arg` — call 3-arg `compute(s, m, dt)` on one PID; call 4-arg `compute(s, m, (m - m_prev)/dt, dt)` on a fresh PID after the same warm-up call; assert equal output within `TOL`.
- `test_4arg_with_external_rate_uses_supplied_rate` — set `Kd = 1.0`, `Kp = Ki = 0`. First call seeds state. Second call: `compute(setpoint=0, measurement=0, rate=2.0, dt=0.1)`; assert output ≈ `-2.0` (derivative-on-measurement: `-Kd * rate`). Independent of `_prevMeasurement`.
- `test_4arg_respects_anti_windup` — `Ki = 1.0`, output clamped `[-1, +1]`. Drive 4-arg version with large positive error 10 times; assert integral does not exceed `_outputMax / _ki`.
- `test_4arg_respects_deadband` — `deadband > 0`, setpoint=0, measurement < deadband, rate=0; assert returns 0 and resets state.
- Register all four in `main()` `RUN_TEST` block.

Run: `pio test -e native -f test_pid` — expect 4 new failures.

**Implementation** (`src/control/PID.h`):
- Add `float compute(float setpoint, float measurement, float rate, float dt)` containing the canonical body. The current 3-arg `compute` becomes a thin wrapper:
  ```cpp
  float compute(float setpoint, float measurement, float dt) {
      float rate = _firstCompute ? 0.0f : (measurement - _prevMeasurement) / dt;
      return compute(setpoint, measurement, rate, dt);
  }
  ```
- The 4-arg version uses `derivative = -rate` (derivative-on-measurement, same sign as today).
- 4-arg version still updates `_prevMeasurement` so a later 3-arg call sees the right history (gives consumers freedom to mix).
- File stays header-only.

Run: `pio test -e native -f test_pid` — expect all green (old + new).

**Parallel?** Yes. No dependencies on other Phase-1 work.

**Done when:** All `test_pid` cases pass; spot-check that `OmniDrivetrain`'s per-wheel PID calls still compile (they continue using the 3-arg form).

**Commit:** `pid: add rate-aware 4-arg compute() overload`

Files staged: `src/control/PID.h`, `test/test_pid/test_pid.cpp`.

---

### Step 1.2 — `ArmingState` module

**Inputs:** Spec §"ArmingState", existing `src/util/` for header conventions (e.g. `BB8_ASSERT` patterns).

**Tests first** — new dir `test/test_arming_state/test_arming_state.cpp`. Mirror the spec test table verbatim:
- `test_boot_state_is_disarmed` — after `ArmingState::begin()`, `get() == Disarmed`; `isArmed() == false`.
- `test_disarmed_to_armed` — `arm()` → `get() == Armed`; `isArmed() == true`.
- `test_armed_to_disarmed` — from Armed, `disarm()` → `get() == Disarmed`.
- `test_any_to_killed` — from Armed: `kill()` → `Killed`. Fresh state, from Disarmed: `kill()` → `Killed`.
- `test_arm_while_killed_is_noop` — `kill(); arm();` → still Killed.
- `test_clearkill_from_non_killed_is_noop` — from Disarmed: `clearKill();` → still Disarmed; from Armed: `clearKill();` → still Armed.
- `test_clearkill_from_killed_to_disarmed` — `kill(); clearKill();` → Disarmed.

Helper at top of file resets state between tests by calling `begin()` in `setUp()`.

Run: `pio test -e native -f test_arming_state` — expect compile failure (header doesn't exist).

**Implementation:**
- Create `src/safety/ArmingState.h` exposing the namespace API exactly as in the spec (lines 117-130). Add `-Isrc/safety` to `platformio.ini` includes for `env:native`, `env:robot`, `env:wemos_d1_uno32`, `env:wemos_d1_uno32_ota_console` (one line each).
- Create `src/safety/ArmingState.cpp` with `std::atomic<State> g_state{State::Disarmed}` (file-static). `arm()` / `disarm()` use `compare_exchange_strong` only if Killed-aware semantics need it; per spec a plain `if (get() == Killed) return; g_state.store(...)` is sufficient given the single-writer invariant — document the invariant in a one-line comment at the top of the .cpp.
- Each transition logs to both `Serial` and `RemoteSerial::println(...)`. Guard the `RemoteSerial::println` with a `#ifdef ARDUINO` since native tests don't link `RemoteSerial`. Equivalent: put the log calls behind a thin `arming_log(const char*)` static that no-ops on native builds.
- Update `env:native` build_flags: add `-Isrc/safety`. Update `src/network/`-adjacent envs likewise.

Run: `pio test -e native -f test_arming_state` — expect green.

**Parallel?** Yes. No dependencies.

**Done when:** all 7 arming-state tests pass; `pio run -e robot` still compiles (sanity check for include path).

**Commit:** `safety: add ArmingState (Disarmed/Armed/Killed) module`

Files staged: `src/safety/ArmingState.h`, `src/safety/ArmingState.cpp`, `test/test_arming_state/test_arming_state.cpp`, `platformio.ini`.

---

### Step 1.3 — `CommandFrameParser` sigil dispatch + tagged result

**Inputs:** `src/input/CommandFrameParser.h`, `test/test_command_frame_parser/` (existing layout for fixture style), spec §"Wire protocol".

**Tests first** (append to existing `test/test_command_frame_parser/test_command_frame_parser.cpp`):
- `test_legacy_velocity_unprefixed_still_parses` — input `"1.0,2.0,3.0"` → `kind == Velocity`, `velocity == {1, 2, 3}`, `error == None`.
- `test_velocity_with_v_sigil_parses` — input `"v1.0,2.0,3.0"` → same as above.
- `test_control_arm` — input `"c:arm"` → `kind == Control`, `control == ControlVerb::Arm`.
- `test_control_disarm` — input `"c:disarm"` → `control == ControlVerb::Disarm`.
- `test_control_kill` — input `"c:kill"` → `control == ControlVerb::Kill`.
- `test_control_clearkill` — input `"c:clearkill"` → `control == ControlVerb::ClearKill`.
- `test_control_unknown_verb` — input `"c:explode"` → `error == InvalidControlVerb` (new enum member).
- `test_control_missing_colon` — input `"carm"` → `error == InvalidControlVerb` (or new `MalformedControlFrame`; pick one and use it consistently).
- `test_negative_leading_minus_is_velocity` — input `"-1.0,0,0"` → velocity-path (regression — the sigil dispatcher must treat `-`, `+`, digit, `.` as velocity-like).

Run: `pio test -e native -f test_command_frame_parser` — expect compile failure / failures.

**Implementation:**
- Edit `src/input/CommandFrameParser.h`. Add new enums and fields:
  ```cpp
  enum class FrameKind : uint8_t { Velocity, Control };
  enum class ControlVerb : uint8_t { Arm, Disarm, Kill, ClearKill };
  // Extend FrameParseError with InvalidControlVerb.
  struct FrameParseResult {
      FrameKind kind = FrameKind::Velocity;
      BodyVelocity velocity{};
      ControlVerb  control = ControlVerb::Arm;
      FrameParseError error = FrameParseError::None;
      bool ok() const { return error == FrameParseError::None; }
  };
  ```
  Rename the existing `value` field to `velocity` (4 call sites under `WebSocketCommandProducer.cpp:148-161` need the rename — bundle that into the same commit so the rename never lands half-applied).
- In `parseCommandFrame`, after the length check, inspect `buf[0]`:
  - `'c'` → control path: require `buf[1] == ':'`, then `strcmp` (or `memcmp` against fixed-length strings) for the four verbs. Return early.
  - `'v'` → velocity path: advance `p` past the `'v'`, then existing CSV logic.
  - digit / `'-'` / `'+'` / `'.'` → velocity path, existing CSV logic from start.
  - anything else → `InvalidVx` (current behaviour for malformed leading char).
- Keep all existing velocity-parse failure modes unchanged.

Run: `pio test -e native -f test_command_frame_parser` — expect green (old + new).

**Parallel?** Yes. Independent of 1.1 and 1.2.

**Done when:** all parser tests pass; backward-compat regression test (unprefixed velocity) passes.

**Commit:** `parser: extend CommandFrameParser with sigil dispatch (v/c)`

Files staged: `src/input/CommandFrameParser.h`, `src/input/WebSocketCommandProducer.cpp` (field-rename only), `test/test_command_frame_parser/test_command_frame_parser.cpp`.

---

### Step 1.4 — `WebSocketCommandProducer` control-frame dispatch

**Inputs:** Steps 1.2, 1.3. Files: `src/input/WebSocketCommandProducer.cpp`, `test/test_mock_command_producer/` (existing fixture style).

**Tests first** — new file `test/test_websocket_control_dispatch/test_websocket_control_dispatch.cpp`. Use FFF fakes for `ArmingState`:
- `DECLARE_FAKE_VOID_FUNC(ArmingState_arm); DECLARE_FAKE_VOID_FUNC(ArmingState_disarm); ...` etc, with a tiny shim header that redefines the namespace functions for the test binary only. (Pattern: see how `test_mock_command_producer` handles transport stubs.)
- `test_control_arm_frame_invokes_arming_state_arm` — feed parser result `{kind=Control, control=Arm}` to the dispatcher; assert `ArmingState_arm_fake.call_count == 1`, latch.write was NOT called.
- `test_control_disarm` / `test_control_kill` / `test_control_clearkill` — analogous.
- `test_velocity_frame_writes_latch_does_not_invoke_arming` — feed velocity frame; assert latch.write count == 1, no arming calls.

Run: expect compile failure (`dispatch()` helper doesn't exist).

**Implementation:**
- Note the file's current shape (post-commit 8a28fc1): `WStype_CONNECTED` uses `_activeClient.exchange(clientNum)` for drop-oldest pre-emption, and heartbeat is `enableHeartbeat(3000, 2500, 2)`. **Do not touch either** — they are orthogonal to this step. Only the `WStype_TEXT` branch (currently lines ~146-174) is in scope.
- Extract the `WStype_TEXT` body into a static-or-private member `void dispatchFrame(const FrameParseResult& r)` that:
  - On `r.kind == FrameKind::Velocity`: `_latch.write(r.velocity)`, increment `_frameCount`.
  - On `r.kind == FrameKind::Control`: switch on `r.control` calling `ArmingState::arm() / disarm() / kill() / clearKill()`. Log each transition to `Serial.printf("[ws] control: %s\n", verbName)`.
- The original event handler calls `dispatchFrame(result)` after the `ok()` check.
- Include `safety/ArmingState.h` in `WebSocketCommandProducer.cpp`.
- The control-frame path does not touch `_parseFailCount` or the per-1000 counter dump — those are velocity-frame-specific. (Rationale: keep "frames" as the throughput measure of the user's stick.)

Run: `pio test -e native -f test_websocket_control_dispatch` — expect green. Also re-run `test_command_frame_parser` (regression).

**Parallel?** No — depends on 1.2 (ArmingState API) and 1.3 (FrameParseResult shape).

**Done when:** new control-dispatch tests pass; existing producer behaviour unchanged for velocity frames (still routed to latch).

**Commit:** `ws-producer: dispatch control frames to ArmingState`

Files staged: `src/input/WebSocketCommandProducer.cpp`, `src/input/WebSocketCommandProducer.h` (if `dispatchFrame` is a member), `test/test_websocket_control_dispatch/test_websocket_control_dispatch.cpp`.

---

### Step 1.5 — `OtaSafeMode::onStart` calls `ArmingState::disarm()`

**Inputs:** Step 1.2. Files: `src/network/OtaSafeMode.cpp`, `src/safety/ArmingState.h`.

**Tests first** — there is no native-testable path for `ArduinoOTA.onStart` itself (it's an upstream library handler). Instead test the small integration helper:
- Refactor the OTA-start handler into a free function `static void onOtaStart()` in `OtaSafeMode.cpp` that does the three things in order: `g_updating.store(true)`, `ArmingState::disarm()`, `Serial.println("[ota] update starting")`. Expose it via a deliberately-named `OtaSafeMode::detail::onOtaStartForTest()` in a `#ifdef BB8_TEST_HOOKS` block (the `env:native` build_flags already define `BB8_DEBUG`; either reuse that or add `-DBB8_TEST_HOOKS` to `env:native`).
- New test dir `test/test_ota_safe_mode_hooks/test_ota_safe_mode_hooks.cpp`:
  - `test_on_ota_start_disarms` — set ArmingState to Armed; call `OtaSafeMode::detail::onOtaStartForTest()`; assert `ArmingState::get() == Disarmed`.
  - `test_on_ota_start_from_disarmed_is_noop` — ArmingState begins Disarmed; call the helper; assert still Disarmed (no exception, no spurious state change).
  - `test_on_ota_start_from_killed_stays_killed` — ArmingState set to Killed; call helper; assert still Killed (`disarm()` is a no-op when Killed per spec).

Run: expect compile failure (`onOtaStartForTest` doesn't exist).

**Implementation:**
- In `src/network/OtaSafeMode.cpp`:
  - `#include "ArmingState.h"` near the top.
  - Locate the `ArduinoOTA.onStart([]() { ... });` lambda inside `OtaSafeMode::begin()` (post-commit 6b3423b: currently around lines 118-121, but use a textual grep — the file is under active maintenance and exact line numbers drift). The lambda today does only `g_updating.store(true)` and `Serial.println("[ota] update starting")`. **Do not** alter `tryConnectSta` (which sets `WIFI_PS_NONE`), the atomic `g_last_sta_connected_ms`, or any other WiFi-tuning logic — those are out of scope for this step.
  - Move the lambda body into a file-static `static void onOtaStart()` containing:
    ```cpp
    static void onOtaStart() {
        g_updating.store(true, std::memory_order_release);
        ArmingState::disarm();
        Serial.println("[ota] update starting");
    }
    ```
  - Replace the lambda registration with `ArduinoOTA.onStart(onOtaStart);`.
  - Under `#ifdef BB8_TEST_HOOKS`, expose `namespace OtaSafeMode::detail { void onOtaStartForTest() { onOtaStart(); } }`.
- `env:native` in `platformio.ini` gains `-DBB8_TEST_HOOKS`. Existing `BB8_DEBUG` flag is untouched (different concern).
- **No change to `BringUp.h` / `BringUp.cpp`.** `BringUp::begin()` calls `OtaSafeMode::begin()` which now (via this commit) registers a lambda that disarms on OTA start. The dependency goes `OtaSafeMode → ArmingState`; `BringUp` stays unaware of `ArmingState`, preserving the facade.
- Note: the existing `test/test_ota_policy/` covers `OtaPolicy` reboot logic. This new directory tests the integration helper specifically — orthogonal scope.

Run: `pio test -e native -f test_ota_safe_mode_hooks` — green.

**Parallel?** No — depends on 1.2.

**Done when:** new tests pass; `pio run -e robot` compiles (link sanity: ArmingState reachable from OtaSafeMode TU).

**Commit:** `ota: disarm via ArmingState on OTA upload start`

Files staged: `src/network/OtaSafeMode.cpp`, `platformio.ini`, `test/test_ota_safe_mode_hooks/test_ota_safe_mode_hooks.cpp`.

---

### Step 1.6 — `main_robot.cpp` arming gate + dispatcher

**Inputs:** Steps 1.2, 1.4, 1.5. Files: `src/main_robot.cpp`.

**Tests first** — no native test (this is integration glue inside the FreeRTOS task body). Bench-verify after build:
- Build `pio run -e robot` — must succeed.
- After upload: on boot, `RemoteSerial` prints `arming: Disarmed (boot)`. Send `c:arm` from the webapp scratchpad — log shows `arming: Armed`. Send `c:kill` — log shows `arming: Killed`. Send `c:clearkill` — back to `Disarmed`.

**Implementation:**
1. Add `#include "ArmingState.h"` at the top of `main_robot.cpp`.
2. **Setup() ordering — interaction with the BringUp facade.** The current `setup()` starts with `BringUp::begin();` (commits b195fb3 / d6fbf62), which internally does `Serial.begin → OtaSafeMode::begin → RemoteSerial::begin`. The OTA-start lambda is registered inside `OtaSafeMode::begin`, so by the time `BringUp::begin()` returns, the lambda is wired but has not yet fired. Insert `ArmingState::begin()` **immediately after** `BringUp::begin()`. The ordering is defensive (the file-static `std::atomic<State>` is already `Disarmed` at boot via static init, so even an OTA hook firing before `ArmingState::begin()` would call `disarm()` on a Disarmed state — a no-op — but the explicit init removes any doubt).
3. Install `RemoteSerial::onMessage(handleRemoteSerialLine)` **after** `BringUp::begin()` (which calls `RemoteSerial::begin()` internally — installing the message handler before that would lose the callback registration). The dispatcher (file-static) does:
   - Trims the line.
   - Matches `"arm"`, `"disarm"`, `"kill"`, `"clearkill"`, `"armstate"` and calls the matching `ArmingState::` function (or prints state for `armstate`).
   - On no match: prints `RemoteSerial::printf("[cmd] unknown verb: %s\n", line.c_str())`.
4. Modify `controlTask` body, replacing lines 138-159:
   ```cpp
   BodyVelocity drive_cmd{0, 0, 0};

   // Pull freshest command + compute staleness ramp (unchanged from current
   // lines 127-148, but assign result into drive_cmd).
   // ...

   const auto arming = ArmingState::get();
   if (arming == ArmingState::State::Killed) {
       g_controller->stop();
       drive_cmd = {0, 0, 0};
   } else if (arming == ArmingState::State::Armed) {
       // Auto-disarm on producer silence (>2000ms). RobotConstants::STALE_DISARM_MS.
       if (have_seen_fresh && (now - last_fresh_ms) > STALE_DISARM_MS) {
           ArmingState::disarm();
           drive_cmd = {0, 0, 0};
       } else {
           IMUReading imu_reading = g_imu.read();
           g_controller->update(drive_cmd, imu_reading);  // (becomes …, dt) in Phase 2)
       }
   } else {
       // Disarmed: hard-zero, skip controller. Per-wheel PIDs hold motors at 0.
       drive_cmd = {0, 0, 0};
       g_drivetrain.drive(drive_cmd);
   }
   g_drivetrain.update(dt);
   ```
5. Delete the `if (OtaSafeMode::isUpdating()) drive_cmd = {0, 0, 0};` block at current lines 153-155 (per pre-flight decision: arming-state alone gates OTA).
6. Add `STALE_DISARM_MS = 2000` to `src/drivetrain/RobotConstants.h` alongside `STALENESS_TIMEOUT_MS`.
7. Add to top of `setup()` log: `RemoteSerial::println("[arming] boot state: Disarmed");`.

**Parallel?** No — depends on 1.2 / 1.4 / 1.5 all landing first.

**Done when:**
- `pio run -e robot` compiles.
- Bench: boot the robot, webapp scratchpad sends `c:arm` / `c:disarm` / `c:kill` / `c:clearkill` and the state transitions log on RemoteSerial. With `PassthroughDrivetrainController` still wired, an armed-state joystick command moves the wheels; a `c:disarm` immediately stops them.
- Bench: trigger an OTA upload from the IDE while armed; observe `[arming] Armed → Disarmed` in the RemoteSerial log at the moment upload starts.
- Bench: leave producer disconnected for >2s while armed; observe auto-disarm.

**Commit:** `main_robot: gate control task on ArmingState, drop ota check`

Files staged: `src/main_robot.cpp`, `src/drivetrain/RobotConstants.h`.

---

### Step 1.7 — Phase 1 reviewer subagent

Dispatch a reviewer subagent (single `general-purpose` Agent) with the following prompt — verbatim — and pause for its report before starting Phase 2:

> You are reviewing the Phase-1 portion of a balancing-drivetrain feature against its spec. The PR is not yet finished — Phase 2 will land the actual balancing controller. Phase 1 lands the wire-protocol, safety scaffolding, and PID overload.
>
> Read the spec at `code/body-firmware/docs/superpowers/specs/2026-05-13-balancing-drivetrain-controller-design.md`, focusing on the sections: "ArmingState", "Wire protocol — CommandFrameParser", "PID extension", "Arming + control-task integration", "OTA composition".
>
> Read the current diff: `git diff main...HEAD -- code/body-firmware/`. List every commit on the branch.
>
> Check:
> 1. Every spec requirement in the listed sections is implemented in some commit. Cite commit + file.
> 2. Each commit is atomic (one logical change, tests + impl together when applicable, no drive-by edits).
> 3. No `git add -A` or `git add .` was used (check via `git log --stat` for unrelated files).
> 4. No `Co-Authored-By` trailers.
> 5. No testing summaries in commit messages.
> 6. `pio test -e native` passes; `pio run -e robot` compiles. (Run both.)
> 7. The system at HEAD is operationally safe — i.e. `PassthroughDrivetrainController` still wired, boot default DISARMED, OTA disarms on start.
>
> Report findings as a table (issue / location / severity / suggested fix). If you find no issues, say so explicitly. Do not modify code.

If reviewer flags issues, address them as fixup commits on the same branch before Phase 2 begins.

---

# Phase 2 — Balancing controller & tuning

End-state of Phase 2: `BalancingDrivetrainController` is wired into `main_robot.cpp`. `BalanceTuner` registers RemoteSerial verbs that mutate `BalanceConfig` over an atomic-swap pointer slot, persisted to NVS. A separate `main_balance_test.cpp` bench binary lets operators tune without the webapp.

---

### Step 2.1 — `DrivetrainController::update` adds `dt`; update `Passthrough` + existing test

**Inputs:** Spec §"DrivetrainController base class change". Files: `src/drivetrain/controller/DrivetrainController.h`, `src/drivetrain/controller/PassthroughDrivetrainController.h`, `src/main_robot.cpp`, `test/test_passthrough_controller/test_passthrough_controller.cpp`.

**Tests first** — modify `test_passthrough_controller.cpp`:
- Change existing `test_update_forwards_command_to_drivetrain` to call `controller->update(cmd, imuData, 0.01f)` and assert the same outcome (signature change is the test).
- New: `test_update_ignores_dt_value` — call with `dt = 0.01f`, then `dt = 1.0f`, then `dt = 0.0f`; assert `lastDrive` is updated each time and matches `cmd` exactly (no scaling by dt).

Run: `pio test -e native -f test_passthrough_controller` — expect compile failure on the signature.

**Implementation:**
- `DrivetrainController.h` line 20: change pure virtual to `virtual void update(const TCommand& command, const TIMUData& imuData, float dt) = 0;`.
- `PassthroughDrivetrainController.h` line 17: change override to `void update(const BodyVelocity& command, const IMUReading& /*imuData*/, float /*dt*/) override { _drivetrain.drive(command); }`.
- `main_robot.cpp:159`: now `g_controller->update(drive_cmd, imu_reading, dt);` — `dt` is already computed on line 162; move that computation earlier in the loop so both calls use the same value.

Run: `pio test -e native -f test_passthrough_controller` — green. `pio run -e robot` — compiles.

**Parallel?** Yes — independent of every other Phase-2 step. (Anything new in Phase 2 will be written against the new signature.)

**Done when:** existing passthrough tests pass; `main_robot` compiles.

**Commit:** `drivetrain-controller: add dt parameter to update()`

Files staged: `src/drivetrain/controller/DrivetrainController.h`, `src/drivetrain/controller/PassthroughDrivetrainController.h`, `src/main_robot.cpp`, `test/test_passthrough_controller/test_passthrough_controller.cpp`.

---

### Step 2.2 — `BalanceConfig` struct + `RobotConstants::balanceConfig()` factory

**Inputs:** Spec §"BalanceConfig". File: new `src/drivetrain/BalanceConfig.h`, modify `src/drivetrain/RobotConstants.h`.

**Tests first** — new dir `test/test_balance_config/test_balance_config.cpp` (this dir hosts both struct-shape tests and the persistence tests added in 2.5; keep them in the same TU):
- `test_default_config_values_match_spec` — call `RobotConstants::balanceConfig()`; assert each field equals the spec's literals (line 88-100 of spec). One `TEST_ASSERT_FLOAT_WITHIN(1e-6f, expected, actual)` per field.
- `test_config_is_trivially_copyable` — `static_assert(std::is_trivially_copyable<BalanceConfig>::value)` at TU scope.
- `test_envelope_invariant_in_defaults` — assert `envelopeExitSin < envelopeEnterSin` (the spec's validation rule must hold for the defaults).

Run: expect compile failure (`BalanceConfig.h` missing).

**Implementation:**
- Create `src/drivetrain/BalanceConfig.h` with the struct exactly as in spec lines 60-83. POD layout; no constructor. Header-only.
- Add `inline BalanceConfig balanceConfig() { return {...}; }` to `src/drivetrain/RobotConstants.h` with the spec's default values.

Run: green.

**Parallel?** Yes — independent of everything.

**Done when:** struct compiles, defaults test passes, trivially-copyable assertion compiles.

**Commit:** `drivetrain: add BalanceConfig struct and default factory`

Files staged: `src/drivetrain/BalanceConfig.h`, `src/drivetrain/RobotConstants.h`, `test/test_balance_config/test_balance_config.cpp`.

---

### Step 2.3 — `quatToBodyGravity` utility + unit tests

**Inputs:** Spec §"Quaternion → body-frame gravity". File: new `src/drivetrain/controller/quatToBodyGravity.h`. (Keep this as a separate header so the math is unit-testable independent of the controller class — TDD discipline for the sign-convention check.)

**Tests first** — new dir `test/test_quat_to_body_gravity/test_quat_to_body_gravity.cpp`:
- `test_identity_quaternion_yields_zero_zero_minus_one` — `q = {1, 0, 0, 0}`; assert `gx ≈ 0`, `gy ≈ 0`, `gz ≈ -1`. **This test is the sign-convention gate** — if it doesn't pass the formula is wrong.
- `test_pitch_forward_30deg` — `q` representing 30° rotation about body-y axis: `q = {cos(15°), 0, sin(15°), 0}`. Assert `gx ≈ sin(30°) = 0.5` (positive — pitched forward gives positive gx), `gy ≈ 0`, `gz ≈ -cos(30°) ≈ -0.866`.
- `test_roll_left_30deg` — `q = {cos(15°), sin(15°), 0, 0}` (30° about body-x). Assert `gx ≈ 0`, `gy ≈ -sin(30°) = -0.5` (left-roll: gy negative per spec sign convention), `gz ≈ -0.866`.
- `test_full_inversion_180deg_about_x` — `q = {0, 1, 0, 0}`. Assert `gz ≈ +1` (upside-down).
- `test_grid_of_pitch_roll_combinations` — loop pitch ∈ {-30°, -15°, 0, 15°, 30°}, roll ∈ {-30°, ..., 30°}, build the quaternion `q = q_roll * q_pitch` (use a tiny helper), compute analytic ground truth `gx = sin(pitch)`, `gy = -sin(roll)*cos(pitch)`, `gz = -cos(roll)*cos(pitch)`, assert within `1e-4`.

Run: expect compile failure.

**Implementation:**
- Create `src/drivetrain/controller/quatToBodyGravity.h`:
  ```cpp
  #pragma once
  #include "Quat.h"

  inline void quatToBodyGravity(const Quat& q, float& gx, float& gy, float& gz) {
      gx =  2.0f * (q.w * q.y - q.x * q.z);
      gy = -2.0f * (q.w * q.x + q.y * q.z);
      gz = -(q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
  }
  ```
- Header-only; no .cpp.

Run: green.

**Parallel?** Yes — pure math, no dependencies.

**Done when:** all gravity tests pass, especially identity-quaternion test (sign-convention contract).

**Commit:** `imu-math: add quatToBodyGravity utility`

Files staged: `src/drivetrain/controller/quatToBodyGravity.h`, `test/test_quat_to_body_gravity/test_quat_to_body_gravity.cpp`.

---

### Step 2.4 — `BalancingDrivetrainController` class

**Inputs:** Steps 1.1, 2.1, 2.2, 2.3. Spec §"BalancingDrivetrainController" and §"Tilt-fault Schmitt".

**Tests first** — new dir `test/test_balancing_controller/test_balancing_controller.cpp`. Use `MockDrivetrain` and `MockIMU` patterns from `test_passthrough_controller.cpp`. Reusable helpers at file scope: `BalanceConfig makeTestConfig()` returning small known gains, `Quat upright()` returning identity, `Quat pitchedForward(float rad)` returning the appropriate quaternion.

The controller test file owns the `std::atomic<const BalanceConfig*>` slot used as input. Tests written from spec table (lines 524-536):
- `test_zero_command_level_platform_emits_zero` — cmd `{0,0,0}`, `q = upright()`, gyro zero; expect `drive_cmd ≈ {0, 0, 0}`.
- `test_zero_command_pitched_forward_drives_backward` — cmd `{0,0,0}`, `q = pitchedForward(10°)`, gyro zero; expect `vx_out < 0` (controller drives backward to correct). One iteration only — assert sign, not magnitude.
- `test_zero_command_rolled_left_drives_right` — analogous on the roll axis.
- `test_forward_command_level_emits_forward_vx_only` — cmd `{0.5, 0, 0}`, `q = upright()`, gyro zero; expect `vx_out > 0` (after several iterations to overcome derivative), `vy_out == 0`, `omega_out == 0`.
- `test_forward_command_at_target_tilt_settles_to_zero` — pre-compute tilt = `0.30 * 0.5 = 0.15 rad`, build that quaternion, gyro zero; expect `vx_out ≈ 0` (error has converged, no integral built yet).
- `test_enter_fault_above_enter_envelope` — `q` corresponding to 65° pitch (above `envelopeEnterSin = sin(60°)`); cmd nonzero; gyro zero. Pre-load `_pitchPid` with a known integral via warm-up calls, assert post-fault entry the drivetrain.lastDrive is zero AND a follow-up update at level platform doesn't re-emit the stored integral (PID reset on entry).
- `test_in_fault_between_thresholds_stays_in_fault` — first put controller in fault (call with 65°), then call with 57° (between exit 55° and enter 60°); assert still emitting zero.
- `test_exit_fault_below_exit_envelope` — from fault, call with 50° → expect normal operation resumes (non-zero output for a nonzero command).
- `test_output_clamp_at_max_velocity` — feed an enormous setpoint via huge PID gains in the config; assert `|vx_out| ≤ maxOutputVelocity` exactly.
- `test_stop_resets_pid_state_and_calls_drivetrain_stop` — warm up PIDs with several updates, call `controller.stop()`, assert `drivetrain.stopCallCount == 1` and a subsequent `update(level, zero, zero)` emits zero (no leftover integral).

Run: expect compile failures.

**Implementation:**
- Create `src/drivetrain/controller/BalancingDrivetrainController.h` and `.cpp`. Class signature exactly as spec lines 147-166.
- Constructor stashes `_configSlot` reference, builds `_pitchPid` and `_rollPid` from initial config snapshot (or from defaults — either works since the slot is loaded at every `update()` anyway; build PIDs with placeholder bounds `[-maxOutputVel, +maxOutputVel]` from the initial snapshot). PIDs use `outputMin = -cfg.maxOutputVelocity`, `outputMax = +cfg.maxOutputVelocity`, no deadband.
- **Important — PID gains coupling:** the PID class today stores gains in its constructor and has no `setGains()` setter. The controller must either:
  1. Reach into the PID via a new `PID::setGains(kp, ki, kd)` setter (small addition; one extra test in `test_pid` for the setter), OR
  2. Reconstruct PID instances on every config swap (heap-free if PIDs are by-value members and we use placement-new; ugly).
  Pick **option 1**: add `PID::setGains(float kp, float ki, float kd)` in this commit (one-line member-mutation) with a dedicated test (`test_set_gains_changes_output`). Document the new setter in the same commit since it's controller-driven.
- `update(cmd, imu, dt)` body follows spec algorithm lines 168-201:
  1. `const BalanceConfig& cfg = *_configSlot.load(std::memory_order_acquire);`
  2. Push current cfg gains into both PIDs via `setGains` (cheap; lets live tuning take effect).
  3. `quatToBodyGravity(imu.orientation, gx, gy, gz);`
  4. `float tiltMagSin = std::sqrt(gx*gx + gy*gy);`
  5. Schmitt: `if (!_inFault && tiltMagSin > cfg.envelopeEnterSin) { _inFault = true; _pitchPid.reset(); _rollPid.reset(); }` then `if (_inFault) { if (tiltMagSin <= cfg.envelopeExitSin) _inFault = false; else { _drivetrain.drive({0,0,0}); return; } }`. Log fault transitions.
  6. `pitch_target = clamp(cfg.tiltPerVelocity * cmd.vx, -cfg.maxTiltSetpoint, +cfg.maxTiltSetpoint);`
  7. `pitch_actual = std::atan2(gx, -gz);` (and roll analogously).
  8. `float gyro_pitch_rate = cfg.gyroPitchSign * imu.gyro.y;` (and roll = `cfg.gyroRollSign * imu.gyro.x`).
  9. `float vx_out = _pitchPid.compute(pitch_target, pitch_actual, gyro_pitch_rate, dt);` (and vy_out).
  10. Clamp to `±cfg.maxOutputVelocity` (redundant with PID clamp, but the spec asks for an explicit clamp after the PID step — done for safety against PID gain glitches and to keep the spec's algorithm verbatim).
  11. `_drivetrain.drive({vx_out, vy_out, cmd.omega});`
- `stop()` calls `_drivetrain.stop()`, resets both PIDs, sets `_inFault = false`.

Run: `pio test -e native -f test_balancing_controller` and `pio test -e native -f test_pid` (for the new `setGains` test) — all green.

**Parallel?** No — depends on 1.1 (PID 4-arg), 2.1 (controller base), 2.2 (BalanceConfig), 2.3 (quatToBodyGravity).

**Done when:** all 10 balancing-controller test cases pass; `pio run -e robot` compiles.

**Commit:** `drivetrain-controller: add BalancingDrivetrainController`

Files staged: `src/drivetrain/controller/BalancingDrivetrainController.h`, `src/drivetrain/controller/BalancingDrivetrainController.cpp`, `src/control/PID.h` (adds `setGains`), `test/test_balancing_controller/test_balancing_controller.cpp`, `test/test_pid/test_pid.cpp` (one new test for `setGains`).

---

### Step 2.5 — `BalanceConfig` NVS persistence

**Inputs:** Step 2.2. Spec §"Persistence". Files: append to the same `test_balance_config` TU; persistence helpers go in a new header `src/drivetrain/BalanceConfigStorage.h` plus a `.cpp` that wraps `Preferences`.

**Tests first** — extend `test/test_balance_config/test_balance_config.cpp`. Storage tests use a `FakePreferences` stub (under `test/stubs/`) since real `Preferences` is an Arduino class. Pattern: see `test/stubs/` and FFF usage in existing native tests.
- `test_save_then_load_round_trip` — store a non-default `BalanceConfig`; load into a fresh struct; assert every field equals what was stored.
- `test_load_with_missing_blob_returns_defaults` — `FakePreferences::getBytes` returns 0; loader returns `RobotConstants::balanceConfig()`.
- `test_load_with_wrong_size_returns_defaults` — `FakePreferences::getBytes` returns `sizeof(BalanceConfig) - 1`; loader returns defaults.
- `test_load_with_wrong_version_returns_defaults` — write a blob with `version = 0`; load returns defaults.
- `test_save_writes_version_prefix` — call `save(cfg)`; inspect `FakePreferences` bytes; assert first 2 bytes are little-endian `0x01 0x00` (version = 1) followed by `sizeof(BalanceConfig)` bytes matching `cfg`.

Run: expect compile failures.

**Implementation:**
- Create `src/drivetrain/BalanceConfigStorage.h` with:
  ```cpp
  namespace BalanceConfigStorage {
      constexpr uint16_t VERSION = 1;
      bool load(BalanceConfig& out);   // returns false on miss/corrupt; out is then unchanged.
      void save(const BalanceConfig& cfg);
  }
  ```
- Create `src/drivetrain/BalanceConfigStorage.cpp` that uses `Preferences` namespace `"balance"`, key `"config"`, mirroring the BNO055 pattern (begin readOnly→getBytes→end, begin writable→putBytes→end). Blob layout: `[uint16_t version][BalanceConfig payload]`.
- For native tests, behind `#ifdef BB8_NATIVE_TEST` (a new `-D` we add to `env:native`), swap `Preferences` for the fake. Cleanest: include a header `BalanceConfigPrefs.h` that aliases the type; native build aliases to `FakePreferences`. (Or use compile-time templating — pick whichever is uglier.)
- The caller (in main, Phase 2 wire-in step) does: `BalanceConfig boot_cfg; if (!BalanceConfigStorage::load(boot_cfg)) boot_cfg = RobotConstants::balanceConfig();`.

Run: green.

**Parallel?** No — depends on 2.2 (struct).

**Done when:** 5 persistence tests pass; `pio run -e robot` compiles.

**Commit:** `drivetrain: add NVS persistence for BalanceConfig`

Files staged: `src/drivetrain/BalanceConfigStorage.h`, `src/drivetrain/BalanceConfigStorage.cpp`, `test/test_balance_config/test_balance_config.cpp`, `test/stubs/FakePreferences.h` (if not already there), `platformio.ini` (add `-DBB8_NATIVE_TEST` to `env:native` if introduced).

---

### Step 2.6 — `BalanceTuner` + atomic config-swap slot

**Inputs:** Steps 1.2, 2.2, 2.5. Spec §"RemoteSerial tuning" and §"Atomic config swap".

**Tests first** — new dir `test/test_balance_tuner/test_balance_tuner.cpp`. The tuner owns the two-buffer slot internally and exposes it via `BalanceTuner::slot()` for the controller to read. Tests don't need RemoteSerial — they call `BalanceTuner::handle(const String&)` directly. Capture output via a callback the tuner uses instead of `RemoteSerial::printf` (or a function pointer set in `setUp()`).
- `test_show_dumps_current_config` — call `handle("balance show")`; assert captured output contains each field name and value.
- `test_show_pids` — `handle("balance show pids")`; output contains the six gain values, nothing else.
- `test_set_valid_field_updates_slot` — `handle("balance set pitchKp 2.5")`; assert `slot.load()->pitchKp == 2.5f`. Subsequent `show` reflects.
- `test_set_atomic_swap_uses_spare_buffer` — call `set` twice in a row, assert the pointer returned by `slot.load()` flips between two stable addresses (and is never null).
- `test_set_invalid_negative_kp_rejected` — `handle("balance set pitchKp -1.0")`; assert slot unchanged; output contains "error".
- `test_set_invalid_envelope_order_rejected` — try to set `envelopeExitSin > envelopeEnterSin`; rejected.
- `test_set_max_tilt_above_fault_envelope_rejected` — start from defaults; `handle("balance set maxTiltSetpoint 1.10")` (≈63°); rejected because `1.10 ≥ asin(envelopeEnterSin) = asin(0.866) ≈ 1.047 rad (60°)`. Slot unchanged; output mentions the invariant. Without this rule the operator can configure commanded tilt above the fault threshold, where every full-stick command instantly faults.
- `test_set_unknown_key_rejected` — `handle("balance set foo 1.0")`; rejected.
- `test_reset_reverts_to_defaults` — set several fields, then `handle("balance reset")`; assert `*slot.load()` equals `RobotConstants::balanceConfig()`.
- `test_save_invokes_storage` — using the `FakePreferences` fake, call `handle("balance save")`; assert blob was written.
- `test_status_prints_pitch_roll_armed` — assert output contains the expected keys (controller state mock fed via a getter).

Run: expect compile failures.

**Implementation:**
- Create `src/drivetrain/BalanceTuner.h` and `.cpp`:
  - Owns two `BalanceConfig` static buffers and a `std::atomic<const BalanceConfig*>` slot pointing at one.
  - `void begin(const BalanceConfig& initial)` — copies `initial` into buffer A, points slot at A.
  - `std::atomic<const BalanceConfig*>& slot()` — for the controller constructor.
  - `void handle(const String& line)` — parses the verb. Lookup table of field-name → `&BalanceConfig::pitchKp` etc., using pointer-to-member-float. Validation rules per spec line 456-458 PLUS the safety invariant `cfg.maxTiltSetpoint < std::asin(cfg.envelopeEnterSin)` (commanded tilt must sit inside the fault envelope; otherwise full-stick instantly faults). On valid `set`, copy live → spare, mutate spare, validate, `slot.store(spare)`, `vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS + 1))` (guarded under `#ifdef ARDUINO`; on native tests, a no-op).
  - Output goes via an injectable `std::function<void(const String&)> _print` defaulting to `RemoteSerial::println` on Arduino, settable in tests.
- The arming verbs (`arm` / `disarm` / `kill` / `clearkill` / `armstate`) are handled by a separate file-static dispatcher in `main_balance_test.cpp` and `main_robot.cpp` (same shape in both; tolerate the small duplication or factor to `src/safety/ArmingHandlers.h`). For this step keep the handlers inline in the main files and write the duplication in 2.7 / 2.8. Acceptable: ~15 lines duplicated.

Run: green.

**Parallel?** No — depends on 1.2, 2.2, 2.5.

**Done when:** all 10 tuner tests pass; atomic swap verified by pointer-comparison test; `pio run -e robot` compiles.

**Commit:** `drivetrain: add BalanceTuner with atomic-swap config slot`

Files staged: `src/drivetrain/BalanceTuner.h`, `src/drivetrain/BalanceTuner.cpp`, `test/test_balance_tuner/test_balance_tuner.cpp`.

---

### Step 2.7 — `main_balance_test.cpp` bench harness + platformio envs

**Inputs:** All Phase 2 steps so far. Files: new `src/main_balance_test.cpp`; modify `platformio.ini`.

**Tests first** — no native tests; this is a top-level main. Build-and-run verification only.

**Implementation:**
- Copy `main_robot.cpp` (the version at HEAD after Step 1.6, which already uses the new `BringUp` facade — `BringUp::begin()` in `setup()` + `BringUp::tick()` in `loop()`, plus `OTA_SAFE_MODE_FOR(...)` at file scope). Reuse that bring-up sequence verbatim — do not hand-roll `Serial.begin()` / `OtaSafeMode::begin()` / `RemoteSerial::begin()` separately. The whole point of commit b195fb3 / 62bc396 / d6fbf62 is that any new `main_*.cpp` gets OTA + WebSerial console with one line.
- Strip from the copy:
  - `WebSocketCommandProducer` and its `start()` / `stop()` call sites — removed entirely.
  - The `links2004/WebSockets` lib dep — not needed (the new `[env:balance_test]` does *not* re-add it).
- **Keep** `CommandLatch<BodyVelocity>` — the bench harness writes from the RemoteSerial AsyncTCP task (Core 0) and reads from the control task (Core 1). Cross-core float assignment is not atomic on Xtensa LX6, so a non-latched `BodyVelocity g_remote_cmd` is racy (one tick can read `vx` from a new command and `vy` from the old one). The race is benign because the operator types at ~1 Hz, but the right shape matches `main_robot.cpp` and costs nothing.
- Add:
  - `OTA_SAFE_MODE_FOR("bb8-balance-test")` at file scope (anchors the OTA-safe-mode link enforcement; required by `OtaSafeMode.h`).
  - `CommandLatch<BodyVelocity> g_remote_latch;` (file-static).
  - `BalanceTuner` instance + initialization in `setup()` after `BNO055IMU.begin()` and after loading the persisted `BalanceConfig` via `BalanceConfigStorage::load`.
  - `BalancingDrivetrainController` instance wired to `g_drivetrain`, `g_imu`, and `tuner.slot()`.
  - `RemoteSerial::onMessage(handleLine)` where `handleLine`:
    - Trims input.
    - Matches `cmd <vx> <vy> <omega>` (regex-light: token by space, `strtof`) — calls `g_remote_latch.write({vx, vy, omega})`. Logs the parsed values.
    - Matches arming verbs (`arm` / `disarm` / `kill` / `clearkill` / `armstate`).
    - Falls through to `BalanceTuner::handle(line)`.
- `setup()` ordering: `BringUp::begin();` → `Wire.begin();` → motors → `g_imu.begin()` → `ArmingState::begin()` → tuner / controller construction → control-task spawn. `BringUp::begin()` must run first so that any later failure leaves the chip OTA-reachable.
- `loop()`: `BringUp::tick();` then `vTaskDelay(...)`. Same as `main_robot.cpp`.
- Control task uses the same shape as `main_robot.cpp` (staleness ramp + arming gate); reads from `g_remote_latch` instead of the WS-fed `g_latch`. The staleness window does the same job here as in `main_robot.cpp` — short-term silence ramps to zero, long-term silence triggers auto-disarm.

> **Note on env extension**: `[env:balance_test]` extends `env:wemos_d1_uno32_ota_console`, which (commit 62bc396) already pulls in WebSerial / AsyncWebServer / AsyncTCP. So the new env does not need to re-list those libs — extending the OTA-console base is sufficient.
- `platformio.ini` — add two envs:
  ```ini
  [env:balance_test]
  extends = env:wemos_d1_uno32_ota_console
  # CommandLatch.cpp is NOT excluded: the bench harness uses it on the
  # RemoteSerial → control-task path to avoid a cross-core race on BodyVelocity.
  build_src_filter = +<*> -<motor/driver/BTS7960Driver.cpp> -<main_original.cpp> -<main_blink.cpp> -<main_drivetrain_test.cpp> -<main_motor_test.cpp> -<main_imu_test.cpp> -<main_webserial_test.cpp> -<main_robot.cpp> -<input/WebSocketCommandProducer.cpp>
  lib_deps =
      ${env:wemos_d1_uno32_ota_console.lib_deps}
      madhephaestus/ESP32Encoder@0.12.0

  [env:balance_test_ota]
  extends = env:balance_test
  upload_protocol = espota
  upload_port = bb8-robot.local
  # Match env:robot_ota's 60s per-block ACK timeout (commit 47a31a9) — consumer
  # Wi-Fi hiccups overrun the espota default 10s, killing legitimate transfers.
  upload_flags =
      --auth=${sysenv.OTA_PASSWORD}
      --timeout=60
  ```
- Add `OTA_SAFE_MODE_FOR("bb8-balance-test")` at file scope.

Verify: `pio run -e balance_test` compiles. Bench-load it onto the robot (post-PR; not blocking).

**Parallel?** No — depends on 2.4 (controller) and 2.6 (tuner).

**Done when:** `pio run -e balance_test` compiles cleanly; OTA env also compiles.

**Commit:** `main: add main_balance_test bench harness + platformio envs`

Files staged: `src/main_balance_test.cpp`, `platformio.ini`.

---

### Step 2.8 — `main_robot.cpp` swap to `BalancingDrivetrainController`

**Inputs:** Phase 2 entirety. File: `src/main_robot.cpp`.

**Tests first** — no native test (integration). Bench verification after build.

**Implementation:**
1. Replace `#include "PassthroughDrivetrainController.h"` with `#include "BalancingDrivetrainController.h"` and `#include "BalanceTuner.h"` + `#include "BalanceConfigStorage.h"`.
2. Replace the global `PassthroughDrivetrainController* g_controller` with `BalancingDrivetrainController* g_controller`. Add a global `BalanceTuner g_tuner;`.
3. In `setup()` after `g_imu.begin()` and before `g_producer->start()`:
   ```cpp
   BalanceConfig boot_cfg;
   if (!BalanceConfigStorage::load(boot_cfg)) {
       boot_cfg = RobotConstants::balanceConfig();
       RemoteSerial::println("[balance] no persisted config; using defaults");
   }
   g_tuner.begin(boot_cfg);
   g_controller = new BalancingDrivetrainController(g_drivetrain, g_imu, g_tuner.slot());
   ```
4. The RemoteSerial line dispatcher (added in Step 1.6) now falls through to `g_tuner.handle(line)` after the arming verbs.
5. `PassthroughDrivetrainController` stays in the tree — `main_drivetrain_test.cpp` continues to use it. Do not delete the header or its test.

Verify:
- `pio run -e robot` compiles.
- `pio run -e drivetrain_test` still compiles (Passthrough used elsewhere).
- `pio test -e native` all green.

**Parallel?** No — depends on 2.4, 2.5, 2.6.

**Done when:** `main_robot` compiles, all native tests pass, `drivetrain_test` env compiles.

**Commit:** `main_robot: swap to BalancingDrivetrainController`

Files staged: `src/main_robot.cpp`.

---

### Step 2.9 — Bench-verification commit (docs + checklist)

**Inputs:** Spec §"Hardware verification (bench)" and §"Tuning guide".

**Tests first** — N/A (this is a documentation commit, not code).

**Implementation:**
- Create `code/body-firmware/docs/superpowers/runbooks/2026-05-13-balance-bringup.md`:
  - Bench-verification sequence from spec lines 567-579 (steps 1-8) as a numbered list.
  - Sign-convention check sub-procedure for `gyroPitchSign` / `gyroRollSign` (spec lines 191-198).
  - **Axis diagnosis section** distinguishing the two failure modes the spec conflates under "inverted axis":
    > Tilt the platform forward by hand and watch the wheels.
    > - **Wheels drive in the wrong direction on forward tilt** (sign-inverted axis) → set `gyroPitchSign = -1.0` via `balance set gyroPitchSign -1.0`. Live, no reflash.
    > - **Wheels drive *sideways* on forward tilt** (rotated mount: chip-Y points along body-X, etc.) → the two `BalanceConfig` sign knobs cannot fix this. Either physically remount the BNO055 so chip-X aligns with body-forward, or call `_bno.setAxisRemap(...)` in `BNO055IMU::begin()` and reflash. Re-run the bring-up procedure after.
    > Repeat the diagnosis for roll on body-X.
  - Tuning-guide table from spec lines 597-610 reproduced verbatim.
  - Persistence smoke test: `balance set pitchKp 2.0`, `balance save`, reboot, `balance show pids` — assert `pitchKp == 2.0`.
  - Configuration footgun list: `maxTiltSetpoint` must stay strictly below `asin(envelopeEnterSin)`; the tuner enforces this, but if the operator sees a "rejected" message on `set`, this is the invariant they violated.
- This runbook is the operator's reference during bring-up; it's separate from the spec (which lives next to the design) so the operator can update it as bench notes accumulate.

**Parallel?** Yes (doc only) — but logically comes after 2.7 / 2.8 since it references them.

**Done when:** runbook is written, scannable, and lists every bench check the spec requires.

**Commit:** `docs: add balance-controller bring-up runbook`

Files staged: `code/body-firmware/docs/superpowers/runbooks/2026-05-13-balance-bringup.md`.

---

### Step 2.10 — Phase 2 / Final reviewer subagent

Dispatch a reviewer subagent with this prompt — verbatim:

> You are reviewing the full diff of a PR that implements a balancing drivetrain controller. The spec is at `code/body-firmware/docs/superpowers/specs/2026-05-13-balancing-drivetrain-controller-design.md`.
>
> The PR has two logical phases:
> - Phase 1 (wire protocol, ArmingState, PID overload) — should already have passed an earlier review.
> - Phase 2 (BalanceConfig, BalancingDrivetrainController, BalanceTuner, NVS persistence, bench harness, wire-in).
>
> Run `git log main..HEAD --oneline` and `git diff main...HEAD --stat` to see scope.
>
> Audit:
> 1. **Spec coverage**: walk every section of the spec; for each requirement, cite the commit + file + lines that implement it. Flag any missing requirement.
> 2. **Atomic commits**: each commit one logical change. No drive-by edits. No `Co-Authored-By` trailers. No `git add -A`.
> 3. **TDD discipline**: every step has a test commit (or tests in the same commit as impl); no untested public APIs added. Tests come *before* implementation in the diff history where possible.
> 4. **Sign conventions are tested, not just runtime-verified**:
>    - `quatToBodyGravity({1,0,0,0})` → `(0, 0, -1)`. (Test exists.)
>    - `BalancingDrivetrainController` test: zero-cmd-tilted-forward emits `vx_out < 0`. (Test exists.)
>    - `BalancingDrivetrainController` test: zero-cmd-rolled-left emits `vy_out > 0` (or `< 0` per the sign convention chosen — confirm consistency).
>    - The 60°/55° Schmitt threshold is tested with explicit pre-fault → fault → recovery transitions.
> 5. **Cross-file consistency**: `update(cmd, imu, dt)` signature matches across `DrivetrainController.h`, `PassthroughDrivetrainController.h`, `BalancingDrivetrainController.h`, and both `main_*.cpp` call sites.
> 6. **Safety state**: at HEAD, boot default is `Disarmed`; OTA upload disarms; producer silence >2s auto-disarms. Trace each via commit + line.
> 7. **PR body**: uses tables, no checkbox test plan, no testing summary.
> 8. **Build & tests**: run `pio test -e native` and `pio run -e robot` and `pio run -e balance_test`. All three must pass.
>
> Report findings as a table: `requirement | implementation site | severity (BLOCKER/MAJOR/MINOR/NIT) | suggested fix`. If you find no blockers, say so explicitly. Do not modify code.

If the reviewer flags blockers, address them as fixup commits before opening / merging the PR.

---

## PR body (paste at PR open time)

Use this skeleton (tables only, no checkboxes, no testing summary):

```markdown
## Summary

Adds a tilt-balancing inner loop for the BB-8 drivetrain. Replaces `PassthroughDrivetrainController` with `BalancingDrivetrainController` behind an explicit arm/disarm/kill control plane on the existing WebSocket wire protocol. Live PID tuning over RemoteSerial with NVS persistence.

## Phases

| Phase | Scope |
| --- | --- |
| 1 | PID 4-arg overload, ArmingState, sigil-dispatch parser, WS control-frame dispatcher, OTA-disarm hook, control-task arming gate. End-state still uses passthrough controller; behaviour unchanged for drivers (system boots `Disarmed`). |
| 2 | BalanceConfig + NVS, quat→gravity utility, BalancingDrivetrainController, BalanceTuner, `main_balance_test` bench harness + platformio envs, wire-in to `main_robot.cpp`. |

## New wire format

| Frame | Example | Effect |
| --- | --- | --- |
| Velocity (legacy) | `1.0,0.5,0.3` | parsed as `BodyVelocity` (back-compat) |
| Velocity (prefixed) | `v1.0,0.5,0.3` | same as legacy |
| Control | `c:arm` / `c:disarm` / `c:kill` / `c:clearkill` | invokes `ArmingState` transition |

## Safety model

| State | Drive command | Controller call | Drivetrain |
| --- | --- | --- | --- |
| Armed | user input | `update(cmd, imu, dt)` | runs |
| Disarmed | hard-zero | skipped | runs (PIDs hold motors at 0) |
| Killed | n/a | `stop()` | hard brake |

OTA upload starts → `disarm()` automatically. Producer silence >2 s → auto-disarm.

## Bench bring-up

See `docs/superpowers/runbooks/2026-05-13-balance-bringup.md`.
```

## Verification (end-to-end, before opening PR)

| Command | Expected |
| --- | --- |
| `pio test -e native` | all green |
| `pio run -e robot` | compiles |
| `pio run -e drivetrain_test` | compiles (Passthrough still works) |
| `pio run -e balance_test` | compiles |
| Bench: boot, send `c:arm`, joystick forward | platform tilts forward, sphere rolls |
| Bench: tilt platform past 60° | drive halts, fault logged; return below 55° → resume |
| Bench: `c:kill` during motion | wheels brake immediately |
| Bench: OTA upload while armed | `[arming] Armed → Disarmed` log at upload start |
| Bench: `balance set pitchKp 2.0`, `balance save`, reboot, `balance show pids` | pitchKp == 2.0 |
