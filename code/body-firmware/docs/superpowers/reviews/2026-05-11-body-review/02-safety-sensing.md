# Safety/Correctness × Sensing & Input — Cell 2

Scope: `src/imu/*`, `src/input/*`, and their feed into `controlTask` in `main_robot.cpp`. Lens: safety & correctness.

## P0 — block on-robot use

```json
{
  "id": "C2-P0-01",
  "title": "Parser accepts NaN/Inf and propagates them through latch into PID/motors",
  "file": "src/input/CommandFrameParser.h",
  "line_range": "46-58",
  "quote": "float vx = std::strtof(p, &end);",
  "risk": "`std::strtof` parses 'nan', 'NaN', 'inf', '+inf', '-inf' (case-insensitive) as valid floats. Parser returns ok and `_latch.write(result.value)` (WebSocketCommandProducer.cpp:161) pushes `BodyVelocity{NaN,...}` to control. `last_known.vx * scale` (main_robot.cpp:179) yields NaN; `OmniKinematics::toWheelRPMs` propagates it (saturation scan `a > maxAbs` is false for NaN). PID has `BB8_ASSERT(std::isfinite(setpoint))` (PID.h:16) but `BB8_ASSERT` is a no-op in all production envs (CONTEXT §7). NaN poisons `_integral` (NaN > integralMax is false → never clamped) and persists through reboot of clean commands — analogWrite output undefined.",
  "suggested_fix": "Reject after each strtof if `!std::isfinite(v)`; add `FrameParseError::NonFinite`. Defensive: assert finite in `OmniDrivetrain::drive`.",
  "test_gap": "test_command_frame_parser.cpp has zero cases for 'nan'/'inf'/'+inf'/'-inf'. The 10 RUN_TESTs (happy/negative/zero/empty/too_long/missing_first_comma/non_numeric_vy/missing_omega/trailing_extra/trailing_newline) never probe strtof's dangerous tokens."
}
```

```json
{
  "id": "C2-P0-02",
  "title": "No magnitude clamp on parsed velocity — any LAN client commands any value",
  "file": "src/input/CommandFrameParser.h",
  "line_range": "46-58",
  "quote": "return {BodyVelocity{vx, vy, omega}, FrameParseError::None};",
  "risk": "Nothing bounds `vx/vy/omega`. Protocol is unauthenticated `ws://` (WebSocketCommandProducer.h:22-23). A client sending `1e9,1e9,1e9` is accepted. IK saturates per wheel, but staleness then bleeds `last_known` over 200 ms as a max-thrust ramp rather than effectively-zero — runaway hazard on a slope.",
  "suggested_fix": "Define `kMaxVxMps/kMaxVyMps/kMaxOmegaRadps` in `RobotConstants.h`; clamp or reject in parser. Defensive clamp also in `OmniDrivetrain::drive`.",
  "test_gap": "No test asserts a rejected/clamped over-range command. `test_happy_path` uses (1,2,3) — inside range."
}
```

```json
{
  "id": "C2-P0-03",
  "title": "Mid-run BNO055 I²C failure is silent — stale/garbage flows, no recovery",
  "file": "src/imu/BNO055IMU.h",
  "line_range": "33-80",
  "quote": "IMUReading read() override {",
  "risk": "`read()` has no error path. Adafruit_BNO055 returns zero/last-state silently on bus error. `controlTask` reads every 10 ms (main_robot.cpp:186). Today passthrough ignores IMU, bounded blast radius — but `docs/controller-discussion.md`'s tilt stabilizer will trust this data; `Quat{1,0,0,0}` is identical for 'no-begin' and 'real-upright', so a dead bus reads as upright while the robot tilts. Clock-stretch hang → TWDT at 1 s (good); silent stale return (bad).",
  "suggested_fix": "Return `bool`/`std::optional<IMUReading>` or add `IMUReading::valid`. Track time-since-last-successful-read; control loop treats stale IMU like stale command (zero + brake).",
  "test_gap": "test_bno055_imu.cpp has no case for `getVector`/`getQuat` failing mid-run. `test_read_before_begin_zeroed` proves pre-begin identity but not post-begin I²C failure."
}
```

## P1 — likely bug / safety-relevant test gap

```json
{
  "id": "C2-P1-01",
  "title": "Staleness path zeroes setpoint but never brakes — coast on slope = drift",
  "file": "src/main_robot.cpp",
  "line_range": "170-187",
  "quote": "BodyVelocity drive_cmd{0.0f, 0.0f, 0.0f};",
  "risk": "On WS silence > 200 ms, `drive_cmd` is zero but controller.update still runs → PIDs against 0-RPM setpoint → L298N at PWM=0 = *coast* (CONTEXT §4). main_robot.cpp:6-9 says 'hard stop' but the path is hard-coast. `OmniDrivetrain::stop()` (brake + reset) is never invoked at runtime (CONTEXT §9.11).",
  "suggested_fix": "Once `age > STALENESS_TIMEOUT_MS` (or `!have_seen_fresh`), call `g_controller->stop()` instead of feeding zeros. Latch a `was_stopped` flag so it isn't called every tick.",
  "test_gap": "No integration test exercises staleness → motor-brake transition. test_passthrough_controller does not assert brake-on-stale."
}
```

```json
{
  "id": "C2-P1-02",
  "title": "WS task alive but starved of frames — heartbeat OK, no commands, no producer fault",
  "file": "src/input/WebSocketCommandProducer.cpp",
  "line_range": "99-115",
  "quote": "esp_task_wdt_add(NULL);",
  "risk": "ws_prod pets TWDT regardless of frame flow; heartbeat keeps `_connected==true`. If the client hangs but TCP stays alive, `connected()` is true and no commands arrive. Control's staleness net catches it, but producer never disconnects a silent peer to free the single-client slot.",
  "suggested_fix": "Track `_lastFrameMs`; if >2 s since last text frame on active client, disconnect them. Log a one-shot warning from control when `now - last_fresh_ms > STALENESS_TIMEOUT_MS`.",
  "test_gap": "No test for 'producer task alive, no frames arriving'. test_mock_command_producer never exercises starvation."
}
```

```json
{
  "id": "C2-P1-03",
  "title": "Identical-command repeat is treated as fresh — stuck stream never goes stale",
  "file": "src/main_robot.cpp",
  "line_range": "162-168",
  "quote": "if (fresh.has_value()) {",
  "risk": "`last_fresh_ms` resets on any latch hit, regardless of value change. A buggy client retransmitting `0.5,0,0` at 100 Hz keeps the loop 'fresh' indefinitely with no liveness signal in the wire protocol (no seq, no timestamp). All-zero stuck stream is safe but accidental; non-zero stuck stream is hazardous.",
  "suggested_fix": "Add a monotonic seq or wall-time field: `\"vx,vy,omega,seq\"`. Reject frames where seq did not advance.",
  "test_gap": "test_command_frame_parser.cpp:test_trailing_extra_field_accepted documents that the 4th field is silently discarded — so adding seq is forward-compatible but currently unenforced. No test covers stuck-identical stream."
}
```

```json
{
  "id": "C2-P1-04",
  "title": "Quaternion consumed without normalization or finiteness check",
  "file": "src/imu/BNO055IMU.h",
  "line_range": "39-43",
  "quote": "r.orientation = Quat{static_cast<float>(q.w()),",
  "risk": "Direct double→float cast, no normalize, no finiteness check. `test_unnormalized_quaternion_not_renormalized` (test_bno055_imu.cpp:214-220) actively asserts unnormalized quats pass through. BNO055 can emit `(0,0,0,0)` after begin/cal glitch. A future controller doing `acos`/`atan2` will divide by `||q||=0` or produce NaN. Today's passthrough dodges it.",
  "suggested_fix": "In `read()` compute `n=sqrt(w²+x²+y²+z²)`; if `n<1e-3 || !isfinite(n)` flag invalid or substitute identity. Otherwise scale.",
  "test_gap": "Existing test enshrines the buggy contract — replace once fixed."
}
```

```json
{
  "id": "C2-P1-05",
  "title": "`isCalibrated()` exists but no consumer gates on it",
  "file": "src/imu/BNO055IMU.h",
  "line_range": "82-84",
  "quote": "bool isCalibrated() {",
  "risk": "Calibration status is in `IMUReading` and exposed via `isCalibrated()`, but neither `main_robot.cpp` nor the controller consults it. A 0/3/2/1 reading is treated identically to 3/3/3/3. Auto-save logic is correct; consumer is missing.",
  "suggested_fix": "Require `gyro >= 2` before IMU-based control modes; propagate validity into `IMUReading::valid`.",
  "test_gap": "test_partial_calibration_reported_accurately (test_bno055_imu.cpp:320-336) confirms partial-cal is reported but never that a consumer reacts."
}
```

```json
{
  "id": "C2-P1-06",
  "title": "Parser tolerance is wider than tested — whitespace/sci-notation/hex-float not pinned",
  "file": "src/input/CommandFrameParser.h",
  "line_range": "46-55",
  "quote": "if (end == p || *end != ',') return {{}, FrameParseError::InvalidVx};",
  "risk": "`std::strtof` per C standard skips leading whitespace, accepts `+`/`-`, scientific notation, hex floats. None is wrong, but none is tested; a future stricter rewrite regresses silently. Embedded null short-circuits strtof; current behavior (parse error at vx) is correct but undocumented.",
  "suggested_fix": "Pin the grammar with explicit tests for: leading whitespace, '+1', '1e3', '0x1p2', embedded null, multi-line, '-0', locale decimal.",
  "test_gap": "10 RUN_TESTs cover 8 structural cases + 2 length boundaries. None of the above tokens."
}
```

```json
{
  "id": "C2-P1-07",
  "title": "`MockCommandProducer` flags non-atomic — interface contract ambiguous on threading",
  "file": "src/input/MockCommandProducer.h",
  "line_range": "17-30",
  "quote": "void start() override { _started = true; }",
  "risk": "Real WS producer uses `std::atomic<bool>` on `_connected`/`_running`. Mock uses plain `bool`. Any future test that drives the mock from a separate task inherits a data race the interface never warned about.",
  "suggested_fix": "Use `std::atomic<bool>` in the mock; or document `connected()` as snapshotting + cross-thread-safe on the interface."
}
```

```json
{
  "id": "C2-P1-08",
  "title": "`CommandProducer` exposes no way to surface invalid frames",
  "file": "src/input/CommandProducer.h",
  "line_range": "11-18",
  "quote": "virtual bool connected() const = 0;",
  "risk": "Interface offers only `connected()`. WS impl counts `_parseFailCount` (WebSocketCommandProducer.cpp:150) but never publishes it. Client typo storm at 100 Hz is indistinguishable from 'no signal' to the app: latch empty → staleness → coast.",
  "suggested_fix": "Add `uint32_t parseFailures() const` and `uint32_t framesSinceLastValid() const`; control loop logs/raises on first nonzero observation."
}
```

## P2 — minor

```json
{
  "id": "C2-P2-01",
  "title": "`IMUField operator&` returns bool — composing two masks with `&` is ill-typed",
  "file": "src/imu/IMUReading.h",
  "line_range": "23-29",
  "quote": "inline bool operator&(IMUField a, IMUField b) {",
  "risk": "Asymmetric bitmask operators (CONTEXT §9.4). Low-risk today; future caller doing mask intersection gets a type error or implicit-conversion surprise.",
  "suggested_fix": "Return `IMUField`; add `bool any(IMUField)` for the boolean reduction."
}
```

```json
{
  "id": "C2-P2-02",
  "title": "Unused IMU read per tick costs 4 I²C transactions for nothing",
  "file": "src/main_robot.cpp",
  "line_range": "186-187",
  "quote": "IMUReading imu_reading = g_imu.read();",
  "risk": "CONTEXT §9.10 calls this out. Passthrough discards the result; worst-case I²C cost eats meaningfully into a 10 ms budget and raises TWDT panic risk on bus stretch.",
  "suggested_fix": "Gate on `g_controller->wantsIMU()`, or move IMU read into a slower dedicated task with its own latch."
}
```

```json
{
  "id": "C2-P2-03",
  "title": "No test asserts BNO055-begin-failure → ESP.restart path",
  "file": "src/main_robot.cpp",
  "line_range": "221-224",
  "quote": "FATAL: BNO055 init failed",
  "risk": "test_begin_failure_returns_false (test_bno055_imu.cpp:94) verifies the IMU returns false, but the main-side reaction is untested. Untestable on native without harness.",
  "suggested_fix": "Document the manual-test step; extract the reaction policy into a strategy for unit testability."
}
```

## Observations — prose

- **Parser is the trust boundary; trusts too much.** Only thing between an unauthenticated `ws://` peer and motor drivers. Validates structure, not values. NaN-accepting strtof + no magnitude clamp + `BB8_ASSERT` no-op in production means one malformed-but-strtof-valid frame can degrade control state until reboot. The 200 ms ramp is *not* sufficient — `NaN * scale == NaN`. C2-P0-01 + C2-P0-02 are the cell's main findings.

- **Staleness story is half-built.** Producer enforces single-client + heartbeat. Control correctly stops gating on `connected()` (comment at main_robot.cpp:170-172 captures the reasoning). But "command absent" zeroes the setpoint instead of stopping the drivetrain; with L298N PWM=0 that's coast, not brake. C2-P1-01 is the highest-impact P1.

- **IMU pipeline has no health signal.** `IMUReading` lacks a `valid` flag; `read()` returns the same shape whether the bus is alive, recalibrating, or hung. Today's passthrough controller bounds the risk; the next controller (`docs/controller-discussion.md`) will inherit a sensor with no failure semantics. C2-P0-03 + C2-P1-04 + C2-P1-05 are the same gap — fix together by adding `IMUReading::valid` (or `std::optional<IMUReading>`).

- **Test coverage is correctly-shaped but mis-targeted.** `test_command_frame_parser.cpp` has 10 cases but misses every safety-relevant strtof tolerance. `test_bno055_imu.cpp` has 27 cases but `test_unnormalized_quaternion_not_renormalized` enshrines the unsafe contract. `test_mock_command_producer.cpp` cannot inject NaN, but the interface contract does not prohibit it.

- **Counters present but unwired.** `_frameCount` / `_parseFailCount` are atomic and logged to Serial only. Incremental fix: expose via `CommandProducer` and have control log/raise on first nonzero observation in a window.

- **Single-active-client policy is well-implemented.** `compare_exchange_strong` on `_activeClient` plus defense-in-depth drop at `WStype_TEXT` (WebSocketCommandProducer.cpp:146) is the right shape. No rate-limit on `WStype_CONNECTED` (CONTEXT §9.16) — track separately.
