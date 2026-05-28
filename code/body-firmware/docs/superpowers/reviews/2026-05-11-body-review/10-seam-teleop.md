# Seam × Teleop Pipeline — Cell 10

Scope: end-to-end teleop seam from WS byte arrival to motor PWM. Files:
`src/input/WebSocketCommandProducer.{h,cpp}`, `src/input/CommandFrameParser.h`,
`src/util/sync/CommandLatch.{h,cpp}`, `src/main_robot.cpp` (controlTask + setup),
`src/drivetrain/controller/PassthroughDrivetrainController.h`, `src/drivetrain/OmniDrivetrain.h`,
`src/drivetrain/BodyVelocity.h`. Tests: `test/test_command_latch/*`,
`test/test_command_frame_parser/*`.

The seam handoffs in order:

```
WS bytes (Core0, prio2) -> parseCommandFrame -> CommandLatch.write (xQueueOverwrite)
   -> [cross-core queue len=1] -> controlTask.read (Core1, prio4, 100Hz)
   -> staleness ramp -> PassthroughDrivetrainController -> OmniDrivetrain.drive
   -> OmniKinematics.toWheelRPMs -> PID*3 -> L298NDriver::setOutput -> analogWrite
```

Threading: producer task and onWsEvent run in the **same** ws_prod task body
(`taskBody` calls `_server->loop()` which dispatches onEvent synchronously, see
`WebSocketCommandProducer.cpp:106`), so there is no second WS callback thread to
worry about. The only cross-core edge is the latch.

---

## P0 — block on-robot use

```json
{
  "id": "C10-P0-01",
  "title": "Reconnect serves stale pre-disconnect command for up to 200 ms — motors lurch on new connect",
  "file": "src/main_robot.cpp",
  "line_range": "150-183",
  "quote": "BodyVelocity last_known{};\n    uint32_t     last_fresh_ms = 0;\n    bool         have_seen_fresh = false;\n\n    TickType_t lastWake = xTaskGetTickCount();\n    const TickType_t period = pdMS_TO_TICKS(CONTROL_PERIOD_MS);\n\n    UBaseType_t stack_hwm_min = static_cast<UBaseType_t>(-1);\n    uint32_t    hwm_iter = 0;\n\n    while (true) {\n        // 1) Pull freshest command from latch.\n        auto fresh = g_latch->read();\n        const uint32_t now = millis();\n        if (fresh.has_value()) {\n            last_known      = *fresh;\n            last_fresh_ms   = now;\n            have_seen_fresh = true;\n        }",
  "risk": "On WS disconnect, `WebSocketCommandProducer::onWsEvent` (DISCONNECTED case, lines 134-142) **does not publish a zero command to the latch** — it only clears `_activeClient` / `_connected` atomics. The control task ignores `connected()` by design (comment at main_robot.cpp:170-172). The staleness ramp protects in steady state: if the previous client was holding `vx=2.0` then crashed, `last_known` stays at `vx=2.0` and the ramp scales it down over 200 ms. BUT — and this is the seam bug — if a **new** client connects within that 200 ms window and sends its first frame, the new frame overwrites `last_known` and `last_fresh_ms` resets to `now`. The ramp is computed from `last_known` (the latest fresh value), not from the in-flight ramp output. The new operator's intent is `vx=0` (their joystick is at rest), but they send `vx=0.05` (small noise) — `last_known` becomes `0.05`, age becomes 0, and the motors immediately jump from the ramped-down value to full `0.05` with **no transition**. Worse: if the previous client left at `vx=2.0` and the new client opens with literally any frame, the latch's previous-disconnect-era `2.0` may have been overwritten — but in the case where the new client's first frame arrives BEFORE the next control tick (10 ms window), the consumer never observed the disconnected-era stale-ramp; the new client effectively inherits whatever motors-are-spinning state the previous client left, and the only thing protecting reaction is the 100 Hz control tick rate (10 ms motor inertia gap). A more dangerous concrete case: previous client disconnects at t=0 holding `vx=2.0`. New client connects at t=50ms, sends `vx=0` at t=51ms. Latch now holds `BodyVelocity{0,0,0}` and `last_fresh_ms=51`. Good. But: between t=0 and t=50ms the ramp was scaling `vx=2.0 * (1 - age/200)`. At t=50ms that scaled value is `~1.5`. PID setpoint at t=50ms was `1.5*max_wheel_rpm` — motors were still spinning. At t=51ms the new command (`0`) jumps setpoint to 0, but the PID integral and motor inertia carry the previous wheel rotation. This is benign because the PID drives to 0 RPM, but the seam allows a hostile or buggy new client to retain authority by reconnecting under the disconnect-ramp window and immediately commanding non-zero motion that the previous client had been ramping down from. There is no 'fresh-connection-mandatory-zero' gate.",
  "suggested_fix": "On `WStype_CONNECTED` for the first client (compare_exchange success) publish a zero `BodyVelocity` to the latch BEFORE the client's first frame can arrive. Equivalently: on `WStype_DISCONNECTED` for the active client, publish `BodyVelocity{0,0,0}` to the latch — this forces the ramp to start from zero so the next connect can only add to a quiescent state. Currently disconnect publishes nothing and reconnect inherits the previous client's latched value plus whatever the new client sends.",
  "test_gap": "No test in test_command_latch.cpp exercises a disconnect-reconnect sequence. No integration test under `[env:native]` simulates the controlTask loop across a producer state change. Add `test_disconnect_publishes_zero` against WebSocketCommandProducer (requires stub WebSocketsServer)."
}
```

```json
{
  "id": "C10-P0-02",
  "title": "Disconnect does not propagate to motors-off — only staleness timer does (200 ms blind window)",
  "file": "src/input/WebSocketCommandProducer.cpp",
  "line_range": "134-142",
  "quote": "case WStype_DISCONNECTED:\n            // Only clear state if the disconnecting client is the active one.\n            // A rejected client's later disconnect must not unset _connected.\n            if (_activeClient.load() == clientNum) {\n                _activeClient = kNoClient;\n                _connected = false;\n                Serial.printf(\"[ws] client %u disconnected\\n\", clientNum);\n            }\n            break;",
  "risk": "From operator-perspective end-to-end latency for 'I closed the tab, robot should stop': WS DISCONNECTED event fires immediately on a clean close, but on a half-open TCP (cable yanked, WiFi router pulled), arduinoWebSockets' heartbeat config is `enableHeartbeat(2000, 1000, 2)` — ping every 2s, fail after 1s, 2 retries — so detection is **~6 seconds**. During those 6 seconds the latch still holds the last good command and the control task happily runs at full `last_known` for 200 ms minus age then ramps for 200 ms — but if the operator was sending commands at 100 Hz, every 10 ms a fresh command resets `last_fresh_ms` to `now` and the ramp **never engages**. Wait — that's the normal flow. But in the half-open case the LOCAL socket buffer may still have frames the server hasn't drained, AND on a cable-yank the operator stops sending (their app gets a TCP error or no ACK) but the embedded side has no idea for ~6s. So actual behavior on cable-yank: operator app fails to send, ws_prod task sees nothing, latch goes stale at t=200ms, ramp engages, motors at zero by t=400ms. That's fine for cable-yank. But for: operator's browser tab is frozen (laptop sleep / GC stall) and not sending but the TCP connection is technically alive — same scenario, staleness handles it. The real hole: **the staleness window means even an explicit 'STOP' button-press has up to 200ms of additional ramping latency vs. an in-band stop command (which doesn't exist in the protocol — see C10-P1-03)**. There's no zero-latency stop primitive. An operator pressing E-stop sends `0,0,0`, which still has to go through staleness (instant-zero because `last_known = {0,0,0}` and scale = 1.0, so PID setpoint immediately drops to 0). OK — that's actually fine, the ramp is on `last_known` not on the output, and `last_known.* * 1.0 = 0`. So in-band stop works. The real P0 risk is: **on `g_producer->stop()` from the OTA path (main_robot.cpp:109), the producer task exits and stops draining the WS server, but it never publishes a final zero frame to the latch**. The control task will keep running `last_known * scale` until age >= 200ms. For a 200ms flash window this is acceptable but the producer destructor + OTA stop should explicitly publish zero so the ramp begins immediately rather than from whatever frame happened to be latched last.",
  "suggested_fix": "In `WebSocketCommandProducer::stop()` (line 68), before deleting the task and server, call `_latch.write(BodyVelocity{0,0,0})`. This makes both OTA-initiated halt and explicit shutdown immediately start the ramp from zero rather than relying on the absence of new frames (which only triggers staleness after 200ms).",
  "test_gap": "No test verifies that producer stop() publishes a quiescent value. No test exercises the OTA-stop -> control-task-continues code path."
}
```

---

## P1 — significant risk, fix before extended runs

```json
{
  "id": "C10-P1-01",
  "title": "Staleness ramp scales omega identically to vx/vy — yaw rate has same 200 ms blind window as translation",
  "file": "src/main_robot.cpp",
  "line_range": "173-183",
  "quote": "BodyVelocity drive_cmd{0.0f, 0.0f, 0.0f};\n        if (have_seen_fresh) {\n            const uint32_t age = now - last_fresh_ms;\n            if (age < STALENESS_TIMEOUT_MS) {\n                const float scale = 1.0f - static_cast<float>(age) /\n                                            static_cast<float>(STALENESS_TIMEOUT_MS);\n                drive_cmd = { last_known.vx * scale,\n                              last_known.vy * scale,\n                              last_known.omega * scale };\n            }\n        }",
  "risk": "Context §9.9 flagged this. A rotating-only command (`vx=0, vy=0, omega=2.0`) is treated identically to a translating-only command. In a sphere-bot, yaw drift while ramping is arguably **more** dangerous than translation drift: a stale rotation continues to spin the drive platform up the inside of the sphere wall along an axis that the operator no longer wants. The 200 ms linear-to-zero ramp is appropriate for translation (where momentum is the bigger risk and abrupt zero would slip wheels), but rotation has lower mechanical inertia and tighter latency demand. A separate `OMEGA_STALENESS_RAMP_MS` (e.g. 50ms) would let yaw decay faster than translation. Currently both are coupled to a single constant `STALENESS_TIMEOUT_MS = 200` (RobotConstants.h).",
  "suggested_fix": "Split into `TRANSLATION_STALENESS_MS` and `ROTATION_STALENESS_MS`, compute two scale factors, apply per-channel. Or, simpler: keep one timer but ramp omega to zero in 50 ms while vx/vy use 200 ms.",
  "test_gap": "No native-env test exercises the staleness ramp at all — the logic lives inline in controlTask and isn't extracted into a testable function."
}
```

```json
{
  "id": "C10-P1-02",
  "title": "Staleness ramp is computed against last_known, not against the previously-emitted drive_cmd — new arrival cancels in-progress ramp instantly",
  "file": "src/main_robot.cpp",
  "line_range": "160-183",
  "quote": "while (true) {\n        // 1) Pull freshest command from latch.\n        auto fresh = g_latch->read();\n        const uint32_t now = millis();\n        if (fresh.has_value()) {\n            last_known      = *fresh;\n            last_fresh_ms   = now;\n            have_seen_fresh = true;\n        }\n\n        // 2) Drive command via staleness alone. ...\n        BodyVelocity drive_cmd{0.0f, 0.0f, 0.0f};\n        if (have_seen_fresh) {\n            const uint32_t age = now - last_fresh_ms;\n            if (age < STALENESS_TIMEOUT_MS) {\n                const float scale = 1.0f - static_cast<float>(age) /\n                                            static_cast<float>(STALENESS_TIMEOUT_MS);\n                drive_cmd = { last_known.vx * scale,\n                              last_known.vy * scale,\n                              last_known.omega * scale };\n            }\n        }",
  "risk": "When a new frame arrives, `last_known` is overwritten and `last_fresh_ms = now` -> age = 0, scale = 1.0. Imagine operator sends `vx=2.0` at t=0, then briefly disconnects (network blip) so no frames arrive from t=0 to t=180ms. The ramp has scaled the drive_cmd to `2.0 * (1 - 180/200) = 0.2`. At t=181ms a new frame arrives with `vx=2.0` (operator still holding the stick). `last_known = 2.0`, `age = 0`, `scale = 1.0`, `drive_cmd.vx = 2.0`. Motors jump from setpoint 0.2 to setpoint 2.0 in one 10ms tick. That's a 10x setpoint step — the PID will saturate to `+1.0` immediately. Whether this is dangerous depends on motor inertia, but the user-visible behavior is a hard surge after a network blip rather than a smooth catch-up. Compare with a ramp computed against the *last emitted* drive_cmd: on fresh arrival, ramp from `prev_drive_cmd` toward `new_cmd` over a fixed window. That would smooth the rejoin.",
  "suggested_fix": "Track `prev_drive_cmd` across iterations. On fresh arrival, instead of `drive_cmd = last_known * 1.0`, blend: `drive_cmd = prev_drive_cmd + clamp((last_known - prev_drive_cmd), max_step)`. Alternatively cap the per-tick delta on PID setpoint inside OmniDrivetrain.",
  "test_gap": "No test for the disconnection-and-rejoin transient. No test of per-tick setpoint slew rate."
}
```

```json
{
  "id": "C10-P1-03",
  "title": "No sequence number / frame ID — dropped commands are undetectable, replay/reorder is invisible",
  "file": "src/input/CommandFrameParser.h",
  "line_range": "32-58",
  "quote": "inline constexpr size_t kCommandFrameMaxLen = 64;\n\ninline FrameParseResult parseCommandFrame(const uint8_t* payload, size_t length) {\n    if (length == 0 || length >= kCommandFrameMaxLen) {\n        return {{}, FrameParseError::EmptyOrTooLong};\n    }\n\n    char buf[kCommandFrameMaxLen];\n    for (size_t i = 0; i < length; ++i) buf[i] = static_cast<char>(payload[i]);\n    buf[length] = '\\0';\n\n    char* p = buf;\n    char* end = nullptr;\n\n    float vx = std::strtof(p, &end);\n    if (end == p || *end != ',') return {{}, FrameParseError::InvalidVx};\n    p = end + 1;\n\n    float vy = std::strtof(p, &end);\n    if (end == p || *end != ',') return {{}, FrameParseError::InvalidVy};\n    p = end + 1;\n\n    float omega = std::strtof(p, &end);\n    if (end == p) return {{}, FrameParseError::InvalidOmega};\n    // Trailing chars (whitespace, newline, extra fields) intentionally ignored.\n\n    return {BodyVelocity{vx, vy, omega}, FrameParseError::None};\n}",
  "risk": "Protocol is `'vx,vy,omega'` with no sequence number, timestamp, or monotonic ID. With a length-1 latch and overwrite-semantics, drops are intentional but **silent**. From the operator's perspective there is no way to detect: (a) frames dropped due to TCP retransmit buffering at high rate, (b) frames dropped due to control loop temporarily falling behind (won't happen at 100Hz / 50-100Hz send rate, but could if loop stalls on I2C), (c) replay if a future client (or compromised hostile client) loops the same frame. Cell §9 flagged that this is `ws://` unauthenticated. The single-client policy mitigates (c) somewhat — only one client at a time — but the producer logs `frames=%u parse_fail=%u` every 1000 frames (line 165-168) and there's no per-second telemetry of accepted-rate, no detection of `received_seq != prev_seq + 1`. For debugging field issues ('motors stuttered at t=12s'), there is no way to tell if the operator sent 100 frames or 30. The forward-compat trailing fields (line 56) make adding a sequence number a non-breaking change.",
  "suggested_fix": "Extend protocol to `'seq,vx,vy,omega'` and reject frames with seq <= last_seq (rejecting reorder). Log per-second accepted/dropped/rejected counts. Forward-compat already supports this via the trailing-fields-ignored rule.",
  "test_gap": "No test for protocol forward compatibility beyond `test_trailing_extra_field_accepted`. No replay-protection test."
}
```

```json
{
  "id": "C10-P1-04",
  "title": "Worst-case end-to-end latency is ~16 ms; spike potential to ~30+ ms under I2C contention",
  "file": "src/main_robot.cpp",
  "line_range": "144-206",
  "quote": "void controlTask(void* /*arg*/) {\n    // Subscribe to the Task Watchdog. ...\n    esp_task_wdt_add(NULL);\n\n    BodyVelocity last_known{};\n    uint32_t     last_fresh_ms = 0;\n    bool         have_seen_fresh = false;\n\n    TickType_t lastWake = xTaskGetTickCount();\n    const TickType_t period = pdMS_TO_TICKS(CONTROL_PERIOD_MS);",
  "risk": "Latency budget byte-to-PWM, worst case: (1) WS task vTaskDelay 1ms between server->loop() calls (WebSocketCommandProducer.cpp:108) — frame arrival could wait up to 1ms before parser sees it; (2) parse+latch.write is O(microseconds); (3) cross-core queue handoff — `xQueueOverwrite` from Core0 to Core1 is fast (a few µs) but the consumer is **not** waiting on the queue (non-blocking `xQueueReceive` with timeout=0, CommandLatch.h:31), so the new value waits for the next control tick. With `vTaskDelayUntil` and a 10ms period, worst-case wait is ~10ms; (4) within the tick, `g_imu.read()` is **blocking I2C** (CONTEXT §9.10) and reads 4 fields — at 100kHz I2C and ~6 bytes per vector that's ~5ms in the worst case under bus contention; (5) controller.update -> drivetrain.drive -> toWheelRPMs -> PID*3 -> setOutput -> analogWrite: sub-millisecond. **Worst-case end-to-end: 1ms (ws delay) + 10ms (next-tick wait) + 5ms (IMU blocking) = ~16ms**. Spike scenarios: (a) I2C clock-stretch on BNO055 calibration cycles can push the read() to 20+ms — pushes total to ~30ms+; (b) if the Arduino loopTask happens to run on Core1 and OTA is handling an update, the control task's priority 4 preempts it but only at scheduler granularity (1ms tick). For teleop at 50-100Hz operator send rate, 30ms is one to three frames behind — visible to the operator as input lag but not a safety issue per se. The wider concern is that the staleness window (200ms) is 6-10x the worst-case latency, which is a reasonable margin, but **the latency budget is not asserted anywhere** — no per-tick timing telemetry exists.",
  "suggested_fix": "(1) Add per-tick max-latency log: time from `g_latch->read()` returning a value to motor->setSpeed completing. Log p99 every 1000 ticks. (2) Consider blocking-with-short-timeout in CommandLatch::read (e.g. ticksToWait=1) so a frame arriving mid-tick doesn't wait the full 10ms — but this conflicts with the deterministic 100Hz schedule. (3) Make IMU read non-blocking or move to a separate task — the passthrough controller doesn't use it; the read is dead code for now (CONTEXT §9.10).",
  "test_gap": "No timing test, no latency benchmark, no instrumentation. test_command_latch.cpp tests semantics but not throughput or cross-core."
}
```

```json
{
  "id": "C10-P1-05",
  "title": "CommandLatch tests are single-threaded native — no cross-core race coverage",
  "file": "test/test_command_latch/test_command_latch.cpp",
  "line_range": "9-80",
  "quote": "static CommandLatch<BodyVelocity>* latch = nullptr;\n\nvoid setUp() {\n    latch = new CommandLatch<BodyVelocity>();\n}\n\nvoid tearDown() {\n    delete latch;\n    latch = nullptr;\n}\n\nvoid test_empty_read_returns_nullopt() {\n    auto v = latch->read();\n    TEST_ASSERT_FALSE(v.has_value());\n}\n\nvoid test_write_then_read_returns_value() {\n    BodyVelocity in{1.0f, 2.0f, 3.0f};\n    latch->write(in);\n    auto v = latch->read();\n    TEST_ASSERT_TRUE(v.has_value());",
  "risk": "All five test cases are single-threaded, sequential, on `[env:native]` with a FreeRTOS-stub queue (file dependency on freertos/queue.h means the native env must stub it). The latch's actual job is cross-core overwrite-with-readback: producer on Core0 calls `xQueueOverwrite` at WS-event rate (~100Hz), consumer on Core1 calls `xQueueReceive` at 100Hz, and they race. The tests prove the API contract on a sequential machine; they do NOT prove: (a) torn read — does `xQueueOverwrite` copy `sizeof(BodyVelocity) = 12 bytes` atomically? FreeRTOS queue uses a mutex internally, so yes, but the test doesn't assert this. (b) ABA / lost-update — if writer publishes A then B before reader runs, only B is observed (which IS the desired behavior — test_overwrite_returns_latest covers it, but only sequentially). (c) memory-ordering — does the read on Core1 observe a fully-published struct? FreeRTOS queue uses a taskENTER_CRITICAL section internally, which acts as a full barrier on Xtensa, but again unasserted. The test_consumed_value_does_not_reappear_after_overwrite test is the closest to a stress case but is also sequential. **There is no test that runs a producer and consumer in parallel** (would need FreeRTOS on-target). On-target stress test (10s of producer at 1kHz, consumer at 100Hz, assert no torn reads, no out-of-order values) is missing.",
  "suggested_fix": "Add on-target test (`test/test_command_latch_ontarget/`) that creates two FreeRTOS tasks on different cores, producer writes monotonically-increasing values, consumer reads and asserts (a) read value is always one the producer wrote (no tears), (b) read sequence is monotonically non-decreasing in producer-publication-time, (c) under producer-faster-than-consumer, consumer never sees the same value twice.",
  "test_gap": "No multi-threaded, multi-core, or stress test. No 'producer at 500 Hz vs consumer at 100 Hz' coverage."
}
```

```json
{
  "id": "C10-P1-06",
  "title": "Parse failure leaves latch holding previous frame — silent staleness during sustained malformed-frame storm",
  "file": "src/input/WebSocketCommandProducer.cpp",
  "line_range": "143-159",
  "quote": "case WStype_TEXT: {\n            // Drop frames from non-active clients (defense in depth: server\n            // disconnect on rejection isn't always immediate).\n            if (_activeClient.load() != clientNum) return;\n\n            auto result = parseCommandFrame(payload, length);\n            if (!result.ok()) {\n                _parseFailCount.fetch_add(1, std::memory_order_relaxed);\n                switch (result.error) {\n                    case FrameParseError::EmptyOrTooLong: logParseFail(\"len\");   break;\n                    case FrameParseError::InvalidVx:      logParseFail(\"vx\");    break;\n                    case FrameParseError::InvalidVy:      logParseFail(\"vy\");    break;\n                    case FrameParseError::InvalidOmega:   logParseFail(\"omega\"); break;\n                    default: break;\n                }\n                return;\n            }\n\n            _latch.write(result.value);",
  "risk": "On parse failure the producer returns without publishing — the latch retains the last good frame. Combined with the consumer-side rule that `last_fresh_ms` is only updated when `g_latch->read().has_value()` returns true, the consumer sees no new frames during a malformed-frame storm and the staleness ramp engages **eventually** (after 200ms of no parse-successes). This is actually the desired behavior. The risk is the inverse: a buggy operator client that sends 1000Hz of malformed frames will (a) burn ws_prod CPU on parse attempts, (b) flood Serial via logParseFail (rate-limited to 1Hz, OK), (c) NEVER cause the safe state because the staleness engages anyway. But: an attacker who interleaves one valid frame every 199ms with 198ms of malformed-frame spam can hold the robot at the valid-frame setpoint indefinitely while looking like normal traffic. There is no per-second rate limit on valid frames vs invalid frames, and no 'too many parse failures -> disconnect this client' policy.",
  "suggested_fix": "If parse failure rate exceeds N/sec (e.g. 50), disconnect the active client. Or: track ratio of valid:invalid frames and disconnect if < 0.5.",
  "test_gap": "No test feeds malformed frames at high rate. No test asserts that sustained parse failure causes staleness ramp."
}
```

---

## P2 — quality / hardening

```json
{
  "id": "C10-P2-01",
  "title": "now = millis() captured once but used both for fresh-time-stamp and age — fine, but document the convention",
  "file": "src/main_robot.cpp",
  "line_range": "162-175",
  "quote": "auto fresh = g_latch->read();\n        const uint32_t now = millis();\n        if (fresh.has_value()) {\n            last_known      = *fresh;\n            last_fresh_ms   = now;\n            have_seen_fresh = true;\n        }\n\n        // 2) Drive command via staleness alone. The producer's connected()\n        //    flag can lie on half-open TCP; age < 200ms is the actual safety\n        //    net so we don't gate on connected() anymore.\n        BodyVelocity drive_cmd{0.0f, 0.0f, 0.0f};\n        if (have_seen_fresh) {\n            const uint32_t age = now - last_fresh_ms;",
  "risk": "Staleness uses the **consumer-side receive timestamp** (`millis()` when controlTask drained the latch), not the **producer-side publish timestamp** (when ws_prod called latch.write). The two differ by the cross-core queue latency + control-tick alignment, which is bounded by 10ms. Failure modes: (a) consumer-side: if the consumer is jammed (TWDT will fire at 1s, but between 0 and 1s the consumer might not run for 100ms while still passing TWDT — unclear, but possible), then `now - last_fresh_ms` is small (the LAST time the consumer ran, it set last_fresh_ms to that consumer-time), and `now` is the same consumer-time delta later — wait, that's correct, both use consumer time. The actual hole: **if the producer publishes a frame, then ws_prod task hangs (server->loop bug) for 100ms while the control task keeps running at 100Hz, every tick the consumer reads no value, the previous frame is consumed and not republished, `last_known` is stuck at its old value, and the ramp engages**. That is actually safe. (b) the other failure mode: clock-skew between the two cores. ESP32 millis() is the same monotonic FreeRTOS tick on both cores, so no skew. OK. (c) the only real subtlety: `millis()` wraps every ~49 days. `now - last_fresh_ms` uses unsigned subtraction, which is wrap-safe in C as long as both are uint32_t. They are. Verified.",
  "suggested_fix": "Add a one-line comment on main_robot.cpp:164 stating 'last_fresh_ms is consumer receive time, not producer publish time — bounded latency means this is sufficient'. Also unit-test the millis() wraparound case (constants set to UINT32_MAX - 5 in a native test).",
  "test_gap": "No wrap-around test. No producer-vs-consumer-timestamp regression."
}
```

```json
{
  "id": "C10-P2-02",
  "title": "OmniDrivetrain::drive is called every tick even when command unchanged — kinematics + saturation recomputed needlessly",
  "file": "src/drivetrain/controller/PassthroughDrivetrainController.h",
  "line_range": "17-19",
  "quote": "void update(const BodyVelocity& command, const IMUReading& /*imuData*/) override {\n        _drivetrain.drive(command);\n    }",
  "risk": "Context §9.15 flagged this. `PassthroughDrivetrainController::update` always calls `_drivetrain.drive(command)`, which recomputes `OmniKinematics::toWheelRPMs` (saturation scan, three trig-baked constants). At 100Hz this is ~100 redundant kinematics computations/sec when the operator's stick is at rest. Not a correctness issue, and the cost is small (<10µs per call), but **the staleness ramp produces a continuously-changing `drive_cmd` even when `last_known` is stable** — so the cache wouldn't help during ramp anyway. The only redundancy is when `drive_cmd == prev_drive_cmd` exactly, which is rare under fresh-frame mode (ramp differs) and exact during quiescent-zero. Minor.",
  "suggested_fix": "Skip the kinematics call if `drive_cmd == prev_drive_cmd` (exact bit-level comparison on 12 bytes). Or accept the cost and document why.",
  "test_gap": "n/a (performance, not correctness)."
}
```

```json
{
  "id": "C10-P2-03",
  "title": "Producer backpressure on jammed consumer is correct (drop-newest via overwrite) but not asserted by test",
  "file": "src/util/sync/CommandLatch.h",
  "line_range": "25-27",
  "quote": "void write(const T& value) {\n        xQueueOverwrite(_queue, &value);\n    }",
  "risk": "If the control task hangs (won't, TWDT fires) or is otherwise slower than the producer, `xQueueOverwrite` replaces the queued value rather than blocking — the producer never stalls. This is the right behavior for teleop (newest matters), but it's not unit-tested as a property. The `test_overwrite_returns_latest` test (test_command_latch.cpp:44) covers the semantic but not the 'producer-faster-than-consumer-never-blocks' invariant explicitly. Also: arduinoWebSockets `_server->loop()` blocks while parsing/dispatching event callbacks. If the latch write somehow blocked, the WS task would stall. It doesn't block — `xQueueOverwrite` never blocks regardless of queue state — so this is safe but worth documenting.",
  "suggested_fix": "Add a comment in CommandLatch.h::write stating 'xQueueOverwrite never blocks; producer-side backpressure is drop-newest by overwrite'. Add a stress test that writes 10000 times faster than the consumer reads and asserts read returns the most recent.",
  "test_gap": "No explicit producer-faster-than-consumer test."
}
```

```json
{
  "id": "C10-P2-04",
  "title": "BodyVelocity has no NaN/Inf guard at the seam — non-finite floats traverse the entire pipeline",
  "file": "src/drivetrain/BodyVelocity.h",
  "line_range": "1-7",
  "quote": "#pragma once\n\nstruct BodyVelocity {\n    float vx = 0;     // forward speed, m/s (positive = forward)\n    float vy = 0;     // strafe speed, m/s (positive = left)\n    float omega = 0;  // rotation rate, rad/s (positive = CCW from above)\n};",
  "risk": "`std::strtof` parses 'nan', 'inf', '-inf' as valid IEEE values. `CommandFrameParser` accepts them. The latch transports them. The staleness ramp multiplies them by `scale` (still NaN/Inf). `OmniKinematics::toWheelRPMs` propagates them to all three wheels. PID amplifies them. `L298NDriver::setOutput` casts the result to uint8_t — undefined behavior. This is referenced separately in Cell 1 (C1-P0-01) but the seam perspective is: **the trust boundary should be the parser, and the parser does not enforce finiteness**. This is a P0-level issue but Cell 1 already calls it out, so I downgrade to P2 here with cross-reference.",
  "suggested_fix": "In `parseCommandFrame`, after the three strtof calls, add `if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(omega)) return {{}, FrameParseError::InvalidVx};` (or a new InvalidNonFinite error). See also C1-P0-01.",
  "test_gap": "No test in test_command_frame_parser.cpp parses 'nan,0,0' or 'inf,0,0'."
}
```

---

## Seam answers (concise)

- **Q1 end-to-end latency**: ~16ms typical worst case, ~30ms under I2C contention. See C10-P1-04.
- **Q2 CommandLatch correctness**: `xQueueOverwrite`/`xQueueReceive` use FreeRTOS critical sections; act as full barriers on Xtensa. Producer never reads, consumer never writes — verified in `WebSocketCommandProducer.cpp:161` (write only) and `main_robot.cpp:162` (read only). Memory ordering is sound.
- **Q3 staleness**: Receive timestamp, not publish. See C10-P2-01. Bounded by 10ms (control-tick alignment).
- **Q4 staleness ramp**: Scales omega identically to vx/vy (C10-P1-01). Ramp is computed from `last_known` not from previous `drive_cmd` — new arrival cancels in-progress ramp instantly (C10-P1-02).
- **Q5 ordering**: Queue copies `sizeof(BodyVelocity)=12` bytes atomically inside a critical section. No torn read possible. Not asserted by test (C10-P1-05).
- **Q6 dropped frames**: Desired (drop-newest). No sequence number / frame ID, so drops are silent (C10-P1-03).
- **Q7 disconnect propagation**: Producer does NOT publish stop on disconnect. Consumer relies on staleness alone. Time to motors-off: ~6s heartbeat detection (worst case) + 200ms ramp = ~6.2s on half-open TCP; ~200ms on clean close (because operator stopped sending). See C10-P0-02.
- **Q8 reconnect**: New client's first command immediately overwrites `last_known` with no transition smoothing. Lurch risk if previous client left non-zero (C10-P0-01).
- **Q9 parse failure**: Producer publishes nothing; latch holds previous frame; staleness engages eventually. Cited at WebSocketCommandProducer.cpp:158 (`return;` without write). See C10-P1-06.
- **Q10 frame ID monotonicity**: None. No replay protection, no drop detection. C10-P1-03.
- **Q11 backpressure**: Drop-newest via `xQueueOverwrite`. Never blocks. C10-P2-03.
- **Q12 concurrency tests**: `test_command_latch.cpp` has 5 cases — `test_empty_read_returns_nullopt`, `test_write_then_read_returns_value`, `test_second_read_returns_nullopt`, `test_overwrite_returns_latest`, `test_consumed_value_does_not_reappear_after_overwrite`. All single-threaded, sequential, on `[env:native]` with stubbed FreeRTOS. No multi-core race coverage. C10-P1-05.
