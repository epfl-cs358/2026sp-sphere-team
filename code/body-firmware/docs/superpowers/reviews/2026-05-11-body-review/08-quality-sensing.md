# Cell 8 — Code Quality & Architecture (Sensing & Input)

Scope: `code/body-firmware/src/imu/**`, `code/body-firmware/src/input/**` (excluding `util/sync`, covered by Cell 10).

```json
{
  "cell": 8,
  "lens": "code-quality-architecture",
  "scope": "imu/, input/",
  "anchor_commit": "f8ca95f",
  "findings": [
    {
      "id": "C8-P2-01",
      "severity": "P2",
      "title": "CommandProducer interface is thin to the point of being unbalanced — Mock/WS contract differs",
      "file": "src/input/CommandProducer.h:11-17 ; src/input/MockCommandProducer.h:14-31 ; src/input/WebSocketCommandProducer.cpp:117-179",
      "quote": "virtual bool connected() const = 0;",
      "rationale": "The only method on `CommandProducer` is `connected()`. The actual command-delivery side-effect (writing into `CommandLatch<T>`) is not part of the interface; it happens inside the implementation. As a result `MockCommandProducer<T>::inject(cmd)` is the test-only escape hatch for what the real producer does asynchronously from a FreeRTOS task on Core 0. This works (the latch is the real seam), but the abstraction is a leaky one: `connected()` is the only observable shared behavior. A consumer (e.g. the control loop) deliberately ignores `connected()` (main_robot.cpp:170-171 per CONTEXT). So the interface in its current shape carries zero semantic load on the hot path; it exists only so tests have a base class. Consider either widening the contract (e.g. `frameCount()`, `lastError()`, `parseFailCount()` which the WS producer already tracks atomically) or removing the abstraction entirely and depending on `CommandLatch` directly — the current shape is the worst of both worlds.",
      "recommendation": "Pick one: (a) drop `CommandProducer.h` and have callers depend on the latch — the WS producer is already a self-contained sink; or (b) widen the interface to expose the counters that the WS impl already maintains (`frameCount`, `parseFailCount`, `lastFrameMs`) so a healthcheck or LED-blink consumer can be transport-agnostic."
    },
    {
      "id": "C8-P2-02",
      "severity": "P2",
      "title": "Mock/real divergence: WS-only behaviors not modeled by Mock — test-fidelity gap",
      "file": "src/input/MockCommandProducer.h:17-31 ; src/input/WebSocketCommandProducer.cpp:42-115",
      "quote": "void inject(const T& cmd) { if (_started) _latch.write(cmd); }",
      "rationale": "The WS producer has at least four behaviors the Mock does not simulate: (1) connection rejection of a second client via `compare_exchange_strong` on `_activeClient` (WSCP.cpp:124-131); (2) heartbeat-driven half-open detection (`enableHeartbeat(2000,1000,2)` at WSCP.cpp:51); (3) parse-failure path (`_parseFailCount.fetch_add` at WSCP.cpp:150); (4) frame arrival jitter and dropped frames under load. The Mock unconditionally writes the latch on `inject()` regardless of `_connected`. Combined with the fact that `connected()` is the only thing the interface exposes, a test that depends on connection-state-vs-latch-state interaction cannot be written against the abstraction. This is acceptable for a single-transport project, but it should be noted: there is no fakes-level test of the half-open path.",
      "recommendation": "If a future second producer is on the roadmap (e.g. SBUS, gamepad), widen the Mock to optionally drop injects when `!_connected`, simulate disconnect events, and surface a `simulateParseFail()` helper. Otherwise document the limitation."
    },
    {
      "id": "C8-P2-03",
      "severity": "P2",
      "title": "IMU<T> templating leaks BNO055-shaped reading into otherwise-generic consumers",
      "file": "src/imu/IMU.h:8-16 ; src/imu/BNO055IMU.h:15-21 ; src/drivetrain/controller/PassthroughDrivetrainController.h:13",
      "quote": "class BNO055IMU : public IMU<IMUReading> {",
      "rationale": "`IMU<T>` is template-generic, which is fine; the problem is that `IMUReading` is named generically but is a direct snapshot of the BNO055 capability set: it includes a BNO055-specific calibration POD (`CalStatus` with sys/gyro/accel/mag 0–3 quanta — a BNO055 register layout) and a `IMUField` enum whose bits enumerate BNO055 vectors (LinearAccel, Gravity). Any other IMU (MPU-6050, LSM6DSV, BMI270) would have a different calibration concept (often none at all, or accel-only bias). The interface itself is not leaky (it's `T`), but the *type chosen for T* in the only deployed implementation closes the abstraction. The downstream consumer signature `DrivetrainController<BodyVelocity, IMUReading, BodyVelocity>` cements `IMUReading` (BNO055-shaped) into the controller's template arguments.",
      "recommendation": "If a non-BNO055 IMU is ever realistic, split `IMUReading` into a sensor-agnostic `Orientation { Quat q; Vec3 gyro; }` plus a separate `BNO055Status { CalStatus cal; }`, and have BNO055IMU return the agnostic one to the controller. Defer until a second IMU is in scope. Quality of the current shape: fine for one-sensor robot; would refactor at second-sensor introduction."
    },
    {
      "id": "C8-P2-04",
      "severity": "P2",
      "title": "IMUReading carries more than current consumers use — wasted I2C and bandwidth",
      "file": "src/imu/IMUReading.h:31-38 ; src/main_robot.cpp:72-74,186-187 ; src/drivetrain/controller/PassthroughDrivetrainController.h:17",
      "quote": "void update(const BodyVelocity& command, const IMUReading& /*imuData*/) override {",
      "rationale": "The only runtime consumer (`PassthroughDrivetrainController::update`) ignores the IMU entirely (parameter named `/*imuData*/`). Yet main_robot.cpp:72-74 configures `Quaternion | Euler | Gyro | Calibration` and the control loop calls `g_imu.read()` every 10 ms (4 I2C reads per tick — see CONTEXT §9 #10). `IMUReading`'s shape (Quat+Euler+Vec3×3+CalStatus = ~64 B per tick) is fine, but it's over-fetched in two senses: too many fields enabled vs the controller's needs (currently: zero), and the struct itself carries fields no one reads. Quaternion and Euler are redundant representations of the same orientation. `gravity` is never read anywhere in source.",
      "recommendation": "Reduce field mask in main_robot.cpp to the actual minimum needed by the current controller (probably none until tilt-stabilizer ships; until then, removing the I2C reads also reduces control-loop latency variance — Cell 4/6 territory). Drop `gravity` from `IMUReading` unless someone is planning to use it; it adds an unused I2C transaction whenever `IMUField::Gravity` is set."
    },
    {
      "id": "C8-P2-05",
      "severity": "P2",
      "title": "IMUField operator& returns bool — asymmetric with operator|, prevents mask composition",
      "file": "src/imu/IMUReading.h:23-29",
      "quote": "inline bool operator&(IMUField a, IMUField b) {",
      "rationale": "Already noted in CONTEXT §9 #4 as an observation, but it lives in this cell's scope. `operator|` returns `IMUField`, `operator&` returns `bool`. That means `(a & b) & c` won't compile (no `operator&(bool, IMUField)`), and you cannot intersect two masks to compute the smaller of two. Currently all call sites use `(_fields & IMUField::X)` to test single bits, so it works. But the asymmetry is a foot-gun for any future code that wants to compute `availableMask & requestedMask`.",
      "recommendation": "Either rename the bool-returning version to `has(IMUField, IMUField)` / `contains()` and add a proper `operator&` returning `IMUField`, or wrap in a small `IMUFieldSet` class. The free-function-on-enum trick is fine but should be symmetric."
    },
    {
      "id": "C8-P2-06",
      "severity": "P2",
      "title": "CommandFrameParser is reusable but has unrelated knobs hardcoded; format constants split across files",
      "file": "src/input/CommandFrameParser.h:32-58 ; src/input/WebSocketCommandProducer.cpp:18-22",
      "quote": "inline constexpr size_t kCommandFrameMaxLen = 64;",
      "rationale": "The parser is well-isolated: header-only, no transport dependencies, returns `FrameParseResult` with discriminated error enum, unit-tested (test/test_command_frame_parser/). Adding a new field is straightforward (copy the strtof+expect-comma block). Positive. But: (1) `kCommandFrameMaxLen` is a parser constant; the WS task's `kTaskStackBytes`, `kStopTimeoutMs`, `kTaskPriority`, `kTaskCore` are in an anonymous namespace in the cpp. There is no central `input/InputConfig.h`. `WS_PORT` is in main_robot.cpp:41 (defaulted to 80 in the constructor). Tunables are scattered across three files. (2) Adding a 4th positional field would mean editing the parser plus updating callers; there is no name-based or self-describing variant — but pragmatically this is fine for a fixed-shape teleop frame.",
      "recommendation": "Consolidate WS+parser tunables into a single `input/InputConfig.h` (port, max-frame-len, task stack/prio/core, stop timeout). Improves grep-ability and makes a binary-frame variant a config change."
    },
    {
      "id": "C8-P2-07",
      "severity": "P2",
      "title": "Plain-text 'vx,vy,omega' frame format — debuggability vs parse cost, document tradeoff",
      "file": "src/input/CommandFrameParser.h:34-59 ; src/input/WebSocketCommandProducer.h:8-23",
      "quote": "float vx = std::strtof(p, &end);",
      "rationale": "Pros: human-readable in `wscat`/browser devtools, trivial to fuzz, forward-compat trailing fields, parse failure is debuggable. Cons: three `strtof` calls per frame (each is a non-trivial libc routine, but at ≤100 Hz on ESP32 this is negligible — order of tens of microseconds); ambiguity around locale (strtof respects `LC_NUMERIC` — on ESP32 newlib this is `C` so no actual issue), and no schema versioning. A 12-byte binary frame (`float[3]`) would be ~5× smaller and zero-parse, but loses every diagnostic benefit. At 100 Hz × 12 B = 1.2 KB/s, link cost is not a constraint. Recommend keeping text; note the choice.",
      "recommendation": "Document the choice in the header comment (already partially done at WSCP.h:5-24). If latency variance from parsing ever becomes an issue, add a binary variant gated on subprotocol negotiation rather than swapping wholesale. P2 observation, no code change required."
    },
    {
      "id": "C8-P2-08",
      "severity": "P2",
      "title": "Header hygiene mostly clean, but BNO055IMU.h pulls Arduino.h and Adafruit_BNO055 into the included world",
      "file": "src/imu/BNO055IMU.h:10-13 ; src/input/WebSocketCommandProducer.h:36",
      "quote": "class WebSocketsServer;  // forward decl from arduinoWebSockets",
      "rationale": "Positive: `WebSocketCommandProducer.h` uses a forward declaration for `WebSocketsServer` and keeps the library include in the .cpp; this is exactly right and explicitly noted in a comment. Negative: `BNO055IMU.h` is header-only and pulls in `Adafruit_BNO055.h`, `Preferences.h`, `Arduino.h`, plus `debug.h`. That means every translation unit transitively including BNO055IMU.h (currently main_robot.cpp, main_imu_test.cpp) pays the include cost and gets the Arduino macros (`min`, `max`, `B0`, etc.) leaking into its namespace. Could be split into a .cpp with the implementation, leaving a thin header that only includes `IMU.h`, `IMUReading.h`, and uses forward decls for `Adafruit_BNO055` and `TwoWire`. No `using namespace` in any header in scope (good).",
      "recommendation": "Split BNO055IMU into .h/.cpp using the same forward-decl pattern as WebSocketCommandProducer. Low priority since only two TUs include it."
    },
    {
      "id": "C8-P2-09",
      "severity": "P2",
      "title": "Naming consistency: `latch`/`producer`/`frame` consistent; `connected` flag is overloaded",
      "file": "src/input/WebSocketCommandProducer.h:64-65 ; src/input/WebSocketCommandProducer.cpp:117-142",
      "quote": "std::atomic<bool> _connected;",
      "rationale": "Cross-file naming is consistent: `producer`/`latch`/`controller`/`drivetrain` are used the same way everywhere (good — see CONTEXT §5). However `_connected` in WSCP conflates two things: 'a WS client has reached CONNECTED state' and 'that client is the active driver'. The accept logic distinguishes them (a rejected second client never sets `_connected`, and its later DISCONNECT does not clear it — WSCP.cpp:137-141). The semantics are right; the name hides them. A second reader looking only at the field name would assume 'true iff some client is connected'.",
      "recommendation": "Rename `_connected` to `_activeClientConnected` (or have `connected()` return `_activeClient.load() != kNoClient`, deriving from existing state and removing `_connected` entirely — would simplify the invariant)."
    },
    {
      "id": "C8-P2-10",
      "severity": "P2",
      "title": "Constants in code vs config: WS port and parser limits not in config/ header",
      "file": "src/main_robot.cpp:41 ; src/input/WebSocketCommandProducer.cpp:18-22 ; src/input/CommandFrameParser.h:32",
      "quote": "constexpr uint16_t WS_PORT                 = 80;",
      "rationale": "`config/pins.h`, `drivetrain/RobotConstants.h` are good patterns. The input subsystem lacks an equivalent. Port lives in main_robot, parser cap in the parser header, FreeRTOS task params in the cpp anonymous namespace. Heartbeat constants (2000/1000/2) are inline literals at WSCP.cpp:51. This is the same finding as C8-P2-06 framed against the project's own convention — there is precedent for an `input/InputConfig.h`.",
      "recommendation": "Either move WS+parser tunables into `src/input/InputConfig.h`, or accept the scatter and document in a header comment. Heartbeat literals at WSCP.cpp:51 should at minimum get named constants in the anonymous namespace."
    },
    {
      "id": "C8-P2-11",
      "severity": "P2",
      "title": "Dead/under-used: BNO055IMU::isCalibrated() has no runtime caller",
      "file": "src/imu/BNO055IMU.h:82-84",
      "quote": "bool isCalibrated() {",
      "rationale": "`isCalibrated()` is a public, non-const method on the concrete class (not on the `IMU<T>` interface). Only callers are the unit tests (`test_bno055_imu.cpp:335-347`). The runtime ('main_robot.cpp', 'main_imu_test.cpp') uses the in-reading `CalStatus` instead. Not strictly dead — it's the unit test API surface — but it exposes BNO055-specific state outside the interface and never gets used in production. Either promote (have the control loop check it for a 'don't trust orientation yet' signal) or demote (make it private and friend the test, or inline its body where the saveCalibration check happens). Also: should be `const` (it currently isn't).",
      "recommendation": "Mark `const`. Decide whether to promote or remove. Same applies to `_savedThisBoot` — auto-save is fine; just be aware it can't be re-armed without a reboot."
    },
    {
      "id": "C8-P2-12",
      "severity": "P2",
      "title": "Parser silently truncates: messages of exactly `kCommandFrameMaxLen` are rejected, between (max-len, ∞) crashes prevented but no clear contract for 'extra fields'",
      "file": "src/input/CommandFrameParser.h:35-56",
      "quote": "if (length == 0 || length >= kCommandFrameMaxLen) {",
      "rationale": "The `>= kCommandFrameMaxLen` guard correctly reserves room for a NUL terminator (since `buf[length] = '\\0'` writes at index 64 if length == 64 — buffer overflow without the `>=`). The comment 'Trailing chars (whitespace, newline, extra fields) intentionally ignored' at line 56 is the documented forward-compat policy, which is good. But: a 'trailing' field that itself contains a comma will be silently dropped without any indication, even when downstream wants it. There's no observability (`extraFieldCount` could go in `FrameParseResult` for diagnostics).",
      "recommendation": "Acceptable as is. If telemetry from client to robot ever piggybacks on this channel (rather than the reverse direction), revisit. Low priority."
    }
  ],
  "summary": {
    "p0": 0,
    "p1": 0,
    "p2": 12,
    "overall": "Sensing & input code is small, well-factored, and unit-tested where it matters (parser, BNO055 mock-level). No P0/P1 in the quality dimension — the architecture is appropriate for a one-IMU, one-transport robot. Findings are all P2 observations clustered around: (a) the `CommandProducer` interface being too thin to carry real polymorphism; (b) `IMUReading`/`IMUField` reflecting BNO055 register layout in a 'generic-looking' shell; (c) tunables and constants scattered across parser, cpp, and main rather than a single `input/InputConfig.h`; (d) Mock/real test-fidelity gap on connection-state behavior. Strengths: WSCP header uses forward decl pattern correctly; parser is properly extracted and tested; naming is consistent; no `using namespace` in any header; FreeRTOS internals are encapsulated behind `void*` to keep headers narrow."
  }
}
```
