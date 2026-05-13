# Cell 5 — Real-time / Concurrency, Sensing & Input

Lens: real-time / concurrency. Scope: `src/imu/**`, `src/input/**` (excluding `src/util/sync/` which belongs to Cell 10).

```json
{
  "cell": 5,
  "lens": "realtime-concurrency",
  "scope": ["src/imu/**", "src/input/**"],
  "findings": [

    {
      "id": "C5-P1-01",
      "severity": "P1",
      "title": "Adafruit_BNO055::getQuat/getVector/getCalibration called inline in control task: 4 sequential I2C round-trips on the 10 ms hot path",
      "file": "src/main_robot.cpp",
      "line": 186,
      "quote": "IMUReading imu_reading = g_imu.read();",
      "secondary": {
        "file": "src/imu/BNO055IMU.h",
        "line": 39,
        "quote": "if (_fields & IMUField::Quaternion) {\n            auto q = _bno.getQuat();"
      },
      "context": "main_robot.cpp configures the IMU with Quaternion | Euler | Gyro | Calibration (4 fields). BNO055IMU::read() calls Adafruit getQuat() (8-byte register read), getVector(EULER) (6 bytes), getVector(GYROSCOPE) (6 bytes), getCalibration() (1 byte) — each is a separate I2C transaction at 100 kHz default. Per-transaction overhead (start, addr, regaddr, restart, read, stop) is roughly 0.5–0.8 ms; 4 reads = 2–3 ms wall-clock, sometimes spiking higher under clock-stretching. This is **synchronous on Core 1 inside vTaskDelayUntil's 10 ms window**. Worst-case it consumes 20–30% of the control budget for data the passthrough controller does not consume.",
      "lens": "realtime-concurrency",
      "impact": "Eats budget. Three consequences: (a) jitter on motor PID update (PID dt is hard-coded 0.010f, but the actual wall-clock work happens later in the slot), (b) any I2C stretch from the BNO055 (the chip is known to stretch up to 1 ms on its slow MCU side) eats into the next tick and can trip TWDT (1 s) only on egregious stalls but routinely chews margin, (c) future stabilizer controller will inherit a hot-path IMU read whose latency is unbounded under I2C contention.",
      "suggestion": "Two options: (1) drop fields the controller doesn't read (Calibration check can run at 1 Hz on a side task, Euler is redundant with Quat). The passthrough controller ignores all of it, so a minimal read of just Quaternion or none would be free. (2) Move IMU sampling to a dedicated task on Core 0 or Core 1 with its own period and publish via a single-slot latch identical to the command latch — control task reads the latest snapshot, never blocks on I2C. Option 1 is cheap and unblocks the immediate hot-path concern."
    },

    {
      "id": "C5-P1-02",
      "severity": "P1",
      "title": "BNO055 calibration save (NVS putBytes) runs inline inside read() on the control task",
      "file": "src/imu/BNO055IMU.h",
      "line": 73,
      "quote": "if (!_savedThisBoot && _bno.isFullyCalibrated()) {\n                saveCalibration();\n                _savedThisBoot = true;\n            }",
      "context": "saveCalibration() calls prefs.begin() / putBytes() / end() (lines 107-115). NVS writes to ESP32 internal flash. A single putBytes of 22 bytes typically takes 10-40 ms; worst case (page erase) can be 100+ ms. This happens once per boot when the IMU reaches full calibration, **on the 100 Hz control task on Core 1**, inside `g_imu.read()` which is called every tick.",
      "lens": "realtime-concurrency",
      "impact": "On the tick this fires, the control task overshoots its 10 ms deadline by 1-4× the period. PID dt remains the hard-coded 0.010f but actual wall-clock between motor updates is ~50 ms — that one cycle the wheels run open-loop at the last command for ~50 ms. If calibration completion happens mid-motion this is a single visible glitch (motor 'kick' or coast). TWDT at 1 s won't trip; the bug is silent.",
      "suggestion": "Don't write NVS from the control task. Either (a) expose `isFullyCalibrated()` as a flag and have the housekeeping `loop()` on Core 1 (or a dedicated low-prio task) do the save, or (b) defer to a one-shot xTaskCreate that self-deletes. Anything off the hot path."
    },

    {
      "id": "C5-P1-03",
      "severity": "P1",
      "title": "ws_prod task polls WebSocketsServer::loop() at ~1 ms via vTaskDelay (not vTaskDelayUntil) — sleep is constant, not period",
      "file": "src/input/WebSocketCommandProducer.cpp",
      "line": 105,
      "quote": "while (_running.load()) {\n        if (_server) _server->loop();\n        esp_task_wdt_reset();\n        vTaskDelay(pdMS_TO_TICKS(kLoopDelayMs));\n    }",
      "context": "links2004/WebSockets `WebSocketsServer::loop()` is the non-blocking pump for the underlying WiFiClient. It does poll/accept/read on the lwIP socket — internally non-blocking but with no event-driven backpressure to user space. With kLoopDelayMs=1 and pdMS_TO_TICKS(1) on the ESP32 (1 ms tick), the actual sleep is 1 tick = ~1 ms. Effective wake rate ~500-1000 Hz. **Sleep is `delay`, not `delay_until`**, so the period drifts: real wake = max(1 tick, work). Under burst (e.g. half-open detect, heartbeat fire), the next wake is pushed.",
      "lens": "realtime-concurrency",
      "impact": "Two issues. (1) Latency: a TEXT frame arriving immediately after server->loop() returns has to wait ~1 ms before being parsed and latched. Combined with the 10 ms control tick this adds 1 ms to teleop end-to-end latency — measurable but small. (2) Heartbeat timing is owned by `_server->loop()` (the library timer-checks heartbeat inside its pump). If a parse or a log printf takes long, the next pump is delayed and the 2 s ping cadence drifts. Not catastrophic at the configured 2000 ms with 1000 ms timeout and 2 retries.",
      "suggestion": "Use a tighter loop or yield only when no work pending (the library doesn't expose a 'work ready' signal, so this is mostly fine). Acceptable to leave. Consider documenting the ~1 ms WS→latch latency in the context doc."
    },

    {
      "id": "C5-P0-04",
      "severity": "P0",
      "title": "WStype_TEXT parse + latch.write runs **on the ws_prod task** (Core 0), not on a separate WS callback thread — but no auth, no rate limit, and parse failures log to Serial which on a flood path can stall the task",
      "file": "src/input/WebSocketCommandProducer.cpp",
      "line": 143,
      "quote": "case WStype_TEXT: {\n            // Drop frames from non-active clients (defense in depth: server\n            // disconnect on rejection isn't always immediate).\n            if (_activeClient.load() != clientNum) return;\n\n            auto result = parseCommandFrame(payload, length);",
      "context": "links2004/WebSockets dispatches onEvent **inline inside `_server->loop()` on the calling task**, so the callback runs on the ws_prod task on Core 0 (no extra hop). Good. However: (a) protocol is plaintext ws:// with no auth (acknowledged in header comment, trust-LAN), (b) a malicious or buggy client can send 1000+ malformed frames per second — `logParseFail` is rate-limited to 1/s, but the parse itself runs to completion every frame and writes Serial.printf in `logParseFail` when the rate-limit window opens. Serial.printf at 115200 baud is synchronous (~0.5 ms per short line until the UART TX FIFO drains). That blocks ws_prod, delaying both heartbeat and further parses.",
      "lens": "realtime-concurrency",
      "impact": "On a flood (intentional or buggy operator), ws_prod's wake rate falls because each Serial.printf burns ~0.5-1 ms. Even rate-limited to 1/s the **periodic counter dump** (`Serial.printf(\"[ws] frames=%u ...`) at line 166 fires every 1000 frames and is unconditional. Worse, the periodic counter dump and the parse-fail printf can interleave with WS heartbeat ack pumping, potentially missing the 2 s heartbeat under sustained flood. Combined with the half-open detect window (~6 s) and the control task's 200 ms staleness window, the failure mode is: control task already zeros motors after 200 ms; ws_prod takes longer to declare disconnect. So motors are safe, but auto-reconnect after a flood may be slow.",
      "suggestion": "P0 because the trust boundary is documented (LAN only) but a single misbehaving client can degrade the entire WS task. Mitigations: (a) Move all Serial logging in onWsEvent behind a debug flag or a non-blocking ring buffer, (b) drop frames > kCommandFrameMaxLen earlier (already done), (c) consider per-client frame rate limit (e.g. reject > 200 Hz). At minimum, document the assumption that the operator stays under 100 Hz."
    },

    {
      "id": "C5-P1-05",
      "severity": "P1",
      "title": "CommandLatch is length-1 with xQueueOverwrite — at producer rate > control rate, all but the last frame between ticks are silently dropped (intended, but undocumented at the WS layer)",
      "file": "src/input/WebSocketCommandProducer.cpp",
      "line": 161,
      "quote": "_latch.write(result.value);",
      "context": "Per 00-CONTEXT.md §3, the latch is `xQueueOverwrite` / non-blocking `xQueueReceive`. If the operator sends at 100 Hz and the control task reads at 100 Hz, the rates can interleave such that 0 or 2 frames are read per tick — average 1, but jitter is real. If the operator bursts (e.g. 500 Hz for a moment because of a UI debounce bug), 4 out of 5 frames per control period are dropped on the floor.",
      "lens": "realtime-concurrency",
      "impact": "By design — newest command wins. The risk is only if the producer believes 'send it and it's queued' semantics. Since the wire protocol is conceptually a level signal (latest body-frame velocity), drop is correct. No code change needed but the operator-side doc should state 'commands are coalesced to ~100 Hz on the robot side; client-side rate above 200 Hz wastes WiFi bandwidth'.",
      "suggestion": "Document in WebSocketCommandProducer.h header that frames are coalesced into a single-slot latch and back-pressure is invisible to the client. No code change."
    },

    {
      "id": "C5-P1-06",
      "severity": "P1",
      "title": "Below the latch: TCP socket buffering is owned by lwIP and the WS library — bounded but not configured. Burst behavior is implicit.",
      "file": "src/input/WebSocketCommandProducer.cpp",
      "line": 106,
      "quote": "if (_server) _server->loop();",
      "context": "There is no per-connection receive ring or backpressure config visible in the producer. arduinoWebSockets uses WiFiClient (lwIP) under the hood; lwIP TCP RX window defaults to ~5840 bytes on ESP32 Arduino. With 16-byte command frames, that's ~365 frames of in-flight data the kernel will buffer before applying TCP backpressure. If ws_prod stalls (e.g. Serial flood, see C5-P0-04), frames pile up in lwIP, then the WS library drains them all in one `server->loop()` call — onWsEvent fires N times sequentially, all parsing onto the same overwriting latch, so only the last sticks. CPU cost on drain is O(N).",
      "lens": "realtime-concurrency",
      "impact": "Bounded — won't OOM. But the drain spike can extend a single ws_prod iteration to dozens of ms, deferring the next heartbeat tick. Combined with the 6 s half-open detect, this is below the safety threshold but the worst-case ws_prod tick is essentially unbounded if a client sends a maximally fragmented WS message tree.",
      "suggestion": "Two cheap mitigations: (a) cap onWsEvent calls per `server->loop()` iteration (link wraps the loop, but you can break after N parses), or (b) raise ws_prod priority to 3 (still below control's 4) so it preempts the WiFi housekeeping task and drains faster. Neither is urgent; flag for later."
    },

    {
      "id": "C5-P2-07",
      "severity": "P2",
      "title": "Heartbeat is driven by `_server->loop()` inline — same task, not a separate timer. A long parse delays the next heartbeat check.",
      "file": "src/input/WebSocketCommandProducer.cpp",
      "line": 51,
      "quote": "_server->enableHeartbeat(2000, 1000, 2);",
      "context": "links2004/WebSockets implements heartbeat by checking elapsed time inside `server->loop()`. There is no FreeRTOS timer or dedicated task — it's the same single-threaded pump that runs onWsEvent. If ws_prod is descheduled or busy in onWsEvent (parse + Serial.printf), the heartbeat ping is sent late. The (2000, 1000, 2) config means ping every 2 s, fail if no pong in 1 s, retry 2 times — total detect window ~6 s. Late pings just shift this window; they don't cause spurious disconnects unless ws_prod stalls > 6 s, which TWDT (1 s) would already catch.",
      "lens": "realtime-concurrency",
      "impact": "Low. TWDT (1 s panic) bounds the worst case before heartbeat even becomes the concern. Spurious disconnect requires ws_prod to be 'mostly' but not 'totally' wedged.",
      "suggestion": "Leave as-is. The TWDT subscription (line 103) is the right backstop."
    },

    {
      "id": "C5-P2-08",
      "severity": "P2",
      "title": "BNO055 begin() is blocking, and it runs **before** `connectWifi()` and **before** `g_producer->start()` in setup() — operator cannot connect until IMU is up",
      "file": "src/main_robot.cpp",
      "line": 221,
      "quote": "if (!g_imu.begin()) {\n        Serial.println(\"FATAL: BNO055 init failed — restarting.\");\n        ESP.restart();\n    }",
      "secondary": {
        "file": "src/imu/BNO055IMU.h",
        "line": 26,
        "quote": "if (!_bno.begin(OPERATION_MODE_NDOF)) return false;"
      },
      "context": "Adafruit_BNO055::begin() internally waits for chip ID + reset settle, typically 650-1000 ms (the library uses 650 ms `delay()` after sending reset). `restoreCalibration()` adds `2 * MODE_SETTLE_MS = 40 ms` if offsets are present in NVS. Total IMU boot: ~700 ms-1 s. WiFi connect (`connectWifi()`) then takes 1-10 s. So the WS server (g_producer->start()) is **not listening for ~2-11 s after power-on**, and on IMU failure, the device reboots without ever starting WS.",
      "lens": "realtime-concurrency",
      "impact": "Operator UX: dashboard cannot WS-connect during the boot window. This is acceptable for a 2-11 s window. The risk is if BNO055 has an intermittent I2C startup failure, the device reboot-loops silently with no remote diagnostics (no WS, no OTA — actually wait, OTA also runs after this because `initOta()` is called after the IMU check). A bricked IMU = unflashable robot until physical USB serial access.",
      "suggestion": "Two options for resilience: (1) make IMU init non-fatal (log+continue) so OTA still comes up — would let you flash a fix remotely. (2) Move WiFi+OTA bring-up before IMU init. Either fixes the brick-on-bad-IMU scenario. Pick (2): start WS+OTA first so a bad IMU is recoverable over the air."
    },

    {
      "id": "C5-P1-09",
      "severity": "P1",
      "title": "WiFi-offline reboot watchdog uses a single `g_lastWifiConnectedMs` updated in two places (WiFi event callback + loop()) — race plus reentrancy hole at the 30 s boundary",
      "file": "src/main_robot.cpp",
      "line": 257,
      "quote": "if (WiFi.status() == WL_CONNECTED) {\n        g_lastWifiConnectedMs = millis();\n    } else if (millis() - g_lastWifiConnectedMs > WIFI_OFFLINE_REBOOT_MS) {\n        Serial.println(\"FATAL: WiFi offline >30s — restarting.\");\n        ESP.restart();\n    }",
      "context": "`g_lastWifiConnectedMs` is declared `volatile uint32_t` (line 83) and written from two contexts: (a) `onWifiEvent` callback (line 88) on the WiFi system-event task, and (b) `loop()` on Core 1's loopTask. The `volatile` ensures visibility but not atomicity for the read-then-subtract on line 259. On ESP32 a uint32_t access is naturally atomic, so torn reads aren't realistic, but the check is racy in a different way: if WiFi recovers at t=29.9 s, the WiFi event task may fire ARDUINO_EVENT_WIFI_STA_GOT_IP and update the timestamp just after `loop()` evaluated `WiFi.status() != WL_CONNECTED` (which is true if disconnect just happened and the status flag hasn't flipped yet). Sequence: loopTask wakes at t=29.95s, reads WiFi.status() = DISCONNECTED, subtracts (29950 - g_lastWifiConnectedMs = 29950 ms), boundary not yet crossed, OK. Next wake t=30.05s, status now CONNECTED (just reconnected), updates g_lastWifiConnectedMs = 30050. Safe in this ordering. But if reconnect happens at t=30.001s **after** the loop has already read status as DISCONNECTED and computed the elapsed (which is now 30001 ms > 30000), the reboot fires even though the radio is back. Window is ~100 ms (one loop period).",
      "lens": "realtime-concurrency",
      "impact": "Rare edge: reconnect happens within the same 100 ms window that loop() decides to reboot. Reboot is recoverable (it just restarts), so the impact is short interruption rather than data loss. Not data-corrupting.",
      "suggestion": "Re-check `WiFi.status()` immediately before `ESP.restart()` to close the window:\n```\nelse if (millis() - g_lastWifiConnectedMs > WIFI_OFFLINE_REBOOT_MS &&\n         WiFi.status() != WL_CONNECTED) {\n    ESP.restart();\n}\n```\nCheap fix; eliminates the spurious reboot."
    },

    {
      "id": "C5-P2-10",
      "severity": "P2",
      "title": "ws_prod task priority (2) is below loopTask (1)? Actually equal — but lower than control (4); the WiFi event task pumps callbacks at priority 23 from a different core",
      "file": "src/input/WebSocketCommandProducer.cpp",
      "line": 19,
      "quote": "constexpr UBaseType_t kTaskPriority = 2;",
      "context": "Per Arduino-ESP32, the WiFi system-event task runs at priority 19 (esp_event_loop). The lwIP TCP task runs at priority 18. ws_prod at priority 2 is far below them, which is correct for back-pressure (the network stack will buffer; ws_prod drains when scheduled). The Arduino loopTask is priority 1. **However**, the OTA start callback `g_producer->stop()` runs on loopTask (priority 1) and blocks up to 1 s on a semaphore that's signalled by ws_prod (priority 2). Since ws_prod is higher priority and on Core 0 while loopTask is on Core 1, no priority inversion. Cross-core, the semaphore is signaled and the blocking task wakes. OK.",
      "lens": "realtime-concurrency",
      "impact": "No bug. Note for future: if anyone moves the OTA callback to Core 0 (e.g. WS-triggered firmware update), the priority relationship between loopTask and ws_prod becomes a single-core scheduling concern.",
      "suggestion": "Add a one-line comment near `kTaskPriority = 2` noting the relationship to loopTask/OTA expectations. No code change."
    },

    {
      "id": "C5-P2-11",
      "severity": "P2",
      "title": "BNO055IMU `IMUField operator&` returns `bool` — composing more than two masks with `&` won't work",
      "file": "src/imu/IMUReading.h",
      "line": 27,
      "quote": "inline bool operator&(IMUField a, IMUField b) {\n    return (static_cast<uint8_t>(a) & static_cast<uint8_t>(b)) != 0;\n}",
      "context": "Acknowledged in 00-CONTEXT.md §9 item 4. Inside read() the usage is binary tests `if (_fields & IMUField::Quaternion)`, so this is fine for the current code. The realtime concern: if a future caller writes `if ((_fields & (IMUField::Quaternion | IMUField::Gyro)) == IMUField::Quaternion)` to selectively re-read, the chained `&` returns bool, which compares against IMUField as int — gives the wrong answer silently.",
      "lens": "realtime-concurrency",
      "impact": "Latent. Not a realtime bug today; will become a correctness bug the first time someone tries multi-bit masking and reads stale IMU data.",
      "suggestion": "Make `operator&` return `IMUField` to match `operator|`. Add a separate `inline bool any(IMUField a, IMUField b)` for the bool test, or just write `static_cast<uint8_t>(_fields & X) != 0`."
    }

  ],

  "lens_summary": {
    "core_0_path": "WS event → ws_prod task (prio 2, Core 0) inline-dispatches onWsEvent (no extra thread). Parse + latch write are tiny (~10 us). Serial logging in fail/counter paths is the main hot-path hazard (C5-P0-04).",
    "core_1_path": "Control task (prio 4, Core 1) reads latch (non-blocking), then synchronously calls g_imu.read() which performs 4 sequential I2C transactions (~2-3 ms typical), then controller+drivetrain. The 4 IMU transactions plus PID work fit in 10 ms in normal cases but consume 20-30% of the budget for data the passthrough controller throws away. NVS save inside read() can blow the deadline once per boot (C5-P1-02).",
    "heartbeat_and_staleness": "Two independent safety nets — WS heartbeat (~6 s detect, library-side, owned by ws_prod) and control-side staleness ramp (200 ms, owned by control task). Control task does NOT consult connected() flag, only latch age. Robust.",
    "boot_window": "IMU init (700 ms-1 s blocking) + WiFi connect (1-10 s blocking) = 2-11 s before WS port 80 listens. Acceptable, but a bricked IMU means no OTA-recoverable firmware (C5-P2-08)."
  }
}
```
