# Cell 6 — Real-time / Concurrency: Glue & Lifecycle

**Scope:** `src/util/*.h` (excl. `util/sync/`), `src/config/*.h`, `src/main_*.cpp`.
**Lens:** core pinning, loop period stability, task priorities, TWDT, stack budget,
inter-core memory ordering, reboot/failsafe.
**Anchor commit:** f8ca95f.

## Summary

The control task is correctly pinned to Core 1 with `vTaskDelayUntil`, TWDT
subscribed, and prio 4 above Arduino loopTask (1) and ws_prod (2). Three
real-time hazards stand out at P0/P1: (1) the control task hard-codes
`dt = 10 ms` to the PID even though `vTaskDelayUntil` only guarantees average
period (any single skipped tick produces a stale `dt` that under-integrates I
and over-amplifies D); (2) every 100 iterations the loop emits a blocking
`Serial.printf` from inside the hot path; (3) `Serial.print` from `controlTask`,
`onWifiEvent` (Core 1 sys-event task), and `controlTask`'s HWM dump are not
synchronized with Core 0's `ws_prod` and `onWsEvent` Serial writes — the
Arduino `HardwareSerial` is thread-safe for short writes via an internal mutex
but the mutex itself can stall the control task for milliseconds when another
core is mid-`printf`. Reboot path uses `ESP.restart()` which does *not*
explicitly stop motors first; the H-bridges latch the last PWM until the CPU
reset clears the GPIO.

```json
[
  {
    "id": "C6-P0-01",
    "severity": "P0",
    "title": "Control loop assumes fixed dt=10ms regardless of actual elapsed time",
    "file": "src/main_robot.cpp",
    "lines": "189-191",
    "quote": "// 4) Tick motor controllers (RPM PID).\n        const float dt = static_cast<float>(CONTROL_PERIOD_MS) / 1000.0f;\n        g_drivetrain.update(dt);",
    "finding": "`dt` is a compile-time constant fed to the PID's integrator and derivative term, but `vTaskDelayUntil` only guarantees the *average* period — when a higher-priority task or an I2C stall stretches the iteration, the next tick fires immediately and the PID sees the wrong `dt`. Worse: if the IMU read blocks for 30 ms (BNO055 over-the-bus stalls have been observed in the wild), `vTaskDelayUntil` will fire the next 3 iterations back-to-back with no delay, all reporting `dt = 0.010` — the integrator advances 3× too slowly and the derivative term sees three near-zero `dt`s. Either (a) compute actual elapsed `dt` from `xTaskGetTickCount()` between iterations and pass that, or (b) use `vTaskDelay(period)` so the dt invariant holds (but admit drift). Mixing the two is unsafe.",
    "lens": "real-time / period jitter accounting"
  },
  {
    "id": "C6-P0-02",
    "severity": "P0",
    "title": "Blocking Serial.printf inside the 100 Hz control loop",
    "file": "src/main_robot.cpp",
    "lines": "195-202",
    "quote": "if (hwm_iter < 100) {\n            UBaseType_t hwm = uxTaskGetStackHighWaterMark(NULL);\n            if (hwm < stack_hwm_min) stack_hwm_min = hwm;\n            if (++hwm_iter == 100) {\n                Serial.printf(\"[ctrl] stack HWM min over 100 iters: %u words free\\n\",\n                              stack_hwm_min);\n            }\n        }",
    "finding": "At 115200 baud each UART byte is ~87 µs. The 50-ish-char HWM line is ~4 ms — 40% of the 10 ms budget. This fires once at iteration 100 (~1 s after boot) and is bounded, but is positioned between `esp_task_wdt_reset()` and `vTaskDelayUntil`, so it eats into the next-tick slack. The HardwareSerial ring buffer is 256 B by default; if the buffer was already partially full (e.g. from `onWifiEvent` on Core 1 sys-task), the print blocks until drained. Move the HWM dump to a low-priority task or stop early using a one-shot bool that pre-formats the line and yields via `Serial.write` only after `vTaskDelayUntil`.",
    "lens": "logging in hot path"
  },
  {
    "id": "C6-P0-03",
    "severity": "P0",
    "title": "ESP.restart() reboot paths do not stop motors first",
    "file": "src/main_robot.cpp",
    "lines": "131-138, 221-224, 257-262",
    "quote": "} else if (millis() - g_lastWifiConnectedMs > WIFI_OFFLINE_REBOOT_MS) {\n        Serial.println(\"FATAL: WiFi offline >30s — restarting.\");\n        ESP.restart();\n    }",
    "finding": "All three failsafe reboots (`ESP.restart()`) jump straight into the bootloader. The L298N driver pins are still being driven by the last `analogWrite` value, and `ESP.restart()` does not guarantee a GPIO reset before the boot ROM re-initialises pins (the boot ROM puts most pins into input/hi-Z, but motor pins are typically forwarded through the H-bridge with capacitive coupling that can hold the FET gate above threshold for tens of ms). At minimum call `g_drivetrain.stop()` immediately before `ESP.restart()` so the L298N `brake` short stalls the motor (both pins HIGH ≡ short across the bridge, fastest passive decel). This matters because the 30 s WiFi-offline path is the most likely runtime trigger — the operator has just lost the link and the robot is moving.",
    "lens": "reboot path safety"
  },
  {
    "id": "C6-P1-04",
    "severity": "P1",
    "title": "lwIP / WiFi event task overlaps ws_prod on Core 0 — priority inversion risk",
    "file": "src/main_robot.cpp + src/input/WebSocketCommandProducer.cpp",
    "lines": "main_robot.cpp:50, WebSocketCommandProducer.cpp:19-20",
    "quote": "constexpr BaseType_t  CONTROL_TASK_CORE     = 1;",
    "finding": "Core assignment is correct (control=1, ws=0) but ws_prod is prio 2 on Core 0 where the ESP-IDF `wifi` task runs at prio 23 (configurable) and `tcpip_thread` at prio 18. ws_prod is *below* both, which is fine for throughput but means `xQueueOverwrite` on the latch can be preempted mid-call by WiFi. The latch is FreeRTOS-queue-backed (cross-core safe per Cell 10), so the preemption is safe — but `_server->loop()` itself takes an internal mutex inside arduinoWebSockets that the `tcpip_thread` also touches. If `tcpip_thread` holds that mutex and then yields to its higher-prio peer, ws_prod inherits the lock wait. The TWDT is 1 s, so this is bounded, but worst-case latch staleness can spike to ~hundreds of ms which is well past the 200 ms staleness ramp. Recommend adding a frame-rate counter sanity log (already present at line 165) to detect this in production.",
    "lens": "priority inversion"
  },
  {
    "id": "C6-P1-05",
    "severity": "P1",
    "title": "Arduino loopTask (Core 1, prio 1) is not subscribed to TWDT and runs OTA blocking I/O",
    "file": "src/main_robot.cpp",
    "lines": "252-264",
    "quote": "void loop() {\n    // Housekeeping: OTA poll + WiFi reboot watchdog. Control runs in dedicated\n    // tasks. Tick at 10 Hz so OTA initiate requests are answered promptly.\n    ArduinoOTA.handle();",
    "finding": "Per context §2, Arduino loopTask is the only task on Core 1 *besides* control, and runs at prio 1 (lower than control's 4). loopTask is not subscribed to TWDT (`esp_task_wdt_add(NULL)` is only called from `controlTask` and `ws_prod` taskBody). `ArduinoOTA.handle()` performs UDP receive + flash erase; an OTA upload can block here for ~10 s. Because OTA runs at lower prio than control, control still ticks — good. But if the WiFi watchdog at line 259 *also* depends on `loop()` running, a stuck `ArduinoOTA.handle()` will silently bypass the 30 s reboot. Either subscribe loopTask to TWDT (with a TWDT budget that allows ~5 s OTA chunks) or move the WiFi-offline reboot watchdog into `controlTask` where TWDT already exists.",
    "lens": "watchdog coverage"
  },
  {
    "id": "C6-P1-06",
    "severity": "P1",
    "title": "Control task subscribes to TWDT but never unsubscribes; task is infinite anyway, so init must be idempotent — verify `esp_task_wdt_init` runs before subscribe",
    "file": "src/main_robot.cpp",
    "lines": "228-247",
    "quote": "esp_task_wdt_init(TWDT_TIMEOUT_S, true);\n\n    xTaskCreatePinnedToCore(\n        &controlTask,\n        \"control\",\n        CONTROL_TASK_STACK_BYTES,\n        nullptr,\n        CONTROL_TASK_PRIORITY,\n        nullptr,\n        CONTROL_TASK_CORE);",
    "finding": "Ordering is correct: `esp_task_wdt_init` at line 238 runs before `xTaskCreatePinnedToCore` at line 240, so when `controlTask` later calls `esp_task_wdt_add(NULL)` (line 148) the TWDT is configured. However ws_prod's `start()` at line 232 (which calls `esp_task_wdt_add(NULL)` inside its task body, line 103 of producer) runs *before* `esp_task_wdt_init` at line 238 — the producer task may start, hit `esp_task_wdt_add` on a default-configured TWDT (Arduino-core default is 5 s, panic disabled on some IDF versions), then the production code re-inits to (1 s, panic=true). On ESP-IDF 4.x, `esp_task_wdt_init` after subscribe will reset the TWDT but keep subscriptions; on 5.x semantics differ. Safer: call `esp_task_wdt_init` *first* in setup, before `g_producer->start()`.",
    "lens": "TWDT ordering / setup ordering"
  },
  {
    "id": "C6-P1-07",
    "severity": "P1",
    "title": "g_lastWifiConnectedMs is volatile-only — torn read between Core 0 WiFi event and Core 1 loopTask",
    "file": "src/main_robot.cpp",
    "lines": "83, 87-88, 257-260",
    "quote": "volatile uint32_t g_lastWifiConnectedMs = 0;",
    "finding": "Written from `onWifiEvent` (which runs on the Arduino sys-event task — pinned to Core 0 by default on ESP-IDF 4.x), read from `loop()` on Core 1. `volatile` on a `uint32_t` does *not* guarantee atomicity across cores on ESP32 — Xtensa LX6 32-bit aligned loads are atomic on a single core, but cache coherency between cores requires explicit synchronization (or in practice the ESP32's L1 caches are write-through so the read will see the write, but the C++ memory model doesn't promise this). At 1 Hz update rate the worst case is reading the value mid-write, which would yield either the old or new timestamp ± a few µs — not safety-critical, but should be `std::atomic<uint32_t>` for correctness and to silence TSAN-like static analysis. Verify that the WiFi event runs on Core 0 (it does in ESP-IDF 4.x); if it ever migrates, the bug becomes a single-core read-modify-write race.",
    "lens": "inter-core memory ordering"
  },
  {
    "id": "C6-P1-08",
    "severity": "P1",
    "title": "Control task period derived from `pdMS_TO_TICKS(10)` quantizes to FreeRTOS tick — verify 1 ms tick rate",
    "file": "src/main_robot.cpp",
    "lines": "154-155, 204",
    "quote": "TickType_t lastWake = xTaskGetTickCount();\n    const TickType_t period = pdMS_TO_TICKS(CONTROL_PERIOD_MS);",
    "finding": "FreeRTOS on ESP32 Arduino defaults to `configTICK_RATE_HZ=1000` (1 ms tick). `pdMS_TO_TICKS(10)` = 10 ticks, exact. If the SDK is ever rebuilt with `configTICK_RATE_HZ=100`, this becomes `pdMS_TO_TICKS(10)` = 1 tick = 10 ms with no jitter floor, *but* every other `vTaskDelay` in the system rounds up — `vTaskDelay(pdMS_TO_TICKS(1))` in ws_prod (line 21) becomes ~10 ms instead of 1 ms, collapsing the producer's WS event loop to 100 Hz. The constant `LOOP_TICK_MS = 100` in `loop()` would still produce 10 Hz. No bug today, but the assumption is undocumented. Add a `static_assert(configTICK_RATE_HZ >= 1000, ...)` in `main_robot.cpp` or `RobotConstants.h`.",
    "lens": "loop period stability"
  },
  {
    "id": "C6-P1-09",
    "severity": "P1",
    "title": "Stack sizes 8192 B not validated against actual call depth — IMU + PID + drivetrain on one frame",
    "file": "src/main_robot.cpp",
    "lines": "48, 144-204",
    "quote": "constexpr uint32_t CONTROL_TASK_STACK_BYTES = 8192;",
    "finding": "Per-iteration call graph: `controlTask` → `g_imu.read()` (Adafruit_BNO055 internal buffers ~32 B I2C transactions × 4 fields) → `g_controller->update(...)` (passthrough, no allocation) → `g_drivetrain.update(dt)` → 3× `motor->update()` (FIT0186Motor stack frame includes a `MovingAverage<5>::push` which is trivial) + 3× `PID::compute()` (small floats only, no recursion). No deep recursion, no large stack arrays. `Serial.printf` (line 199) uses ~256 B vsnprintf buffer internally — fits. The one-shot HWM dump at line 199-200 will print the actual minimum free word count from the first 100 iterations — *use it*. If reported value is < 1024 words (~4 KB free), shrink call depth; if > 4096 words free, reduce stack to 4096 B. As written the budget is generous but unverified post-OTA changes.",
    "lens": "stack sizes"
  },
  {
    "id": "C6-P1-10",
    "severity": "P1",
    "title": "Setup() heap-allocates g_latch/g_producer/g_controller after IMU init but before TWDT init — control task could start before producer",
    "file": "src/main_robot.cpp",
    "lines": "228-247",
    "quote": "g_latch      = new CommandLatch<BodyVelocity>();\n    g_producer   = new WebSocketCommandProducer(*g_latch, WS_PORT);\n    g_controller = new PassthroughDrivetrainController(g_drivetrain, g_imu);\n\n    g_producer->start();\n\n    initOta();",
    "finding": "Ordering is technically safe: control task is created *last* (line 240), after `g_latch`, `g_producer`, and `g_controller` are non-null. But the inverse is not enforced — if a future refactor moves `xTaskCreatePinnedToCore` earlier, the control task body dereferences `g_latch->read()` (line 162), `g_controller->update(...)` (line 187), `g_drivetrain.update(...)` (line 191) without null checks. Add `BB8_ASSERT(g_latch && g_controller, ...)` at the top of `controlTask` or convert globals to `std::optional` with check-once-at-start. (Note: `BB8_ASSERT` is no-op in non-native envs, so this is documentation, not enforcement.)",
    "lens": "setup() ordering"
  },
  {
    "id": "C6-P2-11",
    "severity": "P2",
    "title": "vTaskDelayUntil with `lastWake = xTaskGetTickCount()` initialized before subscribing to TWDT — first iter's deadline may already be in the past",
    "file": "src/main_robot.cpp",
    "lines": "148-155",
    "quote": "esp_task_wdt_add(NULL);\n\n    BodyVelocity last_known{};\n    uint32_t     last_fresh_ms = 0;\n    bool         have_seen_fresh = false;\n\n    TickType_t lastWake = xTaskGetTickCount();",
    "finding": "Minor: if `esp_task_wdt_add(NULL)` ever blocks (it shouldn't, but in pathological cases the TWDT spinlock can hold for tens of µs), `lastWake` is captured *after* the delay, so the first call to `vTaskDelayUntil(&lastWake, 10)` may compute a wake time already past, returning immediately and giving back-to-back ticks 1 and 2. PID will see this as a 0 ms gap. Capture `lastWake` *before* `esp_task_wdt_add`, or accept the one-tick anomaly at boot.",
    "lens": "period stability at boot"
  },
  {
    "id": "C6-P2-12",
    "severity": "P2",
    "title": "Serial.print/printf cross-task contention — ws_prod, controlTask, onWifiEvent, onWsEvent all write Serial",
    "file": "src/main_robot.cpp + src/input/WebSocketCommandProducer.cpp",
    "lines": "main_robot.cpp:89-93, 108, 115, 119, 129-133, 199, 222, 249, 260; producer 127-141, 166-168, 173, 186",
    "quote": "Serial.printf(\"[ctrl] stack HWM min over 100 iters: %u words free\\n\", stack_hwm_min);",
    "finding": "HardwareSerial on ESP32 Arduino-core is internally mutex-protected (a `portMUX_TYPE` taken inside `write()`), so the prints are correct but contended. The mutex is a spinlock on dual-core ESP32 — the holder is never preempted, but the waiter spins for the duration of the holder's write. A 50-char print at 115200 baud takes ~4 ms; if `onWsEvent`'s parse-fail log fires while `controlTask` is mid-printf, control loses its slot. Aggregate Serial bandwidth at 115200 baud is ~11.5 kB/s; current logging is well under that, but the contention shows up as iteration-period jitter not throughput. Mitigation: bump baud to 921600 in `Serial.begin` (line 211) — ESP32 USB-UART chip on Wemos D1 R32 supports it.",
    "lens": "logging contention / period jitter"
  },
  {
    "id": "C6-P2-13",
    "severity": "P2",
    "title": "loop() at 10 Hz is the only thing handling OTA — high-prio control task can starve it during sustained 100 Hz iterations if IMU stalls",
    "file": "src/main_robot.cpp",
    "lines": "252-264",
    "quote": "vTaskDelay(pdMS_TO_TICKS(LOOP_TICK_MS));",
    "finding": "Arduino loopTask is prio 1, control is prio 4, both on Core 1. FreeRTOS preemptive scheduler with same-core differing prios: control fully preempts loop. If `imu.read()` consistently takes >5 ms (BNO055 over I2C can stall), the control task uses ~50% of Core 1, leaving 5 ms every 10 ms for loop. `ArduinoOTA.handle()` needs ~2-10 ms uncontended bursts to accept a new connection's TCP backlog. In normal operation this works; under load (IMU calibration drift, OmniDrivetrain saturated) OTA could miss its 5 s timeout and reject the flash. Not safety-critical (operator can power-cycle) but operationally annoying.",
    "lens": "task priority scheduling"
  },
  {
    "id": "C6-P2-14",
    "severity": "P2",
    "title": "MovingAverage<N>::reset() not used; per-tick `average()` recomputes sum O(N)",
    "file": "src/util/MovingAverage.h",
    "lines": "22-29",
    "quote": "float average() const {\n        if (_count == 0) return 0.0f;\n        float sum = 0.0f;\n        for (size_t i = 0; i < _count; ++i) {\n            sum += _buffer[i];\n        }\n        return sum / static_cast<float>(_count);\n    }",
    "finding": "`average()` is O(N) per call; with N=5 and 3 motors, that's 15 fp-adds per 10 ms — negligible. Not a real-time concern. Comment for the record: a streaming sum (add new, subtract evicted) is trivially O(1) and is the standard form, but cosmetic. Skip.",
    "lens": "stack-local arithmetic in hot path"
  }
]
```

## Cross-references

- `C6-P0-01` (fixed dt) interacts with Cell 7's PID review — if Cell 7 already
  notes that PID expects an accurate `dt`, this is the call site that violates it.
- `C6-P1-04` (priority inversion on Core 0) depends on Cell 5's WS lifecycle
  review for the `_server->loop()` internal locking.
- `C6-P0-03` (motors not stopped before reboot) needs Cell 9's brake-semantics
  review — L298N "brake" is both-pins-HIGH which is the safest pre-reset state.
- `C6-P1-05` (loopTask not on TWDT) shares scope with Cell 11's OTA review.
- The CommandLatch cross-core handoff itself is Cell 10's territory; this cell
  only consumes it.

## Verified safe

- `vTaskDelayUntil` is used (line 204), not `vTaskDelay` — phase-locked period.
  Lens question 2 ANSWERED: correct.
- Core pinning: control=Core 1 (line 50), ws_prod=Core 0 (producer line 20),
  Arduino loopTask=Core 1 (framework default). No overlap. Lens question 1
  ANSWERED: correct.
- TWDT subscribed by both real tasks (`controlTask` line 148, `ws_prod` line
  103) and reset within each loop (line 193, producer line 107). Lens question
  6 ANSWERED: correct.
- Both tasks at 8192 B stack — no deep recursion, no large stack-local arrays
  found in scope. HWM dump (lines 195-202) will validate at runtime. Lens
  question 4 ANSWERED: likely sufficient, but unverified post-OTA.
- `g_drivetrain`, `g_motor*`, `g_pid*`, `g_imu` are only touched by control
  task after `setup()` finishes — no cross-core access beyond CommandLatch.
  Lens question 9 ANSWERED: clean (apart from `g_lastWifiConnectedMs` per
  C6-P1-07).
