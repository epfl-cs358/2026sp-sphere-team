/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "BalanceTelemetry.h"

class AsyncWebServer;

// Per-tick CSV stream over a single AsyncWebSocket at "/telemetry". The WS
// stream is the authoritative history — the laptop capture daemon writes
// every frame to disk. On-chip state is intentionally minimal: a double-
// buffered "most recent snapshot" for /telemetry/latest, a 100-entry dt
// window for /telemetry/stats, 16 atomic event-bit counters, and the
// Core1→Core0 producer queue + pump task. No ring buffer — keeping seconds
// of history on a memory-tight ESP32 is the laptop's job, not the chip's.
//
// Threading:
//   - publish() is the producer entry; Core 1 (control loop), non-blocking.
//     On full queue the snapshot is dropped and dropCount() ticks.
//   - The pump task runs on Core 0 (priority 1, stack 4096). For each
//     snapshot it formats one CSV line, broadcasts via textAll, swaps
//     the latest-pointer, appends to the dt window, accumulates event
//     counters.
//
// latest()/dtStats() are safe to call from any task; latest() uses a
// lock-free pointer swap (writer alternates between two static slots),
// dtStats() does a relaxed scan of the dt ring — readers may observe a
// torn dt entry under heavy contention but min/mean/max are robust to
// single-sample noise.
namespace BalanceTelemetryWs {

// Attach the WebSocket handler to `server` and spawn the pump task.
// Safe to call more than once.
void init(AsyncWebServer& server);

// Producer entry: queue `snap` for the pump task. Non-blocking.
void publish(const BalanceTelemetry& snap);

// Copy the most-recent published snapshot into `out`. Returns false if no
// snapshot has been pumped yet — clients use this to distinguish "no data"
// from "data is genuinely all-zeros."
bool latest(BalanceTelemetry* out);

// dt window stats (seconds). Returns 0 for all three if no valid dt has
// been pumped yet. Window is the last 100 publishes.
void dtStats(float* minOut, float* meanOut, float* maxOut);

// Count of pumped snapshots whose event_flags had `bit` set. Bits ≥ 16
// return 0.
std::uint32_t eventCount(std::size_t bit);

// Snapshots dropped because the producer queue was full.
std::uint32_t dropCount();

// Total snapshots pumped (monotonic).
std::uint32_t writeIdx();

// The canonical CSV header line emitted to every new WS client. No
// trailing newline.
const char* headerLine();

#ifdef BB8_TEST_HOOKS
// Drain one snapshot from the in-process queue and run the full pump
// path. Native test only — on device the FreeRTOS task does this.
void pumpOnce();

// Wipe all module state. Native test isolation only.
void resetForTesting();
#endif

}  // namespace BalanceTelemetryWs
