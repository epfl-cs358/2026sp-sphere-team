/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "BalanceTelemetry.h"

class AsyncWebServer;

// Per-tick CSV stream over a single AsyncWebSocket at "/telemetry", backed by
// a Core1→Core0 FreeRTOS queue, a Core-0 pump task, a 512-deep ring of recent
// snapshots, and 16 atomic per-event-bit counters. The HTTP polling API
// (Wave 3) reads the ring + counters; the WS clients consume the CSV stream.
//
// Threading:
//   - publish() is the producer entry point; called from Core 1 (control loop)
//     and must be non-blocking. On full queue the snapshot is dropped and
//     dropCount() ticks.
//   - The pump task runs on Core 0 (priority 1, stack 4096). It blocks on the
//     queue, formats one CSV line per snapshot, broadcasts to all WS clients,
//     stores the snapshot in the ring, and accumulates event-bit counters.
//
// Idempotent: init() may be called more than once; only the first call wires
// the handler and spawns the task.
namespace BalanceTelemetryWs {

// Capacity of the recent-snapshot ring. ~5 s at 100 Hz. Public so callers
// (e.g. the HTTP API) can size their buffers without dipping into private
// state.
constexpr std::size_t kRingSize = 512;

// Attach the WebSocket handler to `server` and spawn the pump task.
// Safe to call before or after the server begins accepting connections.
void init(AsyncWebServer& server);

// Producer entry: queue `snap` for the pump task. Non-blocking; on full
// queue the snapshot is dropped and dropCount() advances.
void publish(const BalanceTelemetry& snap);

// Copy the most recent up to `maxN` snapshots in chronological order into
// `out`. Returns the count written (≤ min(maxN, kRingSize, writeIdx())).
// Safe to call from any task; uses the seq column for torn-read detection.
std::size_t snapshotRecent(BalanceTelemetry* out, std::size_t maxN);

// Number of times event bit `bit` (0..15) has been set in any published
// snapshot since boot. Bits ≥ 16 return 0.
std::uint32_t eventCount(std::size_t bit);

// Number of snapshots dropped because the producer queue was full.
std::uint32_t dropCount();

// Total snapshots ever processed by the pump (post-queue, monotonic).
// Wraps to ring slots via `% kRingSize` internally.
std::uint32_t writeIdx();

// The fixed CSV header line (no trailing newline). Same string sent to each
// WS client on connect. Exposed for the HTTP API's /telemetry/header route
// and for tests.
const char* headerLine();

#ifdef BB8_TEST_HOOKS
// Test-only entry points. Pump task is not spawned on native — instead,
// publish() pushes onto an internal in-process queue and pumpOnce() drains
// one snapshot synchronously, running the same format → ring → counters
// path the FreeRTOS task would run on device.
void pumpOnce();

// Wipe all internal state (queue, ring, counters, dropCount, writeIdx).
// Test isolation only.
void resetForTesting();
#endif

}  // namespace BalanceTelemetryWs
