/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 */

#pragma once

#include <Arduino.h>

class AsyncWebServer;

// HTTP polling API for the balancing telemetry stream. Sits in front of the
// 512-deep ring buffer + per-event counters owned by BalanceTelemetryWs so
// any client (curl, Claude, the webapp dashboard) can read the same data the
// WS stream emits — without subscribing to the live CSV feed.
//
//   GET /telemetry/latest   -> single most-recent snapshot as JSON
//   GET /telemetry/recent   -> ?n=N (default 100, cap 512) last snapshots
//   GET /telemetry/stats    -> seq, uptime, dt_ms{min,mean,max,jitter},
//                              events_since_boot, ws_drops, ring_overruns
//   GET /telemetry/schema   -> 83-column metadata + event-bit decode
//   GET /telemetry/header   -> canonical CSV header (text/plain)
//   OPTIONS /telemetry/*    -> CORS preflight (origin *)
//
// All responses include permissive CORS headers (origin *) so the webapp on
// any port — and any browser — can consume them. JSON is hand-rolled
// (snprintf / String +=); ArduinoJson is intentionally avoided to keep the
// surface dep-free, matching BalanceHttpApi.
namespace BalanceTelemetryHttpApi {

// MUST match Agent K's call site exactly — Wave 3 dependency.
void registerRoutes(AsyncWebServer& server);

#ifdef BB8_TEST_HOOKS
// Test hooks: exercise the JSON/text builders without dispatching through
// AsyncWebServer (the native stub doesn't route requests).
String buildLatestJson();
String buildRecentJson(std::size_t n);
String buildStatsJson();
String buildSchemaJson();
String buildHeader();
#endif

}  // namespace BalanceTelemetryHttpApi
