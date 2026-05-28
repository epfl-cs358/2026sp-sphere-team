/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 */

#pragma once

#include <Arduino.h>

class AsyncWebServer;

// HTTP polling API for the balancing telemetry stream. Stream-only design:
// the chip publishes via WS; history lives on the laptop capture daemon, not
// on the ESP32. These endpoints expose just the latest snapshot + running
// stats + schema, so any client (curl, Claude, the webapp dashboard) can
// poll without subscribing to the live CSV feed.
//
//   GET /telemetry/latest   -> single most-recent snapshot as JSON
//   GET /telemetry/stats    -> seq, uptime, dt_ms{min,mean,max,jitter},
//                              events_since_boot, ws_drops
//   GET /telemetry/schema   -> 83-column metadata + event-bit decode
//   GET /telemetry/header   -> canonical CSV header (text/plain)
//   OPTIONS /telemetry/*    -> CORS preflight (origin *)
//
// For history beyond "latest", use `tune_pid.py slice` against the laptop's
// captured CSV — keeping a multi-second ring on-chip would re-introduce the
// DRAM-overflow class of bug the stream-only design was built to avoid.
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
String buildStatsJson();
String buildSchemaJson();
String buildHeader();
#endif

}  // namespace BalanceTelemetryHttpApi
