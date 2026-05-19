/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 */

#pragma once

class AsyncWebServer;
class BalanceTuner;

namespace BalanceHttpApi {

// Registers JSON REST routes for the BalanceTuner GUI:
//   GET  /balance/config   -> live config as JSON
//   POST /balance/set      -> {key,value} -> {ok|error}
//   POST /balance/save     -> persists live to NVS
//   POST /balance/reset    -> reverts live to compiled defaults
//   OPTIONS /balance/*     -> CORS preflight
// All responses include permissive CORS headers (origin *).
void registerRoutes(AsyncWebServer& server, BalanceTuner& tuner);

}  // namespace BalanceHttpApi
