/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 */

#pragma once

class AsyncWebServer;

namespace ArmingHttpApi {

// Registers JSON REST routes for arming control from the tuner GUI:
//   GET  /arm/state   -> {"state":"disarmed"|"armed"|"killed"}
//   POST /arm         -> {"ok":true,"state":...}
//   POST /disarm      -> {"ok":true,"state":...}
//   POST /kill        -> {"ok":true,"state":...}
//   POST /clearkill   -> {"ok":true,"state":...}
//   OPTIONS /arm/*    -> CORS preflight
// All responses include permissive CORS headers (origin *).
void registerRoutes(AsyncWebServer& server);

}  // namespace ArmingHttpApi
