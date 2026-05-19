/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 */

#include "BalanceTelemetryHttpApi.h"

#include <ESPAsyncWebServer.h>

#include <cstddef>

#include "BalanceTelemetry.h"
#include "BalanceTelemetryWs.h"

// RED-phase stub: returns empty strings so the test binary links but the
// assertions fail. Real implementation lands in the GREEN commit.

namespace BalanceTelemetryHttpApi {

void registerRoutes(AsyncWebServer& /*server*/) {}

#ifdef BB8_TEST_HOOKS
String buildLatestJson() { return String(); }
String buildRecentJson(std::size_t /*n*/) { return String(); }
String buildStatsJson() { return String(); }
String buildSchemaJson() { return String(); }
String buildHeader() { return String(); }
#endif

}  // namespace BalanceTelemetryHttpApi
