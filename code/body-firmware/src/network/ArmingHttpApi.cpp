/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 */

#include "ArmingHttpApi.h"

#include <ESPAsyncWebServer.h>

#include "ArmingState.h"

namespace {

const char* const kCorsOrigin  = "*";
const char* const kCorsMethods = "GET, POST, OPTIONS";
const char* const kCorsHeaders = "Content-Type";

void addCors(AsyncWebServerResponse* r) {
    r->addHeader("Access-Control-Allow-Origin",  kCorsOrigin);
    r->addHeader("Access-Control-Allow-Methods", kCorsMethods);
    r->addHeader("Access-Control-Allow-Headers", kCorsHeaders);
}

void sendJson(AsyncWebServerRequest* req, int code, const String& body) {
    auto* r = req->beginResponse(code, "application/json", body);
    addCors(r);
    req->send(r);
}

const char* stateLabel(ArmingState::State s) {
    switch (s) {
        case ArmingState::State::Disarmed: return "disarmed";
        case ArmingState::State::Armed:    return "armed";
        case ArmingState::State::Killed:   return "killed";
    }
    return "?";
}

String stateJson(bool ok = true) {
    String s;
    s.reserve(48);
    s += "{\"ok\":";
    s += ok ? "true" : "false";
    s += ",\"state\":\"";
    s += stateLabel(ArmingState::get());
    s += "\"}";
    return s;
}

}  // namespace

namespace ArmingHttpApi {

void registerRoutes(AsyncWebServer& server) {
    server.on("/arm/state", HTTP_GET, [](AsyncWebServerRequest* req) {
        sendJson(req, 200, stateJson());
    });
    server.on("/arm", HTTP_POST, [](AsyncWebServerRequest* req) {
        ArmingState::arm();
        sendJson(req, 200, stateJson());
    });
    server.on("/disarm", HTTP_POST, [](AsyncWebServerRequest* req) {
        ArmingState::disarm();
        sendJson(req, 200, stateJson());
    });
    server.on("/kill", HTTP_POST, [](AsyncWebServerRequest* req) {
        ArmingState::kill();
        sendJson(req, 200, stateJson());
    });
    server.on("/clearkill", HTTP_POST, [](AsyncWebServerRequest* req) {
        ArmingState::clearKill();
        sendJson(req, 200, stateJson());
    });

    server.on("/arm/state", HTTP_OPTIONS, [](AsyncWebServerRequest* req) {
        sendJson(req, 204, String());
    });
    server.on("/arm", HTTP_OPTIONS, [](AsyncWebServerRequest* req) {
        sendJson(req, 204, String());
    });
    server.on("/disarm", HTTP_OPTIONS, [](AsyncWebServerRequest* req) {
        sendJson(req, 204, String());
    });
    server.on("/kill", HTTP_OPTIONS, [](AsyncWebServerRequest* req) {
        sendJson(req, 204, String());
    });
    server.on("/clearkill", HTTP_OPTIONS, [](AsyncWebServerRequest* req) {
        sendJson(req, 204, String());
    });
}

}  // namespace ArmingHttpApi
