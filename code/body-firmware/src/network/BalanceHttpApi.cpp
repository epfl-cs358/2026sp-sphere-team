/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 */

#include "BalanceHttpApi.h"

#include <ESPAsyncWebServer.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "BalanceConfig.h"
#include "BalanceTuner.h"

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

String configJson(const BalanceConfig& c) {
    char buf[768];
    std::snprintf(
        buf, sizeof(buf),
        "{"
        "\"tiltPerVelocity\":%g,"
        "\"maxTiltSetpoint\":%g,"
        "\"pitchKp\":%g,\"pitchKi\":%g,\"pitchKd\":%g,"
        "\"rollKp\":%g,\"rollKi\":%g,\"rollKd\":%g,"
        "\"pitchDeadband\":%g,\"rollDeadband\":%g,"
        "\"maxOutputVelocity\":%g,"
        "\"envelopeEnterSin\":%g,\"envelopeExitSin\":%g,"
        "\"gyroPitchSign\":%g,\"gyroRollSign\":%g"
        "}",
        static_cast<double>(c.tiltPerVelocity),
        static_cast<double>(c.maxTiltSetpoint),
        static_cast<double>(c.pitchKp), static_cast<double>(c.pitchKi), static_cast<double>(c.pitchKd),
        static_cast<double>(c.rollKp),  static_cast<double>(c.rollKi),  static_cast<double>(c.rollKd),
        static_cast<double>(c.pitchDeadband), static_cast<double>(c.rollDeadband),
        static_cast<double>(c.maxOutputVelocity),
        static_cast<double>(c.envelopeEnterSin), static_cast<double>(c.envelopeExitSin),
        static_cast<double>(c.gyroPitchSign),    static_cast<double>(c.gyroRollSign));
    return String(buf);
}

// Minimal handcrafted JSON extractor: returns true on success and writes the
// matched substring into out. Only handles top-level string/number values for
// the two keys we care about ("key", "value") — no nesting, no escapes beyond
// trimming surrounding quotes. ArduinoJson is intentionally avoided to keep
// this surface dep-free.
bool extractField(const String& body, const char* name, String& out) {
    String needle = String("\"") + name + String("\"");
    int k = body.indexOf(needle);
    if (k < 0) return false;
    int colon = body.indexOf(':', k + needle.length());
    if (colon < 0) return false;
    int i = colon + 1;
    while (i < (int)body.length() && (body[i] == ' ' || body[i] == '\t')) ++i;
    if (i >= (int)body.length()) return false;
    int start, end;
    if (body[i] == '"') {
        start = i + 1;
        end   = body.indexOf('"', start);
        if (end < 0) return false;
    } else {
        start = i;
        end   = start;
        while (end < (int)body.length()) {
            char c = body[end];
            if (c == ',' || c == '}' || c == ' ' || c == '\t' || c == '\n' || c == '\r') break;
            ++end;
        }
    }
    out = body.substring(start, end);
    out.trim();
    return out.length() > 0;
}

// Buffered-body helper: accumulates chunks across onBody calls in the
// request's _tempObject pointer (a heap-allocated String). On the final
// chunk (index + len == total), invokes `done` and frees the buffer.
struct BodyBuf {
    String s;
};

void onBodyAccumulate(AsyncWebServerRequest* request,
                      uint8_t* data, size_t len, size_t index, size_t total,
                      std::function<void(AsyncWebServerRequest*, const String&)> done) {
    if (index == 0) {
        auto* buf = new BodyBuf();
        buf->s.reserve(total + 1);
        request->_tempObject = buf;
    }
    auto* buf = static_cast<BodyBuf*>(request->_tempObject);
    if (!buf) return;
    for (size_t i = 0; i < len; ++i) {
        buf->s += static_cast<char>(data[i]);
    }
    if (index + len >= total) {
        String body = buf->s;
        delete buf;
        request->_tempObject = nullptr;
        done(request, body);
    }
}

}  // namespace

namespace BalanceHttpApi {

void registerRoutes(AsyncWebServer& server, BalanceTuner& tuner) {
    server.on("/balance/config", HTTP_GET, [&tuner](AsyncWebServerRequest* req) {
        sendJson(req, 200, configJson(tuner.snapshot()));
    });

    server.on(
        "/balance/set", HTTP_POST,
        [](AsyncWebServerRequest* req) {
            if (req->contentLength() == 0) {
                sendJson(req, 400, String("{\"ok\":false,\"error\":\"empty body\"}"));
            }
        },
        nullptr,
        [&tuner](AsyncWebServerRequest* request, uint8_t* data, size_t len, size_t index, size_t total) {
            onBodyAccumulate(request, data, len, index, total,
                [&tuner](AsyncWebServerRequest* req, const String& body) {
                    String key, valStr;
                    if (!extractField(body, "key", key) || !extractField(body, "value", valStr)) {
                        sendJson(req, 400, String("{\"ok\":false,\"error\":\"missing key or value\"}"));
                        return;
                    }
                    float v = valStr.toFloat();
                    String err;
                    if (tuner.trySet(key, v, err)) {
                        sendJson(req, 200, String("{\"ok\":true}"));
                    } else {
                        String esc;
                        esc.reserve(err.length() + 4);
                        for (size_t i = 0; i < err.length(); ++i) {
                            char c = err[i];
                            if (c == '"' || c == '\\') esc += '\\';
                            esc += c;
                        }
                        sendJson(req, 400,
                                 String("{\"ok\":false,\"error\":\"") + esc + String("\"}"));
                    }
                });
        });

    server.on("/balance/save", HTTP_POST, [&tuner](AsyncWebServerRequest* req) {
        if (tuner.saveNvs()) {
            sendJson(req, 200, String("{\"ok\":true}"));
        } else {
            sendJson(req, 500, String("{\"ok\":false,\"error\":\"save failed\"}"));
        }
    });

    server.on("/balance/reset", HTTP_POST, [&tuner](AsyncWebServerRequest* req) {
        tuner.resetToDefaults();
        sendJson(req, 200, String("{\"ok\":true}"));
    });

    // CORS preflight — match all /balance/* with OPTIONS.
    server.on("/balance/config", HTTP_OPTIONS, [](AsyncWebServerRequest* req) {
        sendJson(req, 204, String());
    });
    server.on("/balance/set", HTTP_OPTIONS, [](AsyncWebServerRequest* req) {
        sendJson(req, 204, String());
    });
    server.on("/balance/save", HTTP_OPTIONS, [](AsyncWebServerRequest* req) {
        sendJson(req, 204, String());
    });
    server.on("/balance/reset", HTTP_OPTIONS, [](AsyncWebServerRequest* req) {
        sendJson(req, 204, String());
    });
}

}  // namespace BalanceHttpApi
