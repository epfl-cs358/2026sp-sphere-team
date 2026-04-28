/*
 * BB-8 Head Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini, with assistance from Claude (Anthropic)
 */

#pragma once

#include <ESPAsyncWebServer.h>
#include "StreamServer.h"
#include "StreamConfig.h"
#include "StatusJson.h"
#include "FrameHeader.h"

class AsyncStreamServer : public StreamServer {
public:
    AsyncStreamServer(StreamConfig& config, int (*getRSSI)(), unsigned long (*getUptime)(),
                       uint16_t port = 80)
        : _config(config), _getRSSI(getRSSI), _getUptime(getUptime),
          _server(port), _ws("/stream") {}

    void begin(uint16_t = 80) override {
        _ws.onEvent([this](AsyncWebSocket* server, AsyncWebSocketClient* client,
                           AwsEventType type, void* arg, uint8_t* data, size_t len) {
            onWsEvent(server, client, type, arg, data, len);
        });
        _server.addHandler(&_ws);

        _server.on("/status", HTTP_GET, [this](AsyncWebServerRequest* req) {
            handleStatus(req);
        });

        _server.on("/config", HTTP_POST, [this](AsyncWebServerRequest* req) {},
            nullptr,
            [this](AsyncWebServerRequest* req, uint8_t* data, size_t len,
                   size_t index, size_t total) {
                handleConfigBody(req, data, len, index, total);
            });

        _server.on("/config", HTTP_OPTIONS, [](AsyncWebServerRequest* req) {
            AsyncWebServerResponse* resp = req->beginResponse(204);
            resp->addHeader("Access-Control-Allow-Origin", "*");
            resp->addHeader("Access-Control-Allow-Methods", "POST, OPTIONS");
            resp->addHeader("Access-Control-Allow-Headers", "Content-Type");
            req->send(resp);
        });

        _server.begin();
        Serial.println("Server started");
    }

    uint8_t connectedClients() override {
        return _ws.count();
    }

    // Called from loop() — only sends if a client requested a frame
    void broadcastFrame(const uint8_t* data, size_t len) override {
        if (!_frameRequested || _ws.count() == 0) return;

        _frameRequested = false;
        _ws.cleanupClients();

        size_t totalLen = FrameHeader::SIZE + len;
        uint8_t* buf = (uint8_t*)malloc(totalLen);
        if (!buf) return;

        FrameHeader::encode(buf, _frameSeq++, millis());
        memcpy(buf + FrameHeader::SIZE, data, len);

        _ws.binaryAll(buf, totalLen);
        free(buf);
    }

    bool frameRequested() const { return _frameRequested; }

    void onConfigChange(ConfigCallback cb) override {
        _configCallback = cb;
    }

private:
    StreamConfig& _config;
    int (*_getRSSI)();
    unsigned long (*_getUptime)();
    AsyncWebServer _server;
    AsyncWebSocket _ws;
    ConfigCallback _configCallback;
    uint32_t _frameSeq = 0;
    volatile bool _frameRequested = false;

    void handleStatus(AsyncWebServerRequest* req) {
        char json[256];
        buildStatusJson(json, sizeof(json), _config,
                       _getRSSI(), _getUptime(), _ws.count());

        AsyncWebServerResponse* resp = req->beginResponse(200, "application/json", json);
        resp->addHeader("Access-Control-Allow-Origin", "*");
        req->send(resp);
    }

    void handleConfigBody(AsyncWebServerRequest* req, uint8_t* data, size_t len,
                          size_t index, size_t total) {
        if (index + len == total) {
            StreamConfig newConfig = _config;
            if (parseConfigJson(reinterpret_cast<const char*>(data), total, newConfig)) {
                _config = newConfig;
                if (_configCallback) _configCallback(_config);
                Serial.printf("Config updated: %s %ufps q%u\n",
                    resolutionToString(_config.resolution), _config.fps, _config.quality);

                char json[256];
                buildStatusJson(json, sizeof(json), _config,
                               _getRSSI(), _getUptime(), _ws.count());

                AsyncWebServerResponse* resp = req->beginResponse(200, "application/json", json);
                resp->addHeader("Access-Control-Allow-Origin", "*");
                req->send(resp);
            } else {
                req->send(400, "application/json", R"({"error":"invalid config"})");
            }
        }
    }

    void onWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client,
                   AwsEventType type, void*, uint8_t* data, size_t len) {
        switch (type) {
            case WS_EVT_CONNECT:
                Serial.printf("WS client %u connected\n", client->id());
                for (auto& c : server->getClients()) {
                    if (c.id() != client->id()) {
                        c.close();
                    }
                }
                break;
            case WS_EVT_DISCONNECT:
                Serial.printf("WS client %u disconnected\n", client->id());
                _frameRequested = false;
                break;
            case WS_EVT_DATA:
                // Client sends "next" to request the next frame
                if (len >= 4 && memcmp(data, "next", 4) == 0) {
                    _frameRequested = true;
                }
                break;
            default:
                break;
        }
    }
};
