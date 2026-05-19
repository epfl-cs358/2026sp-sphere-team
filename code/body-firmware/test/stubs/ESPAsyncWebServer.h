#pragma once

// Native-test stub for ESPAsyncWebServer. Models only the surface
// BalanceTelemetryWs touches: an AsyncWebSocket attached to an
// AsyncWebServer via addHandler(), plus the per-connection text()
// and broadcast textAll() entry points. The pump path on native never
// runs (no FreeRTOS task) so the WS calls are reachable only when a
// test invokes the pumpOnce() hook — in which case we just record the
// last frame string for assertion.

#include <Arduino.h>
#include <cstddef>
#include <cstdint>

class AsyncWebSocketClient {
public:
    void text(const String& s) { last = s; }
    String last;
};

enum AwsEventType { WS_EVT_CONNECT, WS_EVT_DISCONNECT, WS_EVT_DATA, WS_EVT_ERROR, WS_EVT_PONG };

class AsyncWebSocket {
public:
    explicit AsyncWebSocket(const char* path) : _path(path ? path : "") {}

    void textAll(const String& s) {
        lastBroadcast = s;
        broadcastCount++;
    }

    AsyncWebSocketClient* client(uint32_t /*id*/) { return &fakeClient; }

    // Real API: onEvent(handler). Stored but never invoked by tests by default.
    template <typename F>
    void onEvent(F /*handler*/) {}

    const char* url() const { return _path; }

    // Test introspection.
    String lastBroadcast;
    uint32_t broadcastCount = 0;
    AsyncWebSocketClient fakeClient;

private:
    const char* _path;
};

class AsyncWebServerRequest {};

class AsyncWebServer {
public:
    explicit AsyncWebServer(uint16_t port = 0) : _port(port) {}

    // Real API takes an `AsyncWebHandler*`. We type-erase as void* — the test
    // doesn't dispatch requests, it just needs the symbol to exist.
    void addHandler(void* /*handler*/) {}

private:
    uint16_t _port;
};
