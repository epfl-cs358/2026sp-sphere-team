/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini, with assistance from Claude (Anthropic)
 */

#include "WebSocketCommandProducer.h"

#include <Arduino.h>                    // Serial
#include <WebSocketsServer.h>           // arduinoWebSockets (Links2004)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include <cstdlib>                      // strtof

namespace {
constexpr uint32_t kTaskStackBytes = 8192;
constexpr UBaseType_t kTaskPriority = 2;
constexpr BaseType_t kTaskCore = 0;
constexpr TickType_t kLoopDelayMs = 2;
constexpr TickType_t kStopTimeoutMs = 1000;
}

WebSocketCommandProducer::WebSocketCommandProducer(CommandLatch<BodyVelocity>& latch, uint16_t port)
    : _latch(latch),
      _port(port),
      _server(nullptr),
      _taskHandle(nullptr),
      _exitSemaphore(nullptr),
      _running(false),
      _connected(false) {}

WebSocketCommandProducer::~WebSocketCommandProducer() {
    // Defensive: if user forgot to call stop(), do it now.
    if (_running.load()) stop();
}

void WebSocketCommandProducer::start() {
    if (_running.load()) return;

    _server = new WebSocketsServer(_port);
    _server->begin();
    _server->onEvent([this](uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
        this->onWsEvent(num, static_cast<uint8_t>(type), payload, length);
    });

    _exitSemaphore = xSemaphoreCreateBinary();
    _running = true;

    TaskHandle_t handle = nullptr;
    xTaskCreatePinnedToCore(
        &WebSocketCommandProducer::taskTrampoline,
        "ws_prod",
        kTaskStackBytes,
        this,
        kTaskPriority,
        &handle,
        kTaskCore);
    _taskHandle = handle;
}

void WebSocketCommandProducer::stop() {
    if (!_running.load()) return;
    _running = false;

    // Wait for task to signal exit (with timeout fallback).
    if (_exitSemaphore) {
        if (xSemaphoreTake(static_cast<SemaphoreHandle_t>(_exitSemaphore),
                           pdMS_TO_TICKS(kStopTimeoutMs)) != pdTRUE) {
            // Task didn't exit gracefully; force delete.
            if (_taskHandle) vTaskDelete(static_cast<TaskHandle_t>(_taskHandle));
        }
        vSemaphoreDelete(static_cast<SemaphoreHandle_t>(_exitSemaphore));
        _exitSemaphore = nullptr;
    }
    _taskHandle = nullptr;

    if (_server) {
        delete _server;
        _server = nullptr;
    }
    _connected = false;
}

bool WebSocketCommandProducer::connected() const {
    return _connected.load();
}

void WebSocketCommandProducer::taskTrampoline(void* arg) {
    static_cast<WebSocketCommandProducer*>(arg)->taskBody();
}

void WebSocketCommandProducer::taskBody() {
    while (_running.load()) {
        if (_server) _server->loop();
        vTaskDelay(pdMS_TO_TICKS(kLoopDelayMs));
    }
    if (_exitSemaphore) {
        xSemaphoreGive(static_cast<SemaphoreHandle_t>(_exitSemaphore));
    }
    vTaskDelete(NULL);
}

void WebSocketCommandProducer::onWsEvent(uint8_t /*clientNum*/, uint8_t type,
                                         const uint8_t* payload, size_t length) {
    auto wsType = static_cast<WStype_t>(type);
    switch (wsType) {
        case WStype_CONNECTED:
            _connected = true;
            Serial.println("[ws] client connected");
            break;
        case WStype_DISCONNECTED:
            _connected = false;
            Serial.println("[ws] client disconnected");
            break;
        case WStype_TEXT: {
            // Payload is NOT null-terminated; copy with explicit length.
            // Expected format: "vx,vy,omega" (newline optional/trailing).
            constexpr size_t kMaxFrameLen = 64;
            if (length == 0 || length >= kMaxFrameLen) {
                logParseFail("len");
                return;
            }

            char buf[kMaxFrameLen];
            for (size_t i = 0; i < length; ++i) buf[i] = static_cast<char>(payload[i]);
            buf[length] = '\0';

            char* p = buf;
            char* end = nullptr;

            float vx = strtof(p, &end);
            if (end == p || *end != ',') { logParseFail("vx"); return; }
            p = end + 1;

            float vy = strtof(p, &end);
            if (end == p || *end != ',') { logParseFail("vy"); return; }
            p = end + 1;

            float omega = strtof(p, &end);
            if (end == p) { logParseFail("omega"); return; }
            // Any trailing chars (whitespace, newline) ignored.

            _latch.write({vx, vy, omega});
            break;
        }
        default:
            // ERROR, BIN, FRAGMENT_*, PING, PONG — ignored.
            break;
    }
}

void WebSocketCommandProducer::logParseFail(const char* field) {
    // Rate-limit to one message per second to avoid flooding Serial.
    const uint32_t now = millis();
    if (now - _lastParseFailLogMs < 1000) return;
    _lastParseFailLogMs = now;
    Serial.printf("[ws] frame parse failed at %s\n", field);
}
