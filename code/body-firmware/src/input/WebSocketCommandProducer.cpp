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
#include <esp_task_wdt.h>

#include "ArmingState.h"
#include "CommandFrameParser.h"

namespace {
constexpr uint32_t kTaskStackBytes = 8192;
constexpr UBaseType_t kTaskPriority = 2;
constexpr BaseType_t kTaskCore = 0;
constexpr TickType_t kLoopDelayMs = 1;
constexpr TickType_t kStopTimeoutMs = 1000;
}

WebSocketCommandProducer::WebSocketCommandProducer(CommandLatch<BodyVelocity>& latch, uint16_t port)
    : _latch(latch),
      _port(port),
      _server(nullptr),
      _taskHandle(nullptr),
      _exitSemaphore(nullptr),
      _running(false),
      _connected(false),
      _activeClient(kNoClient),
      _frameCount(0),
      _parseFailCount(0) {}

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
    // Detect half-open TCP within ~7.5s: ping every 3s, fail after 2.5s + 2 retries.
    // pingInterval > pongTimeout is required by arduinoWebSockets (see issue #769).
    // Tuned for consumer Wi-Fi where a single ~1s RTT spike is normal.
    _server->enableHeartbeat(3000, 2500, 2);

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
    // Subscribe to the global Task Watchdog so a hang in _server->loop()
    // (library bug, deadlock) triggers a panic+reset rather than silently
    // freezing teleop with WiFi still up.
    esp_task_wdt_add(NULL);

    while (_running.load()) {
        if (_server) _server->loop();
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(kLoopDelayMs));
    }
    esp_task_wdt_delete(NULL);
    if (_exitSemaphore) {
        xSemaphoreGive(static_cast<SemaphoreHandle_t>(_exitSemaphore));
    }
    vTaskDelete(NULL);
}

void WebSocketCommandProducer::onWsEvent(uint8_t clientNum, uint8_t type,
                                         const uint8_t* payload, size_t length) {
    auto wsType = static_cast<WStype_t>(type);
    switch (wsType) {
        case WStype_CONNECTED: {
            // Single-client policy: drop oldest, accept newest. The new client
            // pre-empts any stale slot holder so reconnects after a network blip
            // aren't bounced before the heartbeat evicts the dead connection.
            uint8_t prior = _activeClient.exchange(clientNum);
            _connected = true;
            if (prior != kNoClient) {
                Serial.printf("[ws] client %u took slot from %u\n", clientNum, prior);
                if (_server) _server->disconnect(prior);
            } else {
                Serial.printf("[ws] client %u connected\n", clientNum);
            }
            break;
        }
        case WStype_DISCONNECTED:
            // Only clear state if the disconnecting client is the active one.
            // A rejected client's later disconnect must not unset _connected.
            if (_activeClient.load() == clientNum) {
                _activeClient = kNoClient;
                _connected = false;
                Serial.printf("[ws] client %u disconnected\n", clientNum);
            }
            break;
        case WStype_TEXT: {
            // Drop frames from non-active clients (defense in depth: server
            // disconnect on rejection isn't always immediate).
            if (_activeClient.load() != clientNum) return;

            auto result = parseCommandFrame(payload, length);
            if (!result.ok()) {
                _parseFailCount.fetch_add(1, std::memory_order_relaxed);
                switch (result.error) {
                    case FrameParseError::EmptyOrTooLong:     logParseFail("len");     break;
                    case FrameParseError::InvalidVx:          logParseFail("vx");      break;
                    case FrameParseError::InvalidVy:          logParseFail("vy");      break;
                    case FrameParseError::InvalidOmega:       logParseFail("omega");   break;
                    case FrameParseError::InvalidControlVerb: logParseFail("control"); break;
                    default: break;
                }
                return;
            }

            const uint32_t beforeVelocityCount =
                _frameCount.load(std::memory_order_relaxed);
            dispatchFrame(_latch, _frameCount, result);

            // Periodic counter dump every 1000 *velocity* frames (~10s @ 100 Hz).
            // Control frames bypass the counter (rare, not stick throughput).
            if (result.kind == FrameKind::Velocity) {
                const uint32_t count = beforeVelocityCount + 1;
                if (count % 1000 == 0) {
                    Serial.printf("[ws] frames=%u parse_fail=%u\n",
                                  count,
                                  _parseFailCount.load(std::memory_order_relaxed));
                }
            }
            break;
        }
        case WStype_ERROR:
            Serial.printf("[ws] error event from client %u\n", clientNum);
            break;
        default:
            // BIN, FRAGMENT_*, PING, PONG — ignored.
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
