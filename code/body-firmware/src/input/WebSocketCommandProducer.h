/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini, with assistance from Claude (Anthropic)
 */

#pragma once

#include "CommandProducer.h"
#include "CommandLatch.h"
#include "BodyVelocity.h"

#include <atomic>
#include <cstdint>

class WebSocketsServer;  // forward decl from arduinoWebSockets

class WebSocketCommandProducer : public CommandProducer {
public:
    WebSocketCommandProducer(CommandLatch<BodyVelocity>& latch, uint16_t port = 81);
    ~WebSocketCommandProducer() override;

    WebSocketCommandProducer(const WebSocketCommandProducer&) = delete;
    WebSocketCommandProducer& operator=(const WebSocketCommandProducer&) = delete;

    void start() override;
    void stop() override;
    bool connected() const override;

private:
    static void taskTrampoline(void* arg);
    void taskBody();
    void onWsEvent(uint8_t clientNum, uint8_t type, const uint8_t* payload, size_t length);
    void logParseFail(const char* field);

    CommandLatch<BodyVelocity>& _latch;
    uint16_t _port;
    WebSocketsServer* _server;          // heap-allocated to avoid pulling library header into ours
    void* _taskHandle;                  // TaskHandle_t under the hood
    void* _exitSemaphore;               // SemaphoreHandle_t under the hood
    std::atomic<bool> _running;
    std::atomic<bool> _connected;
    uint32_t _lastParseFailLogMs = 0;
};
