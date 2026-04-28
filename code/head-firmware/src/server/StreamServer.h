/*
 * BB-8 Head Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini, with assistance from Claude (Anthropic)
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <functional>
#include "StreamConfig.h"

class StreamServer {
public:
    using ConfigCallback = std::function<void(const StreamConfig&)>;

    virtual ~StreamServer() = default;
    virtual void begin(uint16_t port = 80) = 0;
    virtual uint8_t connectedClients() = 0;
    virtual void broadcastFrame(const uint8_t* data, size_t len) = 0;
    virtual void onConfigChange(ConfigCallback cb) = 0;
};
