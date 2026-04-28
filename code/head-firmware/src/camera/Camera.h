/*
 * BB-8 Head Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini, with assistance from Claude (Anthropic)
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include "StreamConfig.h"

class Camera {
public:
    virtual ~Camera() = default;
    virtual bool begin(const StreamConfig& config) = 0;
    virtual bool applyConfig(const StreamConfig& config) = 0;
    virtual const uint8_t* capture(size_t& outLen) = 0;
    virtual void release() = 0;
};
