/*
 * BB-8 Head Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini, with assistance from Claude (Anthropic)
 */

#pragma once

#include <cstdint>

enum class LEDState : uint8_t { Connecting, Ready, Streaming, Error };

class StatusLED {
public:
    virtual ~StatusLED() = default;
    virtual void begin() = 0;
    virtual void setState(LEDState state) = 0;
    virtual void update(unsigned long nowMs) = 0;
};
