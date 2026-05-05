/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini
 */

#pragma once

#include <cstdint>

struct EncoderPins {
    uint8_t a;
    uint8_t b;
};

struct MotorConfig {
    EncoderPins encoderPins;
    uint16_t encoderCPR;
};
