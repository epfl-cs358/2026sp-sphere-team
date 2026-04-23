/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini
 */

#pragma once

#include <cstdint>

struct DriverPins {
    uint8_t rpwm;
    uint8_t lpwm;
};

struct EncoderPins {
    uint8_t a;
    uint8_t b;
};

struct MotorConfig {
    DriverPins driverPins;
    EncoderPins encoderPins;
    uint16_t encoderCPR;
};
