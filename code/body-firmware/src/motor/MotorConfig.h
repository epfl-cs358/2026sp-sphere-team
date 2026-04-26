/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini
 */

#pragma once

#include <cstdint>

struct DriverPins {
    uint8_t l_en;
    uint8_t r_en;
    uint8_t l_pwm;
    uint8_t r_pwm;
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
