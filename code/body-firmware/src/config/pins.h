/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini
 */

#pragma once

#include "MotorConfig.h"
#include "FIT0186.h"

// TODO: Replace placeholder pin values with actual wiring
inline constexpr MotorConfig MOTOR0 {
    .driverPins = {.rpwm = 0, .lpwm = 0},
    .encoderPins = {.a = 0, .b = 0},
    .encoderCPR = fit0186::ENCODER_CPR
};

inline constexpr MotorConfig MOTOR1 {
    .driverPins = {.rpwm = 0, .lpwm = 0},
    .encoderPins = {.a = 0, .b = 0},
    .encoderCPR = fit0186::ENCODER_CPR
};

inline constexpr MotorConfig MOTOR2 {
    .driverPins = {.rpwm = 0, .lpwm = 0},
    .encoderPins = {.a = 0, .b = 0},
    .encoderCPR = fit0186::ENCODER_CPR
};
