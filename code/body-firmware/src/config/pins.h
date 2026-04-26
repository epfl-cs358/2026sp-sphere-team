/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini
 */

#pragma once

#include "MotorConfig.h"
#include "FIT0186.h"

// TODO: Replace placeholder pin values with actual wiring
// 0xFF = unconfigured sentinel
inline constexpr MotorConfig MOTOR0 {
    .driverPins = {.l_en = 0xFF, .r_en = 0xFF, .l_pwm = 0xFF, .r_pwm = 0xFF},
    .encoderPins = {.a = 0xFF, .b = 0xFF},
    .encoderCPR = fit0186::ENCODER_CPR
};

inline constexpr MotorConfig MOTOR1 {
    .driverPins = {.l_en = 0xFF, .r_en = 0xFF, .l_pwm = 0xFF, .r_pwm = 0xFF},
    .encoderPins = {.a = 0xFF, .b = 0xFF},
    .encoderCPR = fit0186::ENCODER_CPR
};

inline constexpr MotorConfig MOTOR2 {
    .driverPins = {.l_en = 0xFF, .r_en = 0xFF, .l_pwm = 0xFF, .r_pwm = 0xFF},
    .encoderPins = {.a = 0xFF, .b = 0xFF},
    .encoderCPR = fit0186::ENCODER_CPR
};
