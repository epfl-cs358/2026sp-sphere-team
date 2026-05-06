/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini
 */

#pragma once

#include "L298NDriver.h"
#include "MotorConfig.h"
#include "FIT0186.h"

inline constexpr L298NPins MOTOR0_PINS = {.fwd = 14, .rev = 27};
inline constexpr L298NPins MOTOR1_PINS = {.fwd = 16, .rev = 17};
inline constexpr L298NPins MOTOR2_PINS = {.fwd = 18, .rev = 19};

// Placeholder encoder pins — update once wired
inline constexpr EncoderPins MOTOR0_ENCODER_PINS = {.a = 34, .b = 35};
inline constexpr EncoderPins MOTOR1_ENCODER_PINS = {.a = 32, .b = 33};
inline constexpr EncoderPins MOTOR2_ENCODER_PINS = {.a = 36, .b = 39};

inline constexpr MotorConfig MOTOR0_ENCODER = {.encoderPins = MOTOR0_ENCODER_PINS, .encoderCPR = fit0186::ENCODER_CPR};
inline constexpr MotorConfig MOTOR1_ENCODER = {.encoderPins = MOTOR1_ENCODER_PINS, .encoderCPR = fit0186::ENCODER_CPR};
inline constexpr MotorConfig MOTOR2_ENCODER = {.encoderPins = MOTOR2_ENCODER_PINS, .encoderCPR = fit0186::ENCODER_CPR};
