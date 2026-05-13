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
inline constexpr L298NPins MOTOR2_PINS = {.fwd = 13, .rev = 12};

inline constexpr EncoderPins MOTOR0_ENCODER_PINS = {.a = 35, .b = 34};
inline constexpr EncoderPins MOTOR1_ENCODER_PINS = {.a = 2, .b = 4};
inline constexpr EncoderPins MOTOR2_ENCODER_PINS = {.a = 36, .b = 39};

inline constexpr MotorConfig MOTOR0_ENCODER = {.encoderPins = MOTOR0_ENCODER_PINS, .encoderCPR = fit0186::ENCODER_CPR};
inline constexpr MotorConfig MOTOR1_ENCODER = {.encoderPins = MOTOR1_ENCODER_PINS, .encoderCPR = fit0186::ENCODER_CPR};
inline constexpr MotorConfig MOTOR2_ENCODER = {.encoderPins = MOTOR2_ENCODER_PINS, .encoderCPR = fit0186::ENCODER_CPR};
