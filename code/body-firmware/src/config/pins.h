/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini
 */

#pragma once

#include "L298NDriver.h"

// L298N channel B (IN3/IN4)
inline constexpr L298NPins MOTOR0_PINS = {.fwd = 27, .rev = 14};
inline constexpr L298NPins MOTOR1_PINS = {.fwd = 16, .rev = 17};
inline constexpr L298NPins MOTOR2_PINS = {.fwd = 18, .rev = 19};
