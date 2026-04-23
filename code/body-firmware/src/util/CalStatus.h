/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini, with assistance from Claude (Anthropic)
 */

#pragma once

#include <cstdint>

struct CalStatus {
    uint8_t sys = 0;
    uint8_t gyro = 0;
    uint8_t accel = 0;
    uint8_t mag = 0;
};
