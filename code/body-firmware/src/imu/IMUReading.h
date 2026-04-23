/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini, with assistance from Claude (Anthropic)
 */

#pragma once

#include <cstdint>
#include "Vec3.h"
#include "Quat.h"
#include "Euler.h"
#include "CalStatus.h"

enum class IMUField : uint8_t {
    Quaternion  = 1 << 0,
    Euler       = 1 << 1,
    LinearAccel = 1 << 2,
    Gyro        = 1 << 3,
    Gravity     = 1 << 4,
    Calibration = 1 << 5,
};

inline IMUField operator|(IMUField a, IMUField b) {
    return static_cast<IMUField>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
}

inline bool operator&(IMUField a, IMUField b) {
    return (static_cast<uint8_t>(a) & static_cast<uint8_t>(b)) != 0;
}

struct IMUReading {
    Quat orientation;
    ::Euler euler;
    Vec3 linearAccel;
    Vec3 gyro;
    Vec3 gravity;
    CalStatus calibration;
};
