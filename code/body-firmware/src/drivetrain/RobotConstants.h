#pragma once

#include <cmath>
#include "DrivetrainConfig.h"

namespace RobotConstants {

constexpr float WHEEL_RADIUS  = 0.046225f;  // 9.245 cm diameter / 2, in meters
constexpr float ROBOT_RADIUS  = 0.1745f;    // center to wheel contact, in meters
constexpr float TILT_ANGLE    = 30.0f * static_cast<float>(M_PI) / 180.0f;
constexpr float MAX_RPM       = 251.0f;

inline DrivetrainConfig drivetrainConfig() {
    return {
        .wheelRadius = WHEEL_RADIUS,
        .robotRadius = ROBOT_RADIUS,
        .tiltAngle   = TILT_ANGLE,
        .maxRPM      = MAX_RPM,
    };
}

}
