#pragma once

#include <cmath>
#include <cstdint>
#include "DrivetrainConfig.h"

namespace RobotConstants {

constexpr float WHEEL_RADIUS  = 0.046225f;  // 9.245 cm diameter / 2, in meters
constexpr float ROBOT_RADIUS  = 0.1745f;    // center to wheel contact, in meters
constexpr float TILT_ANGLE    = 30.0f * static_cast<float>(M_PI) / 180.0f;
constexpr float MAX_RPM       = 251.0f;

// Teleop control-loop timing.
constexpr uint32_t STALENESS_TIMEOUT_MS = 200;   // ramp-to-zero window on silence
constexpr uint32_t STALE_DISARM_MS      = 2000;  // long-term auto-disarm backstop
constexpr uint32_t CONTROL_PERIOD_MS    = 10;    // 100 Hz tick

inline DrivetrainConfig drivetrainConfig() {
    constexpr float DEG = static_cast<float>(M_PI) / 180.0f;
    return {
        .wheelRadius = WHEEL_RADIUS,
        .robotRadius = ROBOT_RADIUS,
        .tiltAngle   = TILT_ANGLE,
        .maxRPM      = MAX_RPM,
        // Pull config: motor 0 at back (180°), motors 1 & 2 at front-right (300°)
        // and front-left (60°). Drive wheels load up under forward acceleration.
        .wheelAngles = {180.0f * DEG, 300.0f * DEG, 60.0f * DEG},
    };
}

}
