#pragma once

#include <cmath>
#include <cstdint>
#include "BalanceConfig.h"
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

inline BalanceConfig balanceConfig() {
    return {
        .tiltPerVelocity   = 0.30f,   // 1 m/s → ~17° tilt
        .maxTiltSetpoint   = 0.35f,   // ~20°; leaves ample headroom below 60° fault envelope
        .pitchKp = 1.50f, .pitchKi = 0.0f, .pitchKd = 0.15f,
        .rollKp  = 1.50f, .rollKi  = 0.0f, .rollKd  = 0.15f,
        .maxOutputVelocity = 1.00f,   // m/s; just below physical max (~1.05)
        .envelopeEnterSin  = 0.866f,  // sin(60°)
        .envelopeExitSin   = 0.819f,  // sin(55°)
        .gyroPitchSign     = 1.0f,
        .gyroRollSign      = 1.0f,
    };
}

}
