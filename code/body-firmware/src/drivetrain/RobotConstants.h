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

// Yaw control envelope. OMEGA_MAX bounds inner yaw-rate PID output and (in
// Commit 4) clamps the outer heading-P output before it feeds the inner loop.
// YAW_SPIN_THRESHOLD is the |gyro.z| above which the controller assumes a
// runaway spin (720 deg/s ≈ 12.566 rad/s); sustained for YAW_SPIN_TRIP_MS the
// recovery latch fires and omega is forced to 0 until rate drops back below.
constexpr float    OMEGA_MAX             = 3.5f;     // rad/s (~200 dps)
constexpr float    YAW_SPIN_THRESHOLD    = 12.566f;  // rad/s (~720 dps)
constexpr uint32_t YAW_SPIN_TRIP_MS      = 100;

// Teleop control-loop timing.
constexpr uint32_t STALENESS_TIMEOUT_MS = 200;   // ramp-to-zero window on silence
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
        .pitchDeadband     = 0.0f,    // rad; off by default, raise to kill limit cycle at rest
        .rollDeadband      = 0.0f,
        .maxOutputVelocity = 1.00f,   // m/s; just below physical max (~1.05)
        .envelopeEnterSin  = 0.866f,  // sin(60°)
        .envelopeExitSin   = 0.819f,  // sin(55°)
        .gyroPitchSign     = 1.0f,
        .gyroRollSign      = 1.0f,
        .yawRateKp = 0.0f, .yawRateKi = 0.0f,
        // Conservative starter for heading hold. Engaged by default at arm
        // time; expect to tune at the bench. Zero would keep heading hold
        // ON in shape but emit zero rate command — set non-zero so the
        // default-flashed firmware actually holds heading.
        .headingKp         = 1.0f,
        .gyroYawSign       = 1.0f,
    };
}

}
