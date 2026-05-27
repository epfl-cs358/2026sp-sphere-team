/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 */

#pragma once

// POD layout: no constructor, no methods, no virtuals. Must remain
// trivially-copyable so the live config can be hot-swapped via
// std::atomic<const BalanceConfig*> (see spec §"Two-buffer config").
struct BalanceConfig {
    // Command -> tilt-setpoint mapping
    float tiltPerVelocity;     // rad per (m/s)
    float maxTiltSetpoint;     // rad; clamp on commanded tilt

    // Pitch PID (forward/back)
    float pitchKp, pitchKi, pitchKd;
    // Roll PID (left/right) — separate so platform asymmetries can be tuned
    float rollKp,  rollKi,  rollKd;

    // Tilt-error deadband (rad). When the commanded tilt setpoint is ~0 AND
    // the actual tilt is within this band, the PID emits 0 and resets its
    // integrator — kills the small-signal limit cycle around upright.
    float pitchDeadband;
    float rollDeadband;

    // Output safety
    float maxOutputVelocity;   // m/s; clamp on |vx_out|, |vy_out|

    // Tilt-fault Schmitt thresholds — stored as sin(angle) for cheap compare
    float envelopeEnterSin;    // sin(60°) ≈ 0.866
    float envelopeExitSin;     // sin(55°) ≈ 0.819

    // Gyro axis sign overrides. Default +1; bring-up may set to -1 if the
    // BNO055 axis convention disagrees with our pitch/roll sign assumptions.
    float gyroPitchSign;       // multiplies imu.gyro.y when computing pitch rate
    float gyroRollSign;        // multiplies imu.gyro.x when computing roll rate

    // Yaw-rate PID (inner loop, PI-only — D would numerically differentiate
    // the gyro signal) and heading-hold P (outer loop).
    float yawRateKp, yawRateKi;
    float headingKp;
    float gyroYawSign;         // multiplies imu.gyro.z when computing yaw rate
};
