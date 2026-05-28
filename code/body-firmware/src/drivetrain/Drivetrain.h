/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini, with assistance from Claude (Anthropic)
 */

#pragma once

#include <array>

#include "Motor.h"

// Per-wheel snapshot of the inner control loop, surfaced for telemetry/tuning.
// Populated by concrete drivetrains that maintain wheel-level PIDs.
struct WheelTelemetry {
    float target_rpm;
    float meas_rpm;
    float P;
    float I;
    float D;
    float out;
};

template <typename TVelocity>
class Drivetrain {
public:
    Drivetrain(Motor& m0, Motor& m1, Motor& m2)
        : _motors{&m0, &m1, &m2} {}

    virtual ~Drivetrain() = default;

    // Accept a high-level velocity command and compute per-wheel targets.
    virtual void drive(const TVelocity& velocity) = 0;

    // Run per-wheel PID. Call every control loop iteration with elapsed dt in seconds.
    virtual void update(float dt) = 0;

    // Emergency stop — zero all motors immediately.
    virtual void stop() = 0;

    // Clear any inner control-loop state (e.g. per-wheel PIDs) without
    // commanding the motors. Concrete drivetrains that maintain wheel-level
    // PID state should override; the default is a no-op so drivetrains
    // without inner loops compile unchanged.
    virtual void resetPids() {}

    // Per-wheel telemetry snapshot. Default returns zero-initialized values so
    // drivetrains without inner PID loops compile and report sensibly.
    virtual std::array<WheelTelemetry, 3> getWheelTelemetry() const {
        return std::array<WheelTelemetry, 3>{};
    }

protected:
    Motor* _motors[3];
};
