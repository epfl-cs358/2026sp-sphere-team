/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini, with assistance from Claude (Anthropic)
 */

#pragma once

#include "Drivetrain.h"
#include "IMU.h"

template <typename TCommand, typename TIMUData, typename TVelocity>
class DrivetrainController {
public:
    DrivetrainController(Drivetrain<TVelocity>& drivetrain, IMU<TIMUData>& imu)
        : _drivetrain(drivetrain), _imu(imu) {}

    virtual ~DrivetrainController() = default;

    // Process one control cycle: read IMU, interpret command, drive.
    virtual void update(const TCommand& command, const TIMUData& imuData, float dt) = 0;

    // Emergency stop.
    virtual void stop() = 0;

    // Arming-edge hooks. Defaults are no-ops so controllers that don't
    // accumulate state across arming cycles compile unchanged. Concrete
    // controllers should override to clear stale PID state (including any
    // wheel-level PIDs inside the drivetrain) so the first armed tick
    // doesn't kick from leftover Disarmed-period state.
    virtual void onArmed()    {}
    virtual void onDisarmed() {}

protected:
    Drivetrain<TVelocity>& _drivetrain;
    IMU<TIMUData>& _imu;
};
