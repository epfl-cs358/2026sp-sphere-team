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
    virtual void update(const TCommand& command, const TIMUData& imuData) = 0;

    // Emergency stop.
    virtual void stop() = 0;

protected:
    Drivetrain<TVelocity>& _drivetrain;
    IMU<TIMUData>& _imu;
};
