/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 */

#pragma once

#include <atomic>

#include "DrivetrainController.h"
#include "BodyVelocity.h"
#include "IMUReading.h"
#include "BalanceConfig.h"
#include "PID.h"

class BalancingDrivetrainController
    : public DrivetrainController<BodyVelocity, IMUReading, BodyVelocity> {
public:
    BalancingDrivetrainController(Drivetrain<BodyVelocity>& drive,
                                  IMU<IMUReading>& imu,
                                  std::atomic<const BalanceConfig*>& configSlot);

    void update(const BodyVelocity& cmd,
                const IMUReading& imuData,
                float dt) override;
    void stop() override;

private:
    PID  _pitchPid;
    PID  _rollPid;
    bool _inFault = false;
    std::atomic<const BalanceConfig*>& _configSlot;
};
