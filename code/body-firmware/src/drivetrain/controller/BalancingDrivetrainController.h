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
    void resetIntegrators();

#ifdef BB8_TEST_HOOKS
    // Test-only seams. Direct fault-latch manipulation so unit tests can pin
    // the latch's behavior without routing through quatToBodyGravity (whose
    // sign convention is the deferred B1 question — coupling tests to it
    // means a future sign flip breaks tests for the wrong reason).
    void _setFaultForTest(bool v) { _inFault = v; }
    bool _faultForTest() const { return _inFault; }
#endif

private:
    PID  _pitchPid;
    PID  _rollPid;
    bool _inFault = false;
    std::atomic<const BalanceConfig*>& _configSlot;
};
