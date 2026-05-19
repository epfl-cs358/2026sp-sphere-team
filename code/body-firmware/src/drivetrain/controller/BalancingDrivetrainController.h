/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 */

#pragma once

#include <atomic>

#include "DrivetrainController.h"
#include "BodyVelocity.h"
#include "IMUReading.h"
#include "BalanceConfig.h"
#include "BalanceTelemetry.h"
#include "PID.h"

class BalanceTuner;

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
    void onArmed() override;
    void onDisarmed() override;
    void resetIntegrators();

    // Wave 3 main wiring calls this once during setup to attach the live tuner
    // so the controller can consume pending GAIN_CHANGED / CONFIG_SAVED /
    // CONFIG_RESET event bits into each tick's telemetry. Null is valid —
    // event_flags will reflect only PID + fault state in that case.
    void setTuner(BalanceTuner* tuner) { _tuner = tuner; }

    // Last-tick controller telemetry. Wheel_* fields and armed_state /
    // cmd_stale are left as defaults and overwritten by the control task
    // before publishing — see plan §4 "Producer wiring".
    const BalanceTelemetry& lastTelemetry() const { return _lastTelemetry; }

#ifdef BB8_TEST_HOOKS
    // Test-only seams. Direct fault-latch manipulation so unit tests can pin
    // the latch's behavior without routing through quatToBodyGravity (whose
    // sign convention is the deferred B1 question — coupling tests to it
    // means a future sign flip breaks tests for the wrong reason).
    void _setFaultForTest(bool v) { _inFault = v; }
    bool _faultForTest() const { return _inFault; }
#endif

private:
    void _finalizeTelemetry();

    PID  _pitchPid;
    PID  _rollPid;
    bool _inFault = false;
    bool _prevInFault = false;
    std::atomic<const BalanceConfig*>& _configSlot;
    BalanceTuner* _tuner = nullptr;
    BalanceTelemetry _lastTelemetry{};
};
