/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini, with assistance from Claude (Anthropic)
 */

#pragma once

#include "DrivetrainController.h"
#include "BodyVelocity.h"
#include "IMUReading.h"

class PassthroughDrivetrainController
    : public DrivetrainController<BodyVelocity, IMUReading, BodyVelocity> {
public:
    using DrivetrainController::DrivetrainController;

    void update(const BodyVelocity& command, const IMUReading& /*imuData*/, float /*dt*/) override {
        _drivetrain.drive(command);
    }

    void stop() override {
        _drivetrain.stop();
    }
};
