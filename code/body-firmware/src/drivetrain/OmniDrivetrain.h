#pragma once

#include "Drivetrain.h"
#include "OmniKinematics.h"
#include "PID.h"
#include "BodyVelocity.h"
#include "DrivetrainConfig.h"

class OmniDrivetrain : public Drivetrain<BodyVelocity> {
public:
    OmniDrivetrain(Motor& m0, Motor& m1, Motor& m2,
                   const DrivetrainConfig& config,
                   PID& pid0, PID& pid1, PID& pid2)
        : Drivetrain(m0, m1, m2)
        , _kinematics(config)
        , _pids{&pid0, &pid1, &pid2} {}

    void drive(const BodyVelocity& velocity) override {
        auto rpms = _kinematics.toWheelRPMs(velocity);
        for (int i = 0; i < 3; i++) {
            _targetRPMs[i] = rpms[i];
        }
    }

    void update(float dt) override {
        for (auto* m : _motors) m->update();

        float s0 = _pids[0]->compute(_targetRPMs[0], _motors[0]->getFilteredRPM(), dt);
        float s1 = _pids[1]->compute(_targetRPMs[1], _motors[1]->getFilteredRPM(), dt);
        float s2 = _pids[2]->compute(_targetRPMs[2], _motors[2]->getFilteredRPM(), dt);

        setMotorSpeeds(s0, s1, s2);
    }

    std::array<float, 3> getTargetRPMs() const {
        return {_targetRPMs[0], _targetRPMs[1], _targetRPMs[2]};
    }

    void stop() override {
        for (auto* m : _motors) m->brake();
        for (auto* p : _pids) p->reset();
        for (int i = 0; i < 3; i++) _targetRPMs[i] = 0.0f;
    }

    // Pure inner-loop reset: clears per-wheel PID state and zeroes
    // _targetRPMs but does NOT brake the motors. Used on arming-edge
    // transitions so the next update(dt) recomputes cleanly with
    // _firstCompute=true instead of D-spiking against a stale
    // _prevMeasurement accumulated while the drivetrain was running
    // unarmed.
    void resetPids() override {
        for (auto* p : _pids) p->reset();
        for (int i = 0; i < 3; i++) _targetRPMs[i] = 0.0f;
    }

private:
    void setMotorSpeeds(float s0, float s1, float s2) {
        _motors[0]->setSpeed(s0);
        _motors[1]->setSpeed(s1);
        _motors[2]->setSpeed(s2);
    }

    OmniKinematics _kinematics;
    PID* _pids[3];
    float _targetRPMs[3] = {};
};
