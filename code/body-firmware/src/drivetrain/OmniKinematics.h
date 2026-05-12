#pragma once

#include <array>
#include <cmath>
#include "debug.h"
#include "DrivetrainConfig.h"
#include "BodyVelocity.h"

class OmniKinematics {
public:
    explicit OmniKinematics(const DrivetrainConfig& config) : _config(config) {
        BB8_ASSERT(config.wheelRadius > 0.0f, "OmniKinematics: wheelRadius must be > 0");
        BB8_ASSERT(config.robotRadius > 0.0f, "OmniKinematics: robotRadius must be > 0");
        BB8_ASSERT(config.maxRPM > 0.0f, "OmniKinematics: maxRPM must be > 0");
        BB8_ASSERT(std::abs(cosf(config.tiltAngle)) > 1e-6f,
                   "OmniKinematics: tiltAngle too close to pi/2");
        for (int i = 0; i < 3; i++) {
            _sin[i] = sinf(config.wheelAngles[i]);
            _cos[i] = cosf(config.wheelAngles[i]);
        }
    }

    std::array<float, 3> toWheelRPMs(const BodyVelocity& v) const {
        const float scale = 1.0f / (_config.wheelRadius * cosf(_config.tiltAngle));
        const float R = _config.robotRadius;
        constexpr float RAD_TO_RPM = 60.0f / (2.0f * static_cast<float>(M_PI));

        std::array<float, 3> rpms;
        for (int i = 0; i < 3; i++) {
            const float w = scale * (-_sin[i] * v.vx - _cos[i] * v.vy - R * v.omega);
            rpms[i] = w * RAD_TO_RPM;
        }

        float maxAbs = 0.0f;
        for (int i = 0; i < 3; i++) {
            float a = fabsf(rpms[i]);
            if (a > maxAbs) maxAbs = a;
        }

        if (maxAbs > _config.maxRPM) {
            float factor = _config.maxRPM / maxAbs;
            for (int i = 0; i < 3; i++) {
                rpms[i] *= factor;
            }
        }

        return rpms;
    }

private:
    DrivetrainConfig _config;
    float _sin[3];
    float _cos[3];
};
