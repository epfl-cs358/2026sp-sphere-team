#pragma once

#include <array>
#include <cmath>
#include "DrivetrainConfig.h"
#include "BodyVelocity.h"

class OmniKinematics {
public:
    explicit OmniKinematics(const DrivetrainConfig& config) : _config(config) {}

    std::array<float, 3> toWheelRPMs(const BodyVelocity& v) const {
        constexpr float SIN_0   =  0.0f;
        constexpr float COS_0   =  1.0f;
        constexpr float SIN_120 =  0.86602540378f;
        constexpr float COS_120 = -0.5f;
        constexpr float SIN_240 = -0.86602540378f;
        constexpr float COS_240 = -0.5f;

        float cosAlpha = cosf(_config.tiltAngle);
        float scale = 1.0f / (_config.wheelRadius * cosAlpha);

        float R = _config.robotRadius;

        float w0 = scale * (SIN_0   * v.vx + COS_0   * v.vy + R * v.omega);
        float w1 = scale * (SIN_120 * v.vx + COS_120 * v.vy + R * v.omega);
        float w2 = scale * (SIN_240 * v.vx + COS_240 * v.vy + R * v.omega);

        constexpr float RAD_TO_RPM = 60.0f / (2.0f * static_cast<float>(M_PI));
        std::array<float, 3> rpms = {w0 * RAD_TO_RPM, w1 * RAD_TO_RPM, w2 * RAD_TO_RPM};

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
};
