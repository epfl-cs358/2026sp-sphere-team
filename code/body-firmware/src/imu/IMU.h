/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini, with assistance from Claude (Anthropic)
 */

#pragma once

template <typename T>
class IMU {
public:
    virtual ~IMU() = default;

    virtual bool begin() = 0;

    virtual T read() = 0;
};
