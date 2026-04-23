/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini
 */

#pragma once

class Driver {
public:
    virtual ~Driver() = default;

    virtual void begin() = 0;

    // Set output. -1.0 = full reverse, 0.0 = coast, 1.0 = full forward.
    virtual void setOutput(float value) = 0;

    // Active braking.
    virtual void brake() = 0;
};
