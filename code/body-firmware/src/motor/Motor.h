/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini, with assistance from Claude (Anthropic)
 */

#pragma once

class Motor {
public:
    virtual ~Motor() = default;

    virtual void begin() = 0;

    // Refresh encoder state and recompute RPM. Call at a fixed interval.
    virtual void update() = 0;

    // Set motor output. -1.0 = full reverse, 0.0 = coast, 1.0 = full forward.
    virtual void setSpeed(float speed) = 0;

    // Active braking — shorts motor terminals for fast stop.
    virtual void brake() = 0;

    // Return raw RPM from last update() call.
    virtual float getRPM() = 0;

    // Return smoothed RPM (moving average) from last update() call.
    virtual float getFilteredRPM() = 0;
};
