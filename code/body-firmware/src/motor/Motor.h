/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini, with assistance from Claude (Anthropic)
 */

#pragma once

class Motor {
public:
    virtual ~Motor() = default;

    // Set motor output. -1.0 = full reverse, 0.0 = stop, 1.0 = full forward.
    virtual void setSpeed(float speed) = 0;

    // Return current RPM estimated from encoder feedback.
    virtual float getRPM() = 0;
};
