/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini
 */

#include "BTS7960Driver.h"
#include "debug.h"
#include <cmath>

BTS7960Driver::BTS7960Driver(DriverPins pins)
    : _hbridge(pins.l_en, pins.r_en, pins.l_pwm, pins.r_pwm) {}

void BTS7960Driver::begin() {
    _hbridge.Enable();
}

void BTS7960Driver::setOutput(float value) {
    BB8_ASSERT(value >= -1.0f && value <= 1.0f, "setOutput value out of range [-1,1]");

    if (value == 0.0f) {
        _hbridge.Disable();
        return;
    }

    _hbridge.Enable();
    uint8_t pwm = static_cast<uint8_t>(std::abs(value) * 255.0f);
    if (pwm == 0) {
        _hbridge.Disable();
        return;
    }
    if (value > 0.0f) {
        _hbridge.TurnLeft(pwm);
    } else {
        _hbridge.TurnRight(pwm);
    }
}

void BTS7960Driver::brake() {
    _hbridge.Stop();
}
