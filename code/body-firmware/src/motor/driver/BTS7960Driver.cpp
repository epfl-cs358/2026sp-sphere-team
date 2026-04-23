/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini
 */

#include "BTS7960Driver.h"
#include "debug.h"
#include <cmath>

BTS7960Driver::BTS7960Driver(DriverPins pins)
    : BTS7960Driver(pins.rpwm, pins.lpwm) {}

// BTS7960 lib constructor is (L_EN, R_EN, L_PWM, R_PWM).
// We tie enable pins to PWM pins. If hardware has separate EN pins,
// DriverPins and this constructor need updating.
BTS7960Driver::BTS7960Driver(uint8_t rpwm, uint8_t lpwm)
    : _hbridge(rpwm, lpwm, rpwm, lpwm) {}

void BTS7960Driver::begin() {
    _hbridge.Enable();
}

void BTS7960Driver::setOutput(float value) {
    BB8_ASSERT(value >= -1.0f && value <= 1.0f, "setOutput value out of range [-1,1]");

    if (value == 0.0f) {
        _hbridge.Disable();
        return;
    }

    int8_t pwm = static_cast<int8_t>(std::abs(value) * 127.0f);
    if (value > 0.0f) {
        _hbridge.TurnLeft(pwm);
    } else {
        _hbridge.TurnRight(pwm);
    }
}

void BTS7960Driver::brake() {
    _hbridge.Stop();
}
