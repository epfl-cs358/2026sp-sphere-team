/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini
 */

#include "L298NDriver.h"
#include "debug.h"
#include <Arduino.h>
#include <cmath>

L298NDriver::L298NDriver(L298NPins pins)
    : _pins(pins) {}

void L298NDriver::begin() {
    pinMode(_pins.fwd, OUTPUT);
    pinMode(_pins.rev, OUTPUT);
}

void L298NDriver::setOutput(float value) {
    BB8_ASSERT(value >= -1.0f && value <= 1.0f, "setOutput value out of range [-1,1]");

    uint8_t pwm = static_cast<uint8_t>(std::abs(value) * 255.0f);

    if (pwm == 0) {
        digitalWrite(_pins.fwd, LOW);
        digitalWrite(_pins.rev, LOW);
        return;
    }

    if (value > 0.0f) {
        analogWrite(_pins.fwd, pwm);
        digitalWrite(_pins.rev, LOW);
    } else {
        digitalWrite(_pins.fwd, LOW);
        analogWrite(_pins.rev, pwm);
    }
}

void L298NDriver::brake() {
    digitalWrite(_pins.fwd, HIGH);
    digitalWrite(_pins.rev, HIGH);
}
