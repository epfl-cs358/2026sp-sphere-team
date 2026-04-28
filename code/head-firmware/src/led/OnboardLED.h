/*
 * BB-8 Head Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini, with assistance from Claude (Anthropic)
 */

#pragma once

#include <Arduino.h>
#include "StatusLED.h"

class OnboardLED : public StatusLED {
public:
    explicit OnboardLED(uint8_t pin) : _pin(pin) {}

    void begin() override {
        pinMode(_pin, OUTPUT);
        digitalWrite(_pin, LOW);
    }

    void setState(LEDState state) override {
        _state = state;
        _lastToggle = 0;
        _ledOn = false;
    }

    void update(unsigned long nowMs) override {
        switch (_state) {
            case LEDState::Connecting:
                if (_lastToggle == 0 && !_ledOn) {
                    _ledOn = true;
                    _lastToggle = nowMs;
                } else if (nowMs - _lastToggle >= BLINK_INTERVAL_MS) {
                    _ledOn = !_ledOn;
                    _lastToggle = nowMs;
                }
                digitalWrite(_pin, _ledOn ? HIGH : LOW);
                break;
            case LEDState::Ready:
            case LEDState::Streaming:
                digitalWrite(_pin, HIGH);
                break;
            case LEDState::Error:
                digitalWrite(_pin, LOW);
                break;
        }
    }

private:
    uint8_t _pin;
    LEDState _state = LEDState::Error;
    bool _ledOn = false;
    unsigned long _lastToggle = 0;
    static constexpr unsigned long BLINK_INTERVAL_MS = 250;
};
