/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini
 */

#pragma once

#include "Motor.h"
#include "Driver.h"
#include "MotorConfig.h"
#include "MovingAverage.h"
#include "FIT0186.h"
#include "debug.h"
#include <ESP32Encoder.h>
#include <Arduino.h>

template <size_t FilterN = 5>
class FIT0186Motor : public Motor {
public:
    FIT0186Motor(Driver& driver, EncoderPins encoderPins, uint16_t encoderCPR)
        : _driver(driver)
        , _encoderPins(encoderPins)
        , _encoderCPR(encoderCPR) {
        BB8_ASSERT(encoderCPR > 0, "encoderCPR must be > 0");
    }

    FIT0186Motor(Driver& driver, const MotorConfig& config)
        : FIT0186Motor(driver, config.encoderPins, config.encoderCPR) {}

    void begin() override {
        _driver.begin();
        _encoder.attachFullQuad(_encoderPins.a, _encoderPins.b);
        _lastCount = _encoder.getCount();
        _lastUpdateMicros = micros();
    }

    void update() override {
        unsigned long now = micros();
        float dtSeconds = static_cast<float>(now - _lastUpdateMicros) / 1'000'000.0f;

        if (dtSeconds <= 0.0f) return;

        int64_t count = _encoder.getCount();
        int64_t deltaCount = count - _lastCount;

        _rawRPM = (static_cast<float>(deltaCount) / static_cast<float>(_encoderCPR))
                  * (60.0f / dtSeconds);

        _filter.push(_rawRPM);
        _lastCount = count;
        _lastUpdateMicros = now;
    }

    void setSpeed(float speed) override {
        _driver.setOutput(speed);
    }

    void brake() override {
        _driver.brake();
    }

    float getRPM() override {
        return _rawRPM;
    }

    float getFilteredRPM() override {
        return _filter.average();
    }

    // Public for testing — test code sets _encoder._count directly
    ESP32Encoder _encoder;

private:
    Driver& _driver;
    EncoderPins _encoderPins;
    uint16_t _encoderCPR;

    int64_t _lastCount = 0;
    unsigned long _lastUpdateMicros = 0;
    float _rawRPM = 0.0f;
    MovingAverage<FilterN> _filter;
};
