#pragma once

#include <cmath>
#include "debug.h"

class PID {
public:
    PID(float kp, float ki, float kd, float outputMin, float outputMax,
        float deadband = 0.0f)
        : _kp(kp), _ki(ki), _kd(kd),
          _outputMin(outputMin), _outputMax(outputMax), _deadband(deadband),
          _integral(0.0f), _prevMeasurement(0.0f), _lastOutput(0.0f),
          _firstCompute(true) {}

    float compute(float setpoint, float measurement, float dt) {
        if (dt < 1e-4f) return _lastOutput;
        float rate = _firstCompute ? 0.0f : (measurement - _prevMeasurement) / dt;
        return compute(setpoint, measurement, rate, dt);
    }

    float compute(float setpoint, float measurement, float rate, float dt) {
        BB8_ASSERT(std::isfinite(setpoint), "PID: setpoint must be finite");
        BB8_ASSERT(std::isfinite(measurement), "PID: measurement must be finite");
        BB8_ASSERT(std::isfinite(rate), "PID: rate must be finite");
        BB8_ASSERT(dt >= 0.0f, "PID: dt must be non-negative");

        if (dt < 1e-4f) return _lastOutput;

        if (_deadband > 0.0f && std::abs(setpoint) < 1e-6f && std::abs(measurement) < _deadband) {
            reset();
            return 0.0f;
        }

        float error = setpoint - measurement;

        _integral += error * dt;

        // Integral clamping anti-windup
        if (_ki != 0.0f) {
            float integralMax = _outputMax / _ki;
            float integralMin = _outputMin / _ki;
            if (_integral > integralMax) _integral = integralMax;
            if (_integral < integralMin) _integral = integralMin;
        }

        float derivative = _firstCompute ? 0.0f : -rate;
        _firstCompute = false;

        float output = _kp * error + _ki * _integral + _kd * derivative;

        if (output > _outputMax) output = _outputMax;
        if (output < _outputMin) output = _outputMin;

        _prevMeasurement = measurement;
        _lastOutput = output;
        return output;
    }

    void reset() {
        _integral = 0.0f;
        _prevMeasurement = 0.0f;
        _lastOutput = 0.0f;
        _firstCompute = true;
    }

private:
    float _kp;
    float _ki;
    float _kd;
    float _outputMin;
    float _outputMax;

    float _deadband;
    float _integral;
    float _prevMeasurement;
    float _lastOutput;
    bool _firstCompute;
};
