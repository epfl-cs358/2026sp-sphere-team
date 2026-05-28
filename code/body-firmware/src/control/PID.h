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
          _firstCompute(true),
          _lastP(0.0f), _lastI(0.0f), _lastD(0.0f),
          _lastError(0.0f), _lastIntegral(0.0f),
          _lastWasDeadbandReset(false),
          _lastWasISaturated(false),
          _lastWasOutSaturated(false) {}

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

        _lastWasDeadbandReset = false;
        _lastWasISaturated = false;
        _lastWasOutSaturated = false;

        if (dt < 1e-4f) return _lastOutput;

        if (_deadband > 0.0f && std::abs(setpoint) < 1e-6f && std::abs(measurement) < _deadband) {
            reset();
            _lastWasDeadbandReset = true;
            return 0.0f;
        }

        float error = setpoint - measurement;

        _integral += error * dt;

        // Integral clamping anti-windup
        if (_ki != 0.0f) {
            float integralMax = _outputMax / _ki;
            float integralMin = _outputMin / _ki;
            if (_integral > integralMax) { _integral = integralMax; _lastWasISaturated = true; }
            if (_integral < integralMin) { _integral = integralMin; _lastWasISaturated = true; }
        }

        float derivative = _firstCompute ? 0.0f : -rate;
        _firstCompute = false;

        float output = _kp * error + _ki * _integral + _kd * derivative;

        _lastP = _kp * error;
        _lastI = _ki * _integral;
        _lastD = _kd * derivative;
        _lastError = error;
        _lastIntegral = _integral;

        if (output > _outputMax) { output = _outputMax; _lastWasOutSaturated = true; }
        if (output < _outputMin) { output = _outputMin; _lastWasOutSaturated = true; }

        _prevMeasurement = measurement;
        _lastOutput = output;
        return output;
    }

    void reset() {
        _integral = 0.0f;
        _prevMeasurement = 0.0f;
        _lastOutput = 0.0f;
        _firstCompute = true;
        _lastP = 0.0f;
        _lastI = 0.0f;
        _lastD = 0.0f;
        _lastError = 0.0f;
        _lastIntegral = 0.0f;
        _lastWasDeadbandReset = false;
        _lastWasISaturated = false;
        _lastWasOutSaturated = false;
    }

    // Hot-swap gains without re-constructing the PID. Used by the balancing
    // controller to apply live-tuned BalanceConfig gains every control tick.
    void setGains(float kp, float ki, float kd) {
        _kp = kp;
        _ki = ki;
        _kd = kd;
    }

    void setDeadband(float d) {
        _deadband = d;
    }

    float lastP() const { return _lastP; }
    float lastI() const { return _lastI; }
    float lastD() const { return _lastD; }
    float lastError() const { return _lastError; }
    float lastIntegral() const { return _lastIntegral; }
    float lastOutput() const { return _lastOutput; }
    bool wasDeadbandReset() const { return _lastWasDeadbandReset; }
    bool wasISaturated() const { return _lastWasISaturated; }
    bool wasOutSaturated() const { return _lastWasOutSaturated; }

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

    float _lastP;
    float _lastI;
    float _lastD;
    float _lastError;
    float _lastIntegral;
    bool _lastWasDeadbandReset;
    bool _lastWasISaturated;
    bool _lastWasOutSaturated;
};
