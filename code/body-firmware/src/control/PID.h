#pragma once

class PID {
public:
    PID(float kp, float ki, float kd, float outputMin, float outputMax)
        : _kp(kp), _ki(ki), _kd(kd),
          _outputMin(outputMin), _outputMax(outputMax),
          _integral(0.0f), _prevMeasurement(0.0f), _firstCompute(true) {}

    float compute(float setpoint, float measurement, float dt) {
        float error = setpoint - measurement;

        _integral += error * dt;

        if (_ki != 0.0f) {
            float integralMax = _outputMax / _ki;
            float integralMin = _outputMin / _ki;
            if (_integral > integralMax) _integral = integralMax;
            if (_integral < integralMin) _integral = integralMin;
        }

        float derivative = 0.0f;
        if (!_firstCompute && dt > 1e-6f) {
            derivative = -(measurement - _prevMeasurement) / dt;
        }
        _firstCompute = false;

        float output = _kp * error + _ki * _integral + _kd * derivative;

        if (output > _outputMax) output = _outputMax;
        if (output < _outputMin) output = _outputMin;

        _prevMeasurement = measurement;
        return output;
    }

    void reset() {
        _integral = 0.0f;
        _prevMeasurement = 0.0f;
        _firstCompute = true;
    }

private:
    float _kp;
    float _ki;
    float _kd;
    float _outputMin;
    float _outputMax;

    float _integral;
    float _prevMeasurement;
    bool _firstCompute;
};
