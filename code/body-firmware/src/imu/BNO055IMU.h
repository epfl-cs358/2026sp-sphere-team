/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini, with assistance from Claude (Anthropic)
 */

#pragma once

#include "IMU.h"
#include "IMUReading.h"
#include <Adafruit_BNO055.h>
#include <Preferences.h>
#include <Arduino.h>
#include "debug.h"

class BNO055IMU : public IMU<IMUReading> {
public:
    static constexpr size_t OFFSET_SIZE = 22;
    static constexpr unsigned long MODE_SETTLE_MS = 20;
    BNO055IMU(uint8_t address = 0x28, TwoWire* wire = nullptr,
              IMUField fields = IMUField::Quaternion | IMUField::Gyro | IMUField::Calibration)
        : _bno(-1, address, wire), _fields(fields) {}

    bool begin() override {
        BB8_ASSERT(!_initialized, "BNO055: begin() called twice");

        if (!_bno.begin(OPERATION_MODE_NDOF)) return false;
        _bno.setExtCrystalUse(true);
        restoreCalibration();
        _initialized = true;
        return true;
    }

    IMUReading read() override {
        BB8_ASSERT(_initialized, "BNO055: read() called before begin()");
        if (!_initialized) return IMUReading{};

        IMUReading r;

        if (_fields & IMUField::Quaternion) {
            auto q = _bno.getQuat();
            r.orientation = {static_cast<float>(q.w()), static_cast<float>(q.x()),
                            static_cast<float>(q.y()), static_cast<float>(q.z())};
        }

        if (_fields & IMUField::Euler) {
            auto v = _bno.getVector(VECTOR_EULER);
            r.euler = {static_cast<float>(v.x()), static_cast<float>(v.y()),
                      static_cast<float>(v.z())};
        }

        if (_fields & IMUField::LinearAccel) {
            auto v = _bno.getVector(VECTOR_LINEARACCEL);
            r.linearAccel = {static_cast<float>(v.x()), static_cast<float>(v.y()),
                            static_cast<float>(v.z())};
        }

        if (_fields & IMUField::Gyro) {
            auto v = _bno.getVector(VECTOR_GYROSCOPE);
            r.gyro = {static_cast<float>(v.x()), static_cast<float>(v.y()),
                     static_cast<float>(v.z())};
        }

        if (_fields & IMUField::Gravity) {
            auto v = _bno.getVector(VECTOR_GRAVITY);
            r.gravity = {static_cast<float>(v.x()), static_cast<float>(v.y()),
                        static_cast<float>(v.z())};
        }

        if (_fields & IMUField::Calibration) {
            _bno.getCalibration(&r.calibration.sys, &r.calibration.gyro,
                               &r.calibration.accel, &r.calibration.mag);

            if (!_savedThisBoot && _bno.isFullyCalibrated()) {
                saveCalibration();
                _savedThisBoot = true;
            }
        }

        return r;
    }

    bool isCalibrated() {
        return _bno.isFullyCalibrated();
    }

private:
    Adafruit_BNO055 _bno;
    IMUField _fields;
    bool _initialized = false;
    bool _savedThisBoot = false;

    void restoreCalibration() {
        Preferences prefs;
        prefs.begin("bno055", true);
        uint8_t buf[OFFSET_SIZE];
        size_t len = prefs.getBytes("offsets", buf, sizeof(buf));
        prefs.end();
        if (len == OFFSET_SIZE) {
            _bno.setMode(OPERATION_MODE_CONFIG);
            delay(MODE_SETTLE_MS);
            _bno.setSensorOffsets(buf);
            _bno.setMode(OPERATION_MODE_NDOF);
            delay(MODE_SETTLE_MS);
        }
    }

    void saveCalibration() {
        BB8_ASSERT(_initialized, "BNO055: saveCalibration on uninitialized sensor");
        uint8_t buf[OFFSET_SIZE];
        _bno.getSensorOffsets(buf);
        Preferences prefs;
        prefs.begin("bno055", false);
        prefs.putBytes("offsets", buf, OFFSET_SIZE);
        prefs.end();
    }
};
