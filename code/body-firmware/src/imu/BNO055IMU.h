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
#include "RemoteSerial.h"

class BNO055IMU : public IMU<IMUReading> {
public:
    static constexpr size_t OFFSET_SIZE = 22;
    static constexpr unsigned long MODE_SETTLE_MS = 20;
    BNO055IMU(uint8_t address = 0x28, TwoWire* wire = nullptr,
              IMUField fields = IMUField::Quaternion | IMUField::Gyro | IMUField::Calibration)
        : _bno(-1, address, wire), _fields(fields) {}

    bool begin() override {
        BB8_ASSERT(!_initialized, "BNO055: begin() called twice");

        // IMUPLUS: gyro + accel fusion only. No magnetometer — the 3 nearby
        // motors corrupt mag readings, and a balancing robot needs gravity
        // vector + angular rate, not absolute heading. Adafruit lib's
        // isFullyCalibrated() in IMUPLUS checks (accel == 3 && gyro == 3)
        // and ignores mag/sys.
        if (!_bno.begin(OPERATION_MODE_IMUPLUS)) return false;
        _bno.setExtCrystalUse(true);
        _restored = restoreCalibration();
        _initialized = true;
        return true;
    }

    IMUReading read() override {
        BB8_ASSERT(_initialized, "BNO055: read() called before begin()");
        if (!_initialized) return IMUReading{};

        IMUReading r;

        if (_fields & IMUField::Quaternion) {
            auto q = _bno.getQuat();
            r.orientation = Quat{static_cast<float>(q.w()), static_cast<float>(q.x()),
                                 static_cast<float>(q.y()), static_cast<float>(q.z())};
        }

        if (_fields & IMUField::Euler) {
            auto v = _bno.getVector(Adafruit_BNO055::VECTOR_EULER);
            r.euler = Euler{static_cast<float>(v.x()), static_cast<float>(v.y()),
                            static_cast<float>(v.z())};
        }

        if (_fields & IMUField::LinearAccel) {
            auto v = _bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
            r.linearAccel = Vec3{static_cast<float>(v.x()), static_cast<float>(v.y()),
                                 static_cast<float>(v.z())};
        }

        if (_fields & IMUField::Gyro) {
            auto v = _bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
            r.gyro = Vec3{static_cast<float>(v.x()), static_cast<float>(v.y()),
                          static_cast<float>(v.z())};
        }

        if (_fields & IMUField::Gravity) {
            auto v = _bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
            r.gravity = Vec3{static_cast<float>(v.x()), static_cast<float>(v.y()),
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

    // True iff begin() found valid offsets in NVS and pushed them to the
    // chip. Lets the calibration-wait loop fast-path: chip's cal counters
    // lag the offsets by ~10–20s after a restore even though it's already
    // functionally calibrated, so a warm-boot wait should only gate on the
    // gyro hold-still rather than the full isCalibrated() check.
    bool wasRestored() const { return _restored; }

private:
    Adafruit_BNO055 _bno;
    IMUField _fields;
    bool _initialized = false;
    bool _savedThisBoot = false;
    bool _restored = false;

    // Returns true iff a valid 22-byte offsets blob was found in NVS and
    // pushed to the chip. Diagnostic log goes to RemoteSerial so the
    // operator sees in WebSerial whether the warm-boot path is in effect.
    bool restoreCalibration() {
        Preferences prefs;
        prefs.begin("bno055", true);
        uint8_t buf[OFFSET_SIZE];
        size_t len = prefs.getBytes("offsets", buf, sizeof(buf));
        prefs.end();
        if (len == OFFSET_SIZE) {
            _bno.setMode(OPERATION_MODE_CONFIG);
            delay(MODE_SETTLE_MS);
            _bno.setSensorOffsets(buf);
            _bno.setMode(OPERATION_MODE_IMUPLUS);
            delay(MODE_SETTLE_MS);
            RemoteSerial::println("[imu] offsets restored from NVS");
            return true;
        }
        RemoteSerial::println("[imu] no saved offsets; cold calibration required");
        return false;
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
