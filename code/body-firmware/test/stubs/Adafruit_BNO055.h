#pragma once
#include <cstdint>
#include <utility/imumaths.h>
#include <fff.h>
#include "Wire.h"

typedef enum {
    OPERATION_MODE_CONFIG = 0x00,
    OPERATION_MODE_NDOF   = 0x0C
} adafruit_bno055_opmode_t;

typedef enum {
    VECTOR_ACCELEROMETER = 0x08,
    VECTOR_MAGNETOMETER  = 0x0E,
    VECTOR_GYROSCOPE     = 0x14,
    VECTOR_EULER         = 0x1A,
    VECTOR_LINEARACCEL   = 0x28,
    VECTOR_GRAVITY       = 0x2E
} adafruit_vector_type_t;

typedef struct {
    int16_t accel_offset_x, accel_offset_y, accel_offset_z;
    int16_t mag_offset_x, mag_offset_y, mag_offset_z;
    int16_t gyro_offset_x, gyro_offset_y, gyro_offset_z;
    int16_t accel_radius;
    int16_t mag_radius;
} adafruit_bno055_offsets_t;

DECLARE_FAKE_VALUE_FUNC(bool, bno_begin, uint8_t);
DECLARE_FAKE_VOID_FUNC(bno_setExtCrystalUse, bool);
DECLARE_FAKE_VOID_FUNC(bno_setMode, uint8_t);
DECLARE_FAKE_VALUE_FUNC(imu::Quaternion, bno_getQuat);
DECLARE_FAKE_VALUE_FUNC(imu::Vector<3>, bno_getVector, uint8_t);
DECLARE_FAKE_VOID_FUNC(bno_getCalibration, uint8_t*, uint8_t*, uint8_t*, uint8_t*);
DECLARE_FAKE_VALUE_FUNC(bool, bno_isFullyCalibrated);
DECLARE_FAKE_VALUE_FUNC(bool, bno_getSensorOffsets, uint8_t*);
DECLARE_FAKE_VOID_FUNC(bno_setSensorOffsets, const uint8_t*);

class Adafruit_BNO055 {
public:
    Adafruit_BNO055(int32_t sensorID = -1, uint8_t address = 0x28, TwoWire* theWire = nullptr) {}
    bool begin(uint8_t mode = OPERATION_MODE_NDOF) { return bno_begin(mode); }
    void setExtCrystalUse(bool usextal) { bno_setExtCrystalUse(usextal); }
    void setMode(adafruit_bno055_opmode_t mode) { bno_setMode(static_cast<uint8_t>(mode)); }
    imu::Quaternion getQuat() { return bno_getQuat(); }
    imu::Vector<3> getVector(adafruit_vector_type_t type) { return bno_getVector(static_cast<uint8_t>(type)); }
    void getCalibration(uint8_t* sys, uint8_t* gyro, uint8_t* accel, uint8_t* mag) { bno_getCalibration(sys, gyro, accel, mag); }
    bool isFullyCalibrated() { return bno_isFullyCalibrated(); }
    bool getSensorOffsets(uint8_t* buf) { return bno_getSensorOffsets(buf); }
    void setSensorOffsets(const uint8_t* buf) { bno_setSensorOffsets(buf); }
};

#define RESET_BNO055_FAKES() do { \
    RESET_FAKE(bno_begin); \
    RESET_FAKE(bno_setExtCrystalUse); \
    RESET_FAKE(bno_setMode); \
    RESET_FAKE(bno_getQuat); \
    RESET_FAKE(bno_getVector); \
    RESET_FAKE(bno_getCalibration); \
    RESET_FAKE(bno_isFullyCalibrated); \
    RESET_FAKE(bno_getSensorOffsets); \
    RESET_FAKE(bno_setSensorOffsets); \
    FFF_RESET_HISTORY(); \
} while(0)
