static bool _assert_fired = false;
#define BB8_ASSERT_HANDLER(msg, file, line) _assert_fired = true

#include <fff.h>
DEFINE_FFF_GLOBALS;

#include <unity.h>
#include "Adafruit_BNO055.h"
#include "Preferences.h"
#include "IMUReading.h"
#include "IMU.h"
#include "BNO055IMU.h"

DEFINE_FAKE_VALUE_FUNC(bool, bno_begin, uint8_t);
DEFINE_FAKE_VOID_FUNC(bno_setExtCrystalUse, bool);
DEFINE_FAKE_VOID_FUNC(bno_setMode, uint8_t);
DEFINE_FAKE_VALUE_FUNC(imu::Quaternion, bno_getQuat);
DEFINE_FAKE_VALUE_FUNC(imu::Vector<3>, bno_getVector, uint8_t);
DEFINE_FAKE_VOID_FUNC(bno_getCalibration, uint8_t*, uint8_t*, uint8_t*, uint8_t*);
DEFINE_FAKE_VALUE_FUNC(bool, bno_isFullyCalibrated);
DEFINE_FAKE_VALUE_FUNC(bool, bno_getSensorOffsets, uint8_t*);
DEFINE_FAKE_VOID_FUNC(bno_setSensorOffsets, const uint8_t*);
DEFINE_FAKE_VALUE_FUNC(bool, prefs_begin, const char*, bool);
DEFINE_FAKE_VALUE_FUNC(size_t, prefs_getBytes, const char*, void*, size_t);
DEFINE_FAKE_VALUE_FUNC(size_t, prefs_putBytes, const char*, const void*, size_t);
DEFINE_FAKE_VOID_FUNC(prefs_end);

// Custom fake state
static imu::Quaternion fake_quat_return;
static imu::Vector<3> fake_vector_returns[4];
static uint8_t fake_cal_sys, fake_cal_gyro, fake_cal_accel, fake_cal_mag;

imu::Quaternion custom_getQuat() { return fake_quat_return; }

imu::Vector<3> custom_getVector(uint8_t type) {
    switch (type) {
        case VECTOR_EULER:       return fake_vector_returns[0];
        case VECTOR_LINEARACCEL: return fake_vector_returns[1];
        case VECTOR_GYROSCOPE:   return fake_vector_returns[2];
        case VECTOR_GRAVITY:     return fake_vector_returns[3];
        default:                 return imu::Vector<3>();
    }
}

void custom_getCalibration(uint8_t* s, uint8_t* g, uint8_t* a, uint8_t* m) {
    *s = fake_cal_sys;
    *g = fake_cal_gyro;
    *a = fake_cal_accel;
    *m = fake_cal_mag;
}

static BNO055IMU* imu_dev;

void resetWithFields(IMUField fields) {
    delete imu_dev;
    imu_dev = new BNO055IMU(0x28, nullptr, fields);
    RESET_BNO055_FAKES();
    RESET_PREFERENCES_FAKES();
    bno_begin_fake.return_val = true;
    imu_dev->begin();
    RESET_BNO055_FAKES();
}

void setUp() {
    RESET_BNO055_FAKES();
    RESET_PREFERENCES_FAKES();
    _assert_fired = false;
    fake_quat_return = imu::Quaternion();
    for (int i = 0; i < 4; i++) fake_vector_returns[i] = imu::Vector<3>();
    fake_cal_sys = fake_cal_gyro = fake_cal_accel = fake_cal_mag = 0;
    imu_dev = new BNO055IMU(0x28, nullptr,
        IMUField::Quaternion | IMUField::Gyro | IMUField::Calibration);
    RESET_BNO055_FAKES();
    RESET_PREFERENCES_FAKES();
}

void tearDown() {
    delete imu_dev;
}

// --- Initialization (3) ---

void test_begin_success_initializes_ndof_and_crystal() {
    bno_begin_fake.return_val = true;
    prefs_getBytes_fake.return_val = 0;
    TEST_ASSERT_TRUE(imu_dev->begin());
    TEST_ASSERT_EQUAL_UINT(1, bno_begin_fake.call_count);
    TEST_ASSERT_EQUAL_UINT8(OPERATION_MODE_NDOF, bno_begin_fake.arg0_val);
    TEST_ASSERT_EQUAL_UINT(1, bno_setExtCrystalUse_fake.call_count);
    TEST_ASSERT_TRUE(bno_setExtCrystalUse_fake.arg0_val);
    TEST_ASSERT_EQUAL_UINT(0, bno_setSensorOffsets_fake.call_count);
}

void test_begin_failure_returns_false() {
    bno_begin_fake.return_val = false;
    TEST_ASSERT_FALSE(imu_dev->begin());
    TEST_ASSERT_EQUAL_UINT(0, bno_setExtCrystalUse_fake.call_count);
}

void test_begin_restores_saved_calibration() {
    bno_begin_fake.return_val = true;
    prefs_getBytes_fake.return_val = BNO055IMU::OFFSET_SIZE;
    imu_dev->begin();
    TEST_ASSERT_EQUAL_UINT(2, bno_setMode_fake.call_count);
    TEST_ASSERT_EQUAL_UINT8(OPERATION_MODE_CONFIG, bno_setMode_fake.arg0_history[0]);
    TEST_ASSERT_EQUAL_UINT8(OPERATION_MODE_NDOF, bno_setMode_fake.arg0_history[1]);
    TEST_ASSERT_EQUAL_UINT(1, bno_setSensorOffsets_fake.call_count);
}

// --- Field Selection (4) ---

void test_read_quaternion_only() {
    resetWithFields(IMUField::Quaternion);
    imu_dev->read();
    TEST_ASSERT_EQUAL_UINT(1, bno_getQuat_fake.call_count);
    TEST_ASSERT_EQUAL_UINT(0, bno_getVector_fake.call_count);
    TEST_ASSERT_EQUAL_UINT(0, bno_getCalibration_fake.call_count);
}

void test_read_gyro_only() {
    resetWithFields(IMUField::Gyro);
    imu_dev->read();
    TEST_ASSERT_EQUAL_UINT(1, bno_getVector_fake.call_count);
    TEST_ASSERT_EQUAL_UINT8(VECTOR_GYROSCOPE, bno_getVector_fake.arg0_val);
    TEST_ASSERT_EQUAL_UINT(0, bno_getQuat_fake.call_count);
}

void test_read_all_fields() {
    resetWithFields(
        IMUField::Quaternion | IMUField::Euler | IMUField::LinearAccel |
        IMUField::Gyro | IMUField::Gravity | IMUField::Calibration);
    imu_dev->read();
    TEST_ASSERT_EQUAL_UINT(1, bno_getQuat_fake.call_count);
    TEST_ASSERT_EQUAL_UINT(4, bno_getVector_fake.call_count);
    TEST_ASSERT_EQUAL_UINT(1, bno_getCalibration_fake.call_count);
}

void test_read_euler_and_calibration_only() {
    resetWithFields(IMUField::Euler | IMUField::Calibration);
    imu_dev->read();
    TEST_ASSERT_EQUAL_UINT(1, bno_getVector_fake.call_count);
    TEST_ASSERT_EQUAL_UINT8(VECTOR_EULER, bno_getVector_fake.arg0_val);
    TEST_ASSERT_EQUAL_UINT(1, bno_getCalibration_fake.call_count);
    TEST_ASSERT_EQUAL_UINT(0, bno_getQuat_fake.call_count);
}

// --- Data Conversion (4) ---

void test_quaternion_wxyz_maps_correctly() {
    resetWithFields(IMUField::Quaternion);
    fake_quat_return = imu::Quaternion(0.707, 0.0, 0.707, 0.0);
    bno_getQuat_fake.custom_fake = custom_getQuat;
    IMUReading reading = imu_dev->read();
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.707f, reading.orientation.w);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, reading.orientation.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.707f, reading.orientation.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, reading.orientation.z);
}

void test_euler_xyz_maps_to_heading_roll_pitch() {
    resetWithFields(IMUField::Euler);
    fake_vector_returns[0] = imu::Vector<3>(270.5, -45.2, 12.8);
    bno_getVector_fake.custom_fake = custom_getVector;
    IMUReading reading = imu_dev->read();
    TEST_ASSERT_FLOAT_WITHIN(1e-1f, 270.5f, reading.euler.heading);
    TEST_ASSERT_FLOAT_WITHIN(1e-1f, -45.2f, reading.euler.roll);
    TEST_ASSERT_FLOAT_WITHIN(1e-1f, 12.8f, reading.euler.pitch);
}

void test_double_to_float_precision() {
    resetWithFields(IMUField::LinearAccel);
    fake_vector_returns[1] = imu::Vector<3>(9.80665, -0.00123, 0.00001);
    bno_getVector_fake.custom_fake = custom_getVector;
    IMUReading reading = imu_dev->read();
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 9.80665f, reading.linearAccel.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -0.00123f, reading.linearAccel.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.00001f, reading.linearAccel.z);
}

void test_gravity_maps_to_vec3() {
    resetWithFields(IMUField::Gravity);
    fake_vector_returns[3] = imu::Vector<3>(0.0, 0.0, 9.81);
    bno_getVector_fake.custom_fake = custom_getVector;
    IMUReading reading = imu_dev->read();
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, reading.gravity.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, reading.gravity.y);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 9.81f, reading.gravity.z);
}

// --- Quaternion Edge Cases (3) ---

void test_identity_quaternion_unchanged() {
    resetWithFields(IMUField::Quaternion);
    fake_quat_return = imu::Quaternion(1.0, 0.0, 0.0, 0.0);
    bno_getQuat_fake.custom_fake = custom_getQuat;
    IMUReading reading = imu_dev->read();
    TEST_ASSERT_EQUAL_FLOAT(1.0f, reading.orientation.w);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, reading.orientation.x);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, reading.orientation.y);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, reading.orientation.z);
}

void test_near_gimbal_lock_not_corrupted() {
    resetWithFields(IMUField::Quaternion);
    fake_quat_return = imu::Quaternion(0.5, 0.5, 0.5, 0.5);
    bno_getQuat_fake.custom_fake = custom_getQuat;
    IMUReading reading = imu_dev->read();
    TEST_ASSERT_EQUAL_FLOAT(0.5f, reading.orientation.w);
    TEST_ASSERT_EQUAL_FLOAT(0.5f, reading.orientation.x);
    TEST_ASSERT_EQUAL_FLOAT(0.5f, reading.orientation.y);
    TEST_ASSERT_EQUAL_FLOAT(0.5f, reading.orientation.z);
}

void test_unnormalized_quaternion_not_renormalized() {
    resetWithFields(IMUField::Quaternion);
    fake_quat_return = imu::Quaternion(2.0, 0.0, 0.0, 0.0);
    bno_getQuat_fake.custom_fake = custom_getQuat;
    IMUReading reading = imu_dev->read();
    TEST_ASSERT_EQUAL_FLOAT(2.0f, reading.orientation.w);
}

// --- Calibration Flow (4) ---

void test_calibration_pointer_order_correct() {
    bno_begin_fake.return_val = true;
    imu_dev->begin();
    RESET_BNO055_FAKES();
    fake_cal_sys = 2;
    fake_cal_gyro = 3;
    fake_cal_accel = 1;
    fake_cal_mag = 0;
    bno_getCalibration_fake.custom_fake = custom_getCalibration;
    IMUReading reading = imu_dev->read();
    TEST_ASSERT_EQUAL_UINT8(2, reading.calibration.sys);
    TEST_ASSERT_EQUAL_UINT8(3, reading.calibration.gyro);
    TEST_ASSERT_EQUAL_UINT8(1, reading.calibration.accel);
    TEST_ASSERT_EQUAL_UINT8(0, reading.calibration.mag);
}

void test_auto_save_on_first_full_calibration() {
    bno_begin_fake.return_val = true;
    imu_dev->begin();
    RESET_BNO055_FAKES();
    RESET_PREFERENCES_FAKES();

    bno_isFullyCalibrated_fake.return_val = false;
    for (int i = 0; i < 5; i++) imu_dev->read();
    TEST_ASSERT_EQUAL_UINT(0, prefs_putBytes_fake.call_count);

    bno_isFullyCalibrated_fake.return_val = true;
    imu_dev->read();
    TEST_ASSERT_EQUAL_UINT(1, prefs_putBytes_fake.call_count);

    for (int i = 0; i < 5; i++) imu_dev->read();
    TEST_ASSERT_EQUAL_UINT(1, prefs_putBytes_fake.call_count);
}

void test_no_save_when_partially_calibrated() {
    bno_begin_fake.return_val = true;
    imu_dev->begin();
    RESET_BNO055_FAKES();
    RESET_PREFERENCES_FAKES();

    bno_isFullyCalibrated_fake.return_val = false;
    for (int i = 0; i < 10; i++) imu_dev->read();
    TEST_ASSERT_EQUAL_UINT(0, prefs_putBytes_fake.call_count);
}

void test_no_restore_when_no_saved_data() {
    prefs_getBytes_fake.return_val = 0;
    bno_begin_fake.return_val = true;
    imu_dev->begin();
    TEST_ASSERT_EQUAL_UINT(0, bno_setSensorOffsets_fake.call_count);
}

// --- Sensor Boundaries (4) ---

void test_heading_at_zero_and_near_360() {
    resetWithFields(IMUField::Euler);
    fake_vector_returns[0] = imu::Vector<3>(0.0, 0.0, 0.0);
    bno_getVector_fake.custom_fake = custom_getVector;
    IMUReading reading = imu_dev->read();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, reading.euler.heading);

    fake_vector_returns[0] = imu::Vector<3>(359.99, 0.0, 0.0);
    reading = imu_dev->read();
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 359.99f, reading.euler.heading);
}

void test_pitch_at_negative_extreme() {
    resetWithFields(IMUField::Euler);
    fake_vector_returns[0] = imu::Vector<3>(0.0, 0.0, -180.0);
    bno_getVector_fake.custom_fake = custom_getVector;
    IMUReading reading = imu_dev->read();
    TEST_ASSERT_EQUAL_FLOAT(-180.0f, reading.euler.pitch);
}

void test_gyro_at_max_range() {
    resetWithFields(IMUField::Gyro);
    fake_vector_returns[2] = imu::Vector<3>(2000.0, -2000.0, 2000.0);
    bno_getVector_fake.custom_fake = custom_getVector;
    IMUReading reading = imu_dev->read();
    TEST_ASSERT_EQUAL_FLOAT(2000.0f, reading.gyro.x);
    TEST_ASSERT_EQUAL_FLOAT(-2000.0f, reading.gyro.y);
    TEST_ASSERT_EQUAL_FLOAT(2000.0f, reading.gyro.z);
}

void test_linear_accel_near_zero_not_rounded() {
    resetWithFields(IMUField::LinearAccel);
    fake_vector_returns[1] = imu::Vector<3>(0.001, 0.001, 0.001);
    bno_getVector_fake.custom_fake = custom_getVector;
    IMUReading reading = imu_dev->read();
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.001f, reading.linearAccel.x);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.001f, reading.linearAccel.y);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.001f, reading.linearAccel.z);
}

// --- Error/Defensive (5) ---

void test_partial_calibration_reported_accurately() {
    bno_begin_fake.return_val = true;
    imu_dev->begin();
    RESET_BNO055_FAKES();
    fake_cal_sys = 0;
    fake_cal_gyro = 3;
    fake_cal_accel = 2;
    fake_cal_mag = 1;
    bno_getCalibration_fake.custom_fake = custom_getCalibration;
    bno_isFullyCalibrated_fake.return_val = false;
    IMUReading reading = imu_dev->read();
    TEST_ASSERT_EQUAL_UINT8(0, reading.calibration.sys);
    TEST_ASSERT_EQUAL_UINT8(3, reading.calibration.gyro);
    TEST_ASSERT_EQUAL_UINT8(2, reading.calibration.accel);
    TEST_ASSERT_EQUAL_UINT8(1, reading.calibration.mag);
    TEST_ASSERT_FALSE(imu_dev->isCalibrated());
}

void test_isCalibrated_delegates_to_bno() {
    bno_begin_fake.return_val = true;
    imu_dev->begin();
    RESET_BNO055_FAKES();

    bno_isFullyCalibrated_fake.return_val = true;
    TEST_ASSERT_TRUE(imu_dev->isCalibrated());

    bno_isFullyCalibrated_fake.return_val = false;
    TEST_ASSERT_FALSE(imu_dev->isCalibrated());
}

void test_read_before_begin_zeroed() {
    delete imu_dev;
    imu_dev = new BNO055IMU(0x28, nullptr,
        IMUField::Quaternion | IMUField::Gyro | IMUField::Calibration);
    RESET_BNO055_FAKES();
    IMUReading reading = imu_dev->read();
    TEST_ASSERT_EQUAL_FLOAT(1.0f, reading.orientation.w);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, reading.orientation.x);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, reading.orientation.y);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, reading.orientation.z);
    TEST_ASSERT_EQUAL_UINT(0, bno_getQuat_fake.call_count);
}

void test_assert_fires_on_read_before_begin() {
    delete imu_dev;
    imu_dev = new BNO055IMU(0x28, nullptr,
        IMUField::Quaternion | IMUField::Gyro | IMUField::Calibration);
    _assert_fired = false;
    imu_dev->read();
    TEST_ASSERT_TRUE(_assert_fired);
}

void test_assert_fires_on_double_begin() {
    bno_begin_fake.return_val = true;
    imu_dev->begin();
    TEST_ASSERT_FALSE(_assert_fired);
    imu_dev->begin();
    TEST_ASSERT_TRUE(_assert_fired);
}

int main() {
    UNITY_BEGIN();

    // Initialization
    RUN_TEST(test_begin_success_initializes_ndof_and_crystal);
    RUN_TEST(test_begin_failure_returns_false);
    RUN_TEST(test_begin_restores_saved_calibration);

    // Field Selection
    RUN_TEST(test_read_quaternion_only);
    RUN_TEST(test_read_gyro_only);
    RUN_TEST(test_read_all_fields);
    RUN_TEST(test_read_euler_and_calibration_only);

    // Data Conversion
    RUN_TEST(test_quaternion_wxyz_maps_correctly);
    RUN_TEST(test_euler_xyz_maps_to_heading_roll_pitch);
    RUN_TEST(test_double_to_float_precision);
    RUN_TEST(test_gravity_maps_to_vec3);

    // Quaternion Edge Cases
    RUN_TEST(test_identity_quaternion_unchanged);
    RUN_TEST(test_near_gimbal_lock_not_corrupted);
    RUN_TEST(test_unnormalized_quaternion_not_renormalized);

    // Calibration Flow
    RUN_TEST(test_calibration_pointer_order_correct);
    RUN_TEST(test_auto_save_on_first_full_calibration);
    RUN_TEST(test_no_save_when_partially_calibrated);
    RUN_TEST(test_no_restore_when_no_saved_data);

    // Sensor Boundaries
    RUN_TEST(test_heading_at_zero_and_near_360);
    RUN_TEST(test_pitch_at_negative_extreme);
    RUN_TEST(test_gyro_at_max_range);
    RUN_TEST(test_linear_accel_near_zero_not_rounded);

    // Error/Defensive
    RUN_TEST(test_partial_calibration_reported_accurately);
    RUN_TEST(test_isCalibrated_delegates_to_bno);
    RUN_TEST(test_read_before_begin_zeroed);
    RUN_TEST(test_assert_fires_on_read_before_begin);
    RUN_TEST(test_assert_fires_on_double_begin);

    return UNITY_END();
}
