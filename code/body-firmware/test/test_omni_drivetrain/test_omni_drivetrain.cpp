#include <unity.h>
#include <cmath>
#include "test_mock_motor.h"
#include "OmniDrivetrain.h"
#include "OmniKinematics.h"
#include "PID.h"
#include "DrivetrainConfig.h"
#include "BodyVelocity.h"

static constexpr float TOL = 0.01f;

static MockMotor m0, m1, m2;
static PID pid0(1.0f, 0.0f, 0.0f, -1.0f, 1.0f);
static PID pid1(1.0f, 0.0f, 0.0f, -1.0f, 1.0f);
static PID pid2(1.0f, 0.0f, 0.0f, -1.0f, 1.0f);

static DrivetrainConfig testConfig() {
    return {.wheelRadius = 0.05f, .robotRadius = 0.1f, .tiltAngle = 0.0f, .maxRPM = 300.0f};
}

void setUp() {
    m0.resetMock();
    m1.resetMock();
    m2.resetMock();
    pid0.reset();
    pid1.reset();
    pid2.reset();
}

void tearDown() {}

void test_drive_stores_target_rpms() {
    OmniDrivetrain dt(m0, m1, m2, testConfig(), pid0, pid1, pid2);

    BodyVelocity v = {1.0f, 0.0f, 0.0f};
    dt.drive(v);

    // drive() alone should not call setSpeed — that's update()'s job
    TEST_ASSERT_EQUAL(0, m0.setSpeedCallCount);
    TEST_ASSERT_EQUAL(0, m1.setSpeedCallCount);
    TEST_ASSERT_EQUAL(0, m2.setSpeedCallCount);
}

void test_update_calls_motor_update() {
    OmniDrivetrain dt(m0, m1, m2, testConfig(), pid0, pid1, pid2);
    dt.drive({0.0f, 0.0f, 0.0f});
    dt.update(0.01f);

    TEST_ASSERT_EQUAL(1, m0.updateCallCount);
    TEST_ASSERT_EQUAL(1, m1.updateCallCount);
    TEST_ASSERT_EQUAL(1, m2.updateCallCount);
}

void test_update_sets_motor_speeds() {
    OmniDrivetrain dt(m0, m1, m2, testConfig(), pid0, pid1, pid2);

    BodyVelocity v = {1.0f, 0.0f, 0.0f};
    dt.drive(v);
    dt.update(0.01f);

    TEST_ASSERT_EQUAL(1, m0.setSpeedCallCount);
    TEST_ASSERT_EQUAL(1, m1.setSpeedCallCount);
    TEST_ASSERT_EQUAL(1, m2.setSpeedCallCount);
}

void test_update_pid_output_for_forward() {
    OmniDrivetrain dt(m0, m1, m2, testConfig(), pid0, pid1, pid2);

    m0.filteredRpmToReturn = 0.0f;
    m1.filteredRpmToReturn = 0.0f;
    m2.filteredRpmToReturn = 0.0f;

    BodyVelocity v = {1.0f, 0.0f, 0.0f};
    dt.drive(v);
    dt.update(0.01f);

    // Motor 0 target RPM ≈ 0, so PID error ≈ 0, speed ≈ 0
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, m0.lastSpeed);
    // Motors 1 and 2 have non-zero targets, PID should produce non-zero output
    TEST_ASSERT_TRUE(m1.lastSpeed != 0.0f);
    TEST_ASSERT_TRUE(m2.lastSpeed != 0.0f);
    // Motor 1 and 2 should be opposite sign
    TEST_ASSERT_TRUE(m1.lastSpeed * m2.lastSpeed < 0.0f);
}

void test_stop_brakes_all_motors() {
    OmniDrivetrain dt(m0, m1, m2, testConfig(), pid0, pid1, pid2);
    dt.drive({1.0f, 0.0f, 0.0f});
    dt.stop();

    TEST_ASSERT_TRUE(m0.brakeCalled);
    TEST_ASSERT_TRUE(m1.brakeCalled);
    TEST_ASSERT_TRUE(m2.brakeCalled);
}

void test_stop_resets_pid() {
    OmniDrivetrain dt(m0, m1, m2, testConfig(), pid0, pid1, pid2);

    // Drive and update to accumulate PID state
    dt.drive({1.0f, 0.0f, 0.0f});
    dt.update(0.01f);

    dt.stop();

    // After stop + new drive + update, PID should start fresh
    m0.resetMock();
    m1.resetMock();
    m2.resetMock();

    dt.drive({1.0f, 0.0f, 0.0f});
    dt.update(0.01f);

    // With P-only PID (kp=1), first call after reset: output = error * kp
    // This verifies the PID was actually reset (no leftover integral/derivative)
    OmniKinematics kin(testConfig());
    auto rpms = kin.toWheelRPMs({1.0f, 0.0f, 0.0f});
    // PID output = kp * (targetRPM - actualRPM) = 1.0 * (targetRPM - 0) = targetRPM
    // But clamped to [-1, 1]
    float expected1 = rpms[1];
    if (expected1 > 1.0f) expected1 = 1.0f;
    if (expected1 < -1.0f) expected1 = -1.0f;
    TEST_ASSERT_FLOAT_WITHIN(TOL, expected1, m1.lastSpeed);
}

void test_zero_velocity_zero_speed() {
    OmniDrivetrain dt(m0, m1, m2, testConfig(), pid0, pid1, pid2);

    dt.drive({0.0f, 0.0f, 0.0f});
    dt.update(0.01f);

    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, m0.lastSpeed);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, m1.lastSpeed);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, m2.lastSpeed);
}

static constexpr float PLANT_GAIN = 50.0f;

static void setupPlantMotors(MockMotor& a, MockMotor& b, MockMotor& c) {
    a.plantMode = true; a.plantGain = PLANT_GAIN;
    b.plantMode = true; b.plantGain = PLANT_GAIN;
    c.plantMode = true; c.plantGain = PLANT_GAIN;
}

void test_pid_convergence_with_feedback() {
    MockMotor fm0, fm1, fm2;
    setupPlantMotors(fm0, fm1, fm2);

    PID fp0(0.01f, 0.01f, 0.0f, -1.0f, 1.0f);
    PID fp1(0.01f, 0.01f, 0.0f, -1.0f, 1.0f);
    PID fp2(0.01f, 0.01f, 0.0f, -1.0f, 1.0f);

    OmniDrivetrain dt(fm0, fm1, fm2, testConfig(), fp0, fp1, fp2);
    dt.drive({0.0f, 0.1f, 0.0f});

    for (int i = 0; i < 500; i++) {
        dt.update(0.01f);
    }

    OmniKinematics kin(testConfig());
    auto targetRPMs = kin.toWheelRPMs({0.0f, 0.1f, 0.0f});

    MockMotor* motors[] = {&fm0, &fm1, &fm2};
    for (int i = 0; i < 3; i++) {
        float expectedSpeed = targetRPMs[i] / PLANT_GAIN;
        if (expectedSpeed > 1.0f) expectedSpeed = 1.0f;
        if (expectedSpeed < -1.0f) expectedSpeed = -1.0f;
        TEST_ASSERT_FLOAT_WITHIN(0.05f, expectedSpeed, motors[i]->lastSpeed);
    }
}

void test_stop_then_update_motors_stay_zero() {
    MockMotor fm0, fm1, fm2;
    setupPlantMotors(fm0, fm1, fm2);

    PID fp0(0.01f, 0.01f, 0.0f, -1.0f, 1.0f);
    PID fp1(0.01f, 0.01f, 0.0f, -1.0f, 1.0f);
    PID fp2(0.01f, 0.01f, 0.0f, -1.0f, 1.0f);

    OmniDrivetrain dt(fm0, fm1, fm2, testConfig(), fp0, fp1, fp2);

    dt.drive({0.0f, 0.1f, 0.0f});
    for (int i = 0; i < 200; i++) dt.update(0.01f);

    dt.stop();

    for (int i = 0; i < 200; i++) dt.update(0.01f);

    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, fm0.lastSpeed);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, fm1.lastSpeed);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, fm2.lastSpeed);
}

void test_rapid_direction_reversal() {
    MockMotor fm0, fm1, fm2;
    setupPlantMotors(fm0, fm1, fm2);

    PID fp0(0.01f, 0.01f, 0.0f, -1.0f, 1.0f);
    PID fp1(0.01f, 0.01f, 0.0f, -1.0f, 1.0f);
    PID fp2(0.01f, 0.01f, 0.0f, -1.0f, 1.0f);

    OmniDrivetrain dt(fm0, fm1, fm2, testConfig(), fp0, fp1, fp2);

    dt.drive({0.0f, 0.1f, 0.0f});
    for (int i = 0; i < 500; i++) dt.update(0.01f);
    float m1_forward = fm1.lastSpeed;

    dt.drive({0.0f, -0.1f, 0.0f});
    for (int i = 0; i < 500; i++) dt.update(0.01f);
    float m1_backward = fm1.lastSpeed;

    TEST_ASSERT_TRUE(m1_forward != 0.0f);
    TEST_ASSERT_TRUE(m1_backward != 0.0f);
    TEST_ASSERT_TRUE(m1_forward * m1_backward < 0.0f);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_drive_stores_target_rpms);
    RUN_TEST(test_update_calls_motor_update);
    RUN_TEST(test_update_sets_motor_speeds);
    RUN_TEST(test_update_pid_output_for_forward);
    RUN_TEST(test_stop_brakes_all_motors);
    RUN_TEST(test_stop_resets_pid);
    RUN_TEST(test_zero_velocity_zero_speed);
    RUN_TEST(test_pid_convergence_with_feedback);
    RUN_TEST(test_stop_then_update_motors_stay_zero);
    RUN_TEST(test_rapid_direction_reversal);
    return UNITY_END();
}
