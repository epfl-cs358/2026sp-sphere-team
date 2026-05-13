/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 */

#include <unity.h>
#include "PassthroughDrivetrainController.h"
#include "BodyVelocity.h"
#include "IMUReading.h"
#include "Drivetrain.h"
#include "IMU.h"
#include "test_mock_motor.h"

class MockDrivetrain : public Drivetrain<BodyVelocity> {
public:
    MockDrivetrain() : Drivetrain<BodyVelocity>(_m0, _m1, _m2) {}

    void drive(const BodyVelocity& v) override {
        lastDrive = v;
        driveCallCount++;
    }
    void update(float /*dt*/) override {}
    void stop() override { stopCallCount++; }

    BodyVelocity lastDrive{};
    int driveCallCount = 0;
    int stopCallCount = 0;

private:
    MockMotor _m0, _m1, _m2;
};

class MockIMU : public IMU<IMUReading> {
public:
    bool begin() override { return true; }
    IMUReading read() override { return IMUReading{}; }
};

static MockDrivetrain* drivetrain = nullptr;
static MockIMU* imu = nullptr;
static PassthroughDrivetrainController* controller = nullptr;

void setUp() {
    drivetrain = new MockDrivetrain();
    imu = new MockIMU();
    controller = new PassthroughDrivetrainController(*drivetrain, *imu);
}

void tearDown() {
    delete controller;
    delete imu;
    delete drivetrain;
    controller = nullptr;
    imu = nullptr;
    drivetrain = nullptr;
}

void test_update_forwards_command_to_drivetrain() {
    BodyVelocity cmd{1.5f, -0.5f, 2.0f};
    IMUReading imuData{};
    controller->update(cmd, imuData, 0.01f);
    TEST_ASSERT_EQUAL(1, drivetrain->driveCallCount);
    TEST_ASSERT_EQUAL_FLOAT(1.5f, drivetrain->lastDrive.vx);
    TEST_ASSERT_EQUAL_FLOAT(-0.5f, drivetrain->lastDrive.vy);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, drivetrain->lastDrive.omega);
}

void test_update_ignores_dt_value() {
    BodyVelocity cmd{0.25f, 0.75f, -1.0f};
    IMUReading imuData{};

    controller->update(cmd, imuData, 0.01f);
    TEST_ASSERT_EQUAL(1, drivetrain->driveCallCount);
    TEST_ASSERT_EQUAL_FLOAT(0.25f, drivetrain->lastDrive.vx);
    TEST_ASSERT_EQUAL_FLOAT(0.75f, drivetrain->lastDrive.vy);
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, drivetrain->lastDrive.omega);

    controller->update(cmd, imuData, 1.0f);
    TEST_ASSERT_EQUAL(2, drivetrain->driveCallCount);
    TEST_ASSERT_EQUAL_FLOAT(0.25f, drivetrain->lastDrive.vx);
    TEST_ASSERT_EQUAL_FLOAT(0.75f, drivetrain->lastDrive.vy);
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, drivetrain->lastDrive.omega);

    controller->update(cmd, imuData, 0.0f);
    TEST_ASSERT_EQUAL(3, drivetrain->driveCallCount);
    TEST_ASSERT_EQUAL_FLOAT(0.25f, drivetrain->lastDrive.vx);
    TEST_ASSERT_EQUAL_FLOAT(0.75f, drivetrain->lastDrive.vy);
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, drivetrain->lastDrive.omega);
}

void test_stop_calls_drivetrain_stop() {
    controller->stop();
    TEST_ASSERT_EQUAL(1, drivetrain->stopCallCount);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_update_forwards_command_to_drivetrain);
    RUN_TEST(test_update_ignores_dt_value);
    RUN_TEST(test_stop_calls_drivetrain_stop);
    return UNITY_END();
}
