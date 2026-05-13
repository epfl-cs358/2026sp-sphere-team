/*
 * BB-8 Head Firmware — MIT 2026 SP, Team Sphere
 */

#include <unity.h>
#include "IServo.h"
#include "ServoController.h"

class MockServo : public IServo {
public:
    int attachedPin = -1;
    int lastAngle = -1;
    int writeCallCount = 0;

    void attach(int pin) override { attachedPin = pin; }
    void write(int angle) override { lastAngle = angle; writeCallCount++; }

    void reset() { attachedPin = -1; lastAngle = -1; writeCallCount = 0; }
};

static MockServo mock;
static ServoController* ctrl;

void setUp() {
    mock.reset();
    ctrl = new ServoController(mock, 1);
}

void tearDown() {
    delete ctrl;
}

void test_begin_attaches_servo() {
    ctrl->begin();
    TEST_ASSERT_EQUAL(1, mock.attachedPin);
}

void test_begin_centers_servo() {
    ctrl->begin();
    TEST_ASSERT_EQUAL(90, mock.lastAngle);
}

void test_valid_command_writes_angle() {
    ctrl->begin();
    mock.reset();
    ctrl->handleCommand("tilt:45");
    TEST_ASSERT_EQUAL(45, mock.lastAngle);
    TEST_ASSERT_EQUAL(1, mock.writeCallCount);
}

void test_negative_angle_clamped_to_zero() {
    ctrl->begin();
    mock.reset();
    ctrl->handleCommand("tilt:-5");
    TEST_ASSERT_EQUAL(0, mock.lastAngle);
}

void test_over_max_angle_clamped_to_180() {
    ctrl->begin();
    mock.reset();
    ctrl->handleCommand("tilt:270");
    TEST_ASSERT_EQUAL(180, mock.lastAngle);
}

void test_parse_error_ignored() {
    ctrl->begin();
    mock.reset();
    ctrl->handleCommand("bad_command");
    TEST_ASSERT_EQUAL(0, mock.writeCallCount);
}

void test_zero_angle_written() {
    ctrl->begin();
    mock.reset();
    ctrl->handleCommand("tilt:0");
    TEST_ASSERT_EQUAL(0, mock.lastAngle);
}

void test_max_angle_written() {
    ctrl->begin();
    mock.reset();
    ctrl->handleCommand("tilt:180");
    TEST_ASSERT_EQUAL(180, mock.lastAngle);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_begin_attaches_servo);
    RUN_TEST(test_begin_centers_servo);
    RUN_TEST(test_valid_command_writes_angle);
    RUN_TEST(test_negative_angle_clamped_to_zero);
    RUN_TEST(test_over_max_angle_clamped_to_180);
    RUN_TEST(test_parse_error_ignored);
    RUN_TEST(test_zero_angle_written);
    RUN_TEST(test_max_angle_written);
    return UNITY_END();
}
