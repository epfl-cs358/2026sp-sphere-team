#include <unity.h>
#include "Arduino.h"
#include "FIT0186Motor.h"
#include "test_mock_driver.h"

static MockDriver driver;
static FIT0186Motor<5>* motor;

void setUp() {
    driver = MockDriver();
    motor = new FIT0186Motor<5>(driver, EncoderPins{.a = 1, .b = 2}, 700);
    _set_micros(0);
    motor->begin();
}

void tearDown() {
    delete motor;
}

void test_begin_calls_driver_begin() {
    TEST_ASSERT_TRUE(driver.beginCalled);
}

void test_set_speed_delegates_to_driver() {
    motor->setSpeed(0.5f);
    TEST_ASSERT_EQUAL_FLOAT(0.5f, driver.lastOutput);
}

void test_brake_delegates_to_driver() {
    motor->brake();
    TEST_ASSERT_TRUE(driver.brakeCalled);
}

void test_rpm_zero_initially() {
    TEST_ASSERT_EQUAL_FLOAT(0.0f, motor->getRPM());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, motor->getFilteredRPM());
}

void test_rpm_calculation() {
    // Simulate 10ms elapsed, 29 encoder ticks (approx 249 RPM)
    _set_micros(10000);  // 10ms = 10000 us
    motor->_encoder._count = 29;
    motor->update();

    // expected: (29 / 700.0) * (60.0 / 0.01) = 248.57 RPM
    float expected = (29.0f / 700.0f) * (60.0f / 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, expected, motor->getRPM());
}

void test_filtered_rpm_smooths() {
    for (int i = 1; i <= 5; i++) {
        _set_micros(i * 10000);
        motor->_encoder._count = i * 29;
        motor->update();
    }
    // All updates have same RPM delta, so filtered should equal raw
    TEST_ASSERT_FLOAT_WITHIN(1.0f, motor->getRPM(), motor->getFilteredRPM());
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_begin_calls_driver_begin);
    RUN_TEST(test_set_speed_delegates_to_driver);
    RUN_TEST(test_brake_delegates_to_driver);
    RUN_TEST(test_rpm_zero_initially);
    RUN_TEST(test_rpm_calculation);
    RUN_TEST(test_filtered_rpm_smooths);
    return UNITY_END();
}
