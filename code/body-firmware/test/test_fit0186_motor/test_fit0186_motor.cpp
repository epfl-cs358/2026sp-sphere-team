static bool _assert_fired = false;
#define BB8_ASSERT_HANDLER(msg, file, line) _assert_fired = true

#include <unity.h>
#include <cmath>
#include <climits>
#include "Arduino.h"
#include "FIT0186Motor.h"
#include "test_mock_driver.h"

static MockDriver driver;
static FIT0186Motor<5>* motor;

void setUp() {
    driver = MockDriver();
    _assert_fired = false;
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
    // Simulate 10ms elapsed, 29 encoder ticks
    _set_micros(10000);  // 10ms = 10000 us
    motor->_encoder._count = 29;
    motor->update();

    // expected: (29 / 700.0) * (60.0 / 0.01) / 43.8 = output-shaft RPM
    float expected = (29.0f / 700.0f) * (60.0f / 0.01f) / fit0186::GEAR_RATIO;
    TEST_ASSERT_FLOAT_WITHIN(0.1f, expected, motor->getRPM());
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

void test_reverse_direction() {
    _set_micros(10000);
    motor->_encoder._count = -29;
    motor->update();
    float expected = (-29.0f / 700.0f) * (60.0f / 0.01f) / fit0186::GEAR_RATIO;
    TEST_ASSERT_FLOAT_WITHIN(0.1f, expected, motor->getRPM());
    TEST_ASSERT_TRUE(motor->getRPM() < 0.0f);
}

void test_micros_overflow() {
    // begin near ULONG_MAX, update after wraparound
    delete motor;
    driver = MockDriver();
    motor = new FIT0186Motor<5>(driver, EncoderPins{.a = 1, .b = 2}, 700);
    _set_micros(ULONG_MAX - 5000);
    motor->begin();

    // 10ms later, micros wraps around
    _set_micros(ULONG_MAX - 5000 + 10000);  // wraps via unsigned overflow
    motor->_encoder._count = 29;
    motor->update();

    float expected = (29.0f / 700.0f) * (60.0f / 0.01f) / fit0186::GEAR_RATIO;
    TEST_ASSERT_FLOAT_WITHIN(0.1f, expected, motor->getRPM());
}

void test_same_micros_noop() {
    // update() with dt=0 should not change RPM
    motor->_encoder._count = 100;
    _set_micros(0);  // same as begin()
    motor->update();
    TEST_ASSERT_EQUAL_FLOAT(0.0f, motor->getRPM());
}

void test_skipped_update_preserves_ticks() {
    // dt=0 skip, then real update — ticks from skip are counted
    motor->_encoder._count = 100;
    _set_micros(0);
    motor->update();  // skipped (dt=0)

    _set_micros(10000);  // 10ms
    motor->update();  // should see full 100 tick delta
    float expected = (100.0f / 700.0f) * (60.0f / 0.01f) / fit0186::GEAR_RATIO;
    TEST_ASSERT_FLOAT_WITHIN(0.1f, expected, motor->getRPM());
}

void test_zero_cpr_asserts() {
    delete motor;
    driver = MockDriver();
    _assert_fired = false;
    motor = new FIT0186Motor<5>(driver, EncoderPins{.a = 1, .b = 2}, 0);
    TEST_ASSERT_TRUE(_assert_fired);
    // Recreate valid motor for tearDown
    motor = new FIT0186Motor<5>(driver, EncoderPins{.a = 1, .b = 2}, 700);
}

void test_filter_convergence_on_reversal() {
    // 5 forward updates at ~248 RPM
    for (int i = 1; i <= 5; i++) {
        _set_micros(i * 10000);
        motor->_encoder._count = i * 29;
        motor->update();
    }
    float forwardRPM = motor->getFilteredRPM();
    TEST_ASSERT_TRUE(forwardRPM > 4.0f);

    // 5 reverse updates (encoder goes backwards)
    for (int i = 6; i <= 10; i++) {
        _set_micros(i * 10000);
        motor->_encoder._count = 5 * 29 - (i - 5) * 29;  // decreasing
        motor->update();
    }
    // After 5 reverse samples, filter should fully converge to negative
    TEST_ASSERT_TRUE(motor->getFilteredRPM() < -4.0f);
}

void test_split_intervals_consistent() {
    // 350 ticks in 500ms
    _set_micros(500000);
    motor->_encoder._count = 350;
    motor->update();
    float rpm1 = motor->getRPM();

    // another 350 ticks in 500ms
    _set_micros(1000000);
    motor->_encoder._count = 700;
    motor->update();
    float rpm2 = motor->getRPM();

    // Output-shaft: (350/700) * (60/0.5) / 43.8 ≈ 1.37 RPM
    float expected = 60.0f / fit0186::GEAR_RATIO;
    TEST_ASSERT_FLOAT_WITHIN(0.01f, expected, rpm1);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, expected, rpm2);
}

void test_high_rpm() {
    // 700 ticks in 100 microseconds = 600,000 RPM
    _set_micros(100);
    motor->_encoder._count = 700;
    motor->update();
    float expected = (700.0f / 700.0f) * (60.0f / 0.0001f) / fit0186::GEAR_RATIO;
    TEST_ASSERT_FLOAT_WITHIN(10.0f, expected, motor->getRPM());
    TEST_ASSERT_TRUE(std::isfinite(motor->getRPM()));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_begin_calls_driver_begin);
    RUN_TEST(test_set_speed_delegates_to_driver);
    RUN_TEST(test_brake_delegates_to_driver);
    RUN_TEST(test_rpm_zero_initially);
    RUN_TEST(test_rpm_calculation);
    RUN_TEST(test_filtered_rpm_smooths);
    RUN_TEST(test_reverse_direction);
    RUN_TEST(test_micros_overflow);
    RUN_TEST(test_same_micros_noop);
    RUN_TEST(test_skipped_update_preserves_ticks);
    RUN_TEST(test_zero_cpr_asserts);
    RUN_TEST(test_filter_convergence_on_reversal);
    RUN_TEST(test_split_intervals_consistent);
    RUN_TEST(test_high_rpm);
    return UNITY_END();
}
