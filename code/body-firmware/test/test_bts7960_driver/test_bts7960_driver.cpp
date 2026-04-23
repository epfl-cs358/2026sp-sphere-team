static bool _assert_fired = false;
#define BB8_ASSERT_HANDLER(msg, file, line) _assert_fired = true

#include <fff.h>
DEFINE_FFF_GLOBALS;

#include <unity.h>
#include "BTS7960.h"
#include "BTS7960Driver.h"

// Include the implementation directly since test_build_src = false
#include "BTS7960Driver.cpp"

DEFINE_FAKE_VOID_FUNC(BTS7960_Enable);
DEFINE_FAKE_VOID_FUNC(BTS7960_Disable);
DEFINE_FAKE_VOID_FUNC(BTS7960_Stop);
DEFINE_FAKE_VOID_FUNC(BTS7960_TurnLeft, int8_t);
DEFINE_FAKE_VOID_FUNC(BTS7960_TurnRight, int8_t);

static BTS7960Driver* driver;

void setUp() {
    RESET_BTS7960_FAKES();
    _assert_fired = false;
    driver = new BTS7960Driver(DriverPins{.rpwm = 1, .lpwm = 2});
    RESET_BTS7960_FAKES();
}

void tearDown() {
    delete driver;
}

void test_begin_enables() {
    driver->begin();
    TEST_ASSERT_EQUAL(1, BTS7960_Enable_fake.call_count);
}

void test_full_forward() {
    driver->begin();
    RESET_BTS7960_FAKES();
    driver->setOutput(1.0f);
    TEST_ASSERT_EQUAL(1, BTS7960_Enable_fake.call_count);
    TEST_ASSERT_EQUAL(1, BTS7960_TurnLeft_fake.call_count);
    TEST_ASSERT_EQUAL_INT8(127, BTS7960_TurnLeft_fake.arg0_val);
}

void test_full_reverse() {
    driver->begin();
    RESET_BTS7960_FAKES();
    driver->setOutput(-1.0f);
    TEST_ASSERT_EQUAL(1, BTS7960_Enable_fake.call_count);
    TEST_ASSERT_EQUAL(1, BTS7960_TurnRight_fake.call_count);
    TEST_ASSERT_EQUAL_INT8(127, BTS7960_TurnRight_fake.arg0_val);
}

void test_coast() {
    driver->begin();
    RESET_BTS7960_FAKES();
    driver->setOutput(0.0f);
    TEST_ASSERT_EQUAL(1, BTS7960_Disable_fake.call_count);
    TEST_ASSERT_EQUAL(0, BTS7960_Stop_fake.call_count);
}

void test_brake() {
    driver->begin();
    driver->brake();
    TEST_ASSERT_EQUAL(1, BTS7960_Stop_fake.call_count);
}

void test_recover_after_coast() {
    driver->begin();
    driver->setOutput(0.5f);
    driver->setOutput(0.0f);  // coast — calls Disable()
    RESET_BTS7960_FAKES();
    driver->setOutput(0.5f);  // must re-enable
    TEST_ASSERT_EQUAL(1, BTS7960_Enable_fake.call_count);
    TEST_ASSERT_EQUAL(1, BTS7960_TurnLeft_fake.call_count);
}

void test_recover_after_brake() {
    driver->begin();
    driver->brake();  // calls Stop()
    RESET_BTS7960_FAKES();
    driver->setOutput(0.8f);  // must re-enable
    TEST_ASSERT_EQUAL(1, BTS7960_Enable_fake.call_count);
    TEST_ASSERT_EQUAL(1, BTS7960_TurnLeft_fake.call_count);
}

void test_direction_switch() {
    driver->begin();
    driver->setOutput(0.5f);
    RESET_BTS7960_FAKES();
    driver->setOutput(-0.5f);
    TEST_ASSERT_EQUAL(1, BTS7960_TurnRight_fake.call_count);
    TEST_ASSERT_EQUAL_INT8(63, BTS7960_TurnRight_fake.arg0_val);
}

void test_dead_zone() {
    driver->begin();
    RESET_BTS7960_FAKES();
    driver->setOutput(0.001f);
    // 0.001 * 127 = 0.127, truncated to int8_t 0
    TEST_ASSERT_EQUAL(1, BTS7960_TurnLeft_fake.call_count);
    TEST_ASSERT_EQUAL_INT8(0, BTS7960_TurnLeft_fake.arg0_val);
}

void test_assert_passes_at_boundaries() {
    driver->begin();
    driver->setOutput(1.0f);
    TEST_ASSERT_FALSE(_assert_fired);
    driver->setOutput(-1.0f);
    TEST_ASSERT_FALSE(_assert_fired);
}

void test_assert_fires_out_of_range() {
    driver->begin();
    driver->setOutput(1.001f);
    TEST_ASSERT_TRUE(_assert_fired);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_begin_enables);
    RUN_TEST(test_full_forward);
    RUN_TEST(test_full_reverse);
    RUN_TEST(test_coast);
    RUN_TEST(test_brake);
    RUN_TEST(test_recover_after_coast);
    RUN_TEST(test_recover_after_brake);
    RUN_TEST(test_direction_switch);
    RUN_TEST(test_dead_zone);
    RUN_TEST(test_assert_passes_at_boundaries);
    RUN_TEST(test_assert_fires_out_of_range);
    return UNITY_END();
}
