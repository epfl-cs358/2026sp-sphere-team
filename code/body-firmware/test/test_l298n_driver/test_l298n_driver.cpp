static bool _assert_fired = false;
#define BB8_ASSERT_HANDLER(msg, file, line) _assert_fired = true

#define ARDUINO_GPIO_FAKES

#include <fff.h>
DEFINE_FFF_GLOBALS;

#include <unity.h>
#include "Arduino.h"
#include "L298NDriver.h"

#include "L298NDriver.cpp"

DEFINE_FAKE_VOID_FUNC(pinMode, uint8_t, uint8_t);
DEFINE_FAKE_VOID_FUNC(digitalWrite, uint8_t, uint8_t);
DEFINE_FAKE_VOID_FUNC(analogWrite, uint8_t, uint8_t);

static L298NDriver* driver;
static constexpr L298NPins TEST_PINS{.fwd = 5, .rev = 6};

void setUp() {
    RESET_ARDUINO_FAKES();
    _assert_fired = false;
    driver = new L298NDriver(TEST_PINS);
    RESET_ARDUINO_FAKES();
}

void tearDown() {
    delete driver;
}

void test_begin_sets_pin_modes() {
    driver->begin();
    TEST_ASSERT_EQUAL(2, pinMode_fake.call_count);
    TEST_ASSERT_EQUAL_UINT8(5, pinMode_fake.arg0_history[0]);
    TEST_ASSERT_EQUAL_UINT8(OUTPUT, pinMode_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL_UINT8(6, pinMode_fake.arg0_history[1]);
    TEST_ASSERT_EQUAL_UINT8(OUTPUT, pinMode_fake.arg1_history[1]);
}

void test_full_forward() {
    driver->begin();
    RESET_ARDUINO_FAKES();
    driver->setOutput(1.0f);
    TEST_ASSERT_EQUAL(2, analogWrite_fake.call_count);
    TEST_ASSERT_EQUAL_UINT8(5, analogWrite_fake.arg0_history[0]);
    TEST_ASSERT_EQUAL_UINT8(255, analogWrite_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL_UINT8(6, analogWrite_fake.arg0_history[1]);
    TEST_ASSERT_EQUAL_UINT8(0, analogWrite_fake.arg1_history[1]);
}

void test_full_reverse() {
    driver->begin();
    RESET_ARDUINO_FAKES();
    driver->setOutput(-1.0f);
    TEST_ASSERT_EQUAL(2, analogWrite_fake.call_count);
    TEST_ASSERT_EQUAL_UINT8(5, analogWrite_fake.arg0_history[0]);
    TEST_ASSERT_EQUAL_UINT8(0, analogWrite_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL_UINT8(6, analogWrite_fake.arg0_history[1]);
    TEST_ASSERT_EQUAL_UINT8(255, analogWrite_fake.arg1_history[1]);
}

void test_half_forward() {
    driver->begin();
    RESET_ARDUINO_FAKES();
    driver->setOutput(0.5f);
    TEST_ASSERT_EQUAL(2, analogWrite_fake.call_count);
    TEST_ASSERT_EQUAL_UINT8(5, analogWrite_fake.arg0_history[0]);
    TEST_ASSERT_EQUAL_UINT8(127, analogWrite_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL_UINT8(6, analogWrite_fake.arg0_history[1]);
    TEST_ASSERT_EQUAL_UINT8(0, analogWrite_fake.arg1_history[1]);
}

void test_half_reverse() {
    driver->begin();
    RESET_ARDUINO_FAKES();
    driver->setOutput(-0.5f);
    TEST_ASSERT_EQUAL(2, analogWrite_fake.call_count);
    TEST_ASSERT_EQUAL_UINT8(5, analogWrite_fake.arg0_history[0]);
    TEST_ASSERT_EQUAL_UINT8(0, analogWrite_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL_UINT8(6, analogWrite_fake.arg0_history[1]);
    TEST_ASSERT_EQUAL_UINT8(127, analogWrite_fake.arg1_history[1]);
}

void test_coast() {
    driver->begin();
    RESET_ARDUINO_FAKES();
    driver->setOutput(0.0f);
    TEST_ASSERT_EQUAL(2, analogWrite_fake.call_count);
    TEST_ASSERT_EQUAL_UINT8(5, analogWrite_fake.arg0_history[0]);
    TEST_ASSERT_EQUAL_UINT8(0, analogWrite_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL_UINT8(6, analogWrite_fake.arg0_history[1]);
    TEST_ASSERT_EQUAL_UINT8(0, analogWrite_fake.arg1_history[1]);
}

void test_brake() {
    driver->begin();
    RESET_ARDUINO_FAKES();
    driver->brake();
    TEST_ASSERT_EQUAL(2, analogWrite_fake.call_count);
    TEST_ASSERT_EQUAL_UINT8(5, analogWrite_fake.arg0_history[0]);
    TEST_ASSERT_EQUAL_UINT8(255, analogWrite_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL_UINT8(6, analogWrite_fake.arg0_history[1]);
    TEST_ASSERT_EQUAL_UINT8(255, analogWrite_fake.arg1_history[1]);
}

void test_direction_switch() {
    driver->begin();
    driver->setOutput(0.5f);
    RESET_ARDUINO_FAKES();
    driver->setOutput(-0.5f);
    TEST_ASSERT_EQUAL_UINT8(0, analogWrite_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL_UINT8(127, analogWrite_fake.arg1_history[1]);
}

void test_dead_zone() {
    driver->begin();
    RESET_ARDUINO_FAKES();
    driver->setOutput(0.001f);
    TEST_ASSERT_EQUAL_UINT8(0, analogWrite_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL_UINT8(0, analogWrite_fake.arg1_history[1]);
}

void test_recover_after_coast() {
    driver->begin();
    driver->setOutput(0.5f);
    driver->setOutput(0.0f);
    RESET_ARDUINO_FAKES();
    driver->setOutput(0.5f);
    TEST_ASSERT_EQUAL_UINT8(127, analogWrite_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL_UINT8(0, analogWrite_fake.arg1_history[1]);
}

void test_recover_after_brake() {
    driver->begin();
    driver->brake();
    RESET_ARDUINO_FAKES();
    driver->setOutput(0.8f);
    uint8_t expected = static_cast<uint8_t>(0.8f * 255.0f);
    TEST_ASSERT_EQUAL_UINT8(expected, analogWrite_fake.arg1_history[0]);
    TEST_ASSERT_EQUAL_UINT8(0, analogWrite_fake.arg1_history[1]);
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
    RUN_TEST(test_begin_sets_pin_modes);
    RUN_TEST(test_full_forward);
    RUN_TEST(test_full_reverse);
    RUN_TEST(test_half_forward);
    RUN_TEST(test_half_reverse);
    RUN_TEST(test_coast);
    RUN_TEST(test_brake);
    RUN_TEST(test_direction_switch);
    RUN_TEST(test_dead_zone);
    RUN_TEST(test_recover_after_coast);
    RUN_TEST(test_recover_after_brake);
    RUN_TEST(test_assert_passes_at_boundaries);
    RUN_TEST(test_assert_fires_out_of_range);
    return UNITY_END();
}
