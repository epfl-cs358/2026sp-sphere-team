#include <fff.h>
DEFINE_FFF_GLOBALS;

#include <unity.h>
#include <Arduino.h>
#include "OnboardLED.h"

DEFINE_FAKE_VOID_FUNC(pinMode, uint8_t, uint8_t);
DEFINE_FAKE_VOID_FUNC(digitalWrite, uint8_t, uint8_t);

static constexpr uint8_t TEST_PIN = 21;
static OnboardLED* led;

void setUp() {
    RESET_ARDUINO_FAKES();
    FFF_RESET_HISTORY();
    led = new OnboardLED(TEST_PIN);
}

void tearDown() {
    delete led;
}

// --- begin() ---

void test_begin_sets_pin_mode_output() {
    led->begin();
    TEST_ASSERT_EQUAL(1, pinMode_fake.call_count);
    TEST_ASSERT_EQUAL(TEST_PIN, pinMode_fake.arg0_val);
    TEST_ASSERT_EQUAL(OUTPUT, pinMode_fake.arg1_val);
}

// --- Initial state ---

void test_initial_state_is_error_led_off() {
    led->begin();
    led->update(0);
    TEST_ASSERT_EQUAL(LOW, digitalWrite_fake.arg1_val);
}

// --- Error state ---

void test_error_state_led_off() {
    led->begin();
    led->setState(LEDState::Error);
    led->update(0);
    TEST_ASSERT_EQUAL(LOW, digitalWrite_fake.arg1_val);
}

// --- Ready state ---

void test_ready_state_led_on() {
    led->begin();
    led->setState(LEDState::Ready);
    led->update(0);
    TEST_ASSERT_EQUAL(HIGH, digitalWrite_fake.arg1_val);
}

// --- Streaming state ---

void test_streaming_state_led_on() {
    led->begin();
    led->setState(LEDState::Streaming);
    led->update(0);
    TEST_ASSERT_EQUAL(HIGH, digitalWrite_fake.arg1_val);
}

// --- Connecting state (blink) ---

void test_connecting_state_starts_on() {
    led->begin();
    led->setState(LEDState::Connecting);
    led->update(0);
    TEST_ASSERT_EQUAL(HIGH, digitalWrite_fake.arg1_val);
}

void test_connecting_state_toggles_after_250ms() {
    led->begin();
    led->setState(LEDState::Connecting);

    led->update(0);
    TEST_ASSERT_EQUAL(HIGH, digitalWrite_fake.arg1_val);

    led->update(250);
    TEST_ASSERT_EQUAL(LOW, digitalWrite_fake.arg1_val);
}

void test_connecting_state_toggles_back_after_500ms() {
    led->begin();
    led->setState(LEDState::Connecting);

    led->update(0);
    led->update(250);
    led->update(500);
    TEST_ASSERT_EQUAL(HIGH, digitalWrite_fake.arg1_val);
}

void test_connecting_no_toggle_before_interval() {
    led->begin();
    led->setState(LEDState::Connecting);

    led->update(0);
    uint8_t first_val = digitalWrite_fake.arg1_val;

    led->update(100);
    TEST_ASSERT_EQUAL(first_val, digitalWrite_fake.arg1_val);
}

// --- State transitions ---

void test_transition_from_connecting_to_ready_stops_blinking() {
    led->begin();
    led->setState(LEDState::Connecting);
    led->update(0);

    led->setState(LEDState::Ready);
    led->update(100);
    TEST_ASSERT_EQUAL(HIGH, digitalWrite_fake.arg1_val);

    led->update(350);
    TEST_ASSERT_EQUAL(HIGH, digitalWrite_fake.arg1_val);
}

int main() {
    UNITY_BEGIN();

    RUN_TEST(test_begin_sets_pin_mode_output);
    RUN_TEST(test_initial_state_is_error_led_off);
    RUN_TEST(test_error_state_led_off);
    RUN_TEST(test_ready_state_led_on);
    RUN_TEST(test_streaming_state_led_on);
    RUN_TEST(test_connecting_state_starts_on);
    RUN_TEST(test_connecting_state_toggles_after_250ms);
    RUN_TEST(test_connecting_state_toggles_back_after_500ms);
    RUN_TEST(test_connecting_no_toggle_before_interval);
    RUN_TEST(test_transition_from_connecting_to_ready_stops_blinking);

    return UNITY_END();
}
