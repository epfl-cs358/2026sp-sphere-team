#include <unity.h>
#include "ArmingState.h"
#include "ArmingState.cpp"

using ArmingState::State;

void setUp() {
    ArmingState::begin();
}

void tearDown() {}

void test_boot_state_is_disarmed() {
    TEST_ASSERT_EQUAL(static_cast<int>(State::Disarmed),
                      static_cast<int>(ArmingState::get()));
    TEST_ASSERT_FALSE(ArmingState::isArmed());
}

void test_disarmed_to_armed() {
    ArmingState::arm();
    TEST_ASSERT_EQUAL(static_cast<int>(State::Armed),
                      static_cast<int>(ArmingState::get()));
    TEST_ASSERT_TRUE(ArmingState::isArmed());
}

void test_armed_to_disarmed() {
    ArmingState::arm();
    ArmingState::disarm();
    TEST_ASSERT_EQUAL(static_cast<int>(State::Disarmed),
                      static_cast<int>(ArmingState::get()));
}

void test_any_to_killed() {
    ArmingState::arm();
    ArmingState::kill();
    TEST_ASSERT_EQUAL(static_cast<int>(State::Killed),
                      static_cast<int>(ArmingState::get()));

    ArmingState::begin();
    ArmingState::kill();
    TEST_ASSERT_EQUAL(static_cast<int>(State::Killed),
                      static_cast<int>(ArmingState::get()));
}

void test_arm_while_killed_is_noop() {
    ArmingState::kill();
    ArmingState::arm();
    TEST_ASSERT_EQUAL(static_cast<int>(State::Killed),
                      static_cast<int>(ArmingState::get()));
}

void test_clearkill_from_non_killed_is_noop() {
    ArmingState::clearKill();
    TEST_ASSERT_EQUAL(static_cast<int>(State::Disarmed),
                      static_cast<int>(ArmingState::get()));

    ArmingState::arm();
    ArmingState::clearKill();
    TEST_ASSERT_EQUAL(static_cast<int>(State::Armed),
                      static_cast<int>(ArmingState::get()));
}

void test_clearkill_from_killed_to_disarmed() {
    ArmingState::kill();
    ArmingState::clearKill();
    TEST_ASSERT_EQUAL(static_cast<int>(State::Disarmed),
                      static_cast<int>(ArmingState::get()));
}

int main() {
    UNITY_BEGIN();

    RUN_TEST(test_boot_state_is_disarmed);
    RUN_TEST(test_disarmed_to_armed);
    RUN_TEST(test_armed_to_disarmed);
    RUN_TEST(test_any_to_killed);
    RUN_TEST(test_arm_while_killed_is_noop);
    RUN_TEST(test_clearkill_from_non_killed_is_noop);
    RUN_TEST(test_clearkill_from_killed_to_disarmed);

    return UNITY_END();
}
