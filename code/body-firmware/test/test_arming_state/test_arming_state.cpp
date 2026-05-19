#include <unity.h>
#include <atomic>
#include <thread>
#include <chrono>

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

// TOCTOU: a concurrent arm() must NEVER overwrite a kill() that landed
// between arm()'s load and store. With load-then-store this flakes; with
// compare_exchange against the expected source state it cannot.
void test_arm_does_not_overwrite_concurrent_kill() {
    // Hammer arm() across many iterations to maximize the chance a slow
    // load-then-store implementation loses to the kill() store.
    for (int iter = 0; iter < 200; ++iter) {
        ArmingState::begin();  // back to Disarmed each iteration

        std::atomic<bool> go{false};
        std::atomic<bool> done{false};

        std::thread armer([&] {
            while (!go.load(std::memory_order_acquire)) { /* spin */ }
            for (int i = 0; i < 1000 && !done.load(std::memory_order_acquire); ++i) {
                ArmingState::arm();
            }
        });

        go.store(true, std::memory_order_release);
        // Let the armer get going so kill() lands mid-stream.
        std::this_thread::sleep_for(std::chrono::microseconds(50));
        ArmingState::kill();
        done.store(true, std::memory_order_release);
        armer.join();

        // After both finish, kill must have won (latched). arm() must not have
        // overwritten Killed.
        TEST_ASSERT_EQUAL(static_cast<int>(State::Killed),
                          static_cast<int>(ArmingState::get()));
    }
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
    RUN_TEST(test_arm_does_not_overwrite_concurrent_kill);

    return UNITY_END();
}
