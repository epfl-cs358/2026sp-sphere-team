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

// ---- pre-arm gyro-quiet gate tests (Commit 4) ----------------------------

// PREARM_GYRO_QUIET_DPS = 1.5 dps ≈ 0.02618 rad/s. If any sample in the last
// 500 ms (50 samples @ 100Hz) exceeds the threshold, arm() must refuse and
// fire kEvent_PREARM_REJECTED.

void test_prearm_refuses_when_gyro_is_loud() {
    // Push 50 loud samples (0.05 rad/s ≈ 2.9 dps, well above threshold).
    for (int i = 0; i < 50; ++i) {
        ArmingState::recordGyroZ(0.05f);
    }
    // Clear any prior rejection latch.
    (void)ArmingState::consumePrearmRejected();

    ArmingState::arm();

    TEST_ASSERT_EQUAL(static_cast<int>(State::Disarmed),
                      static_cast<int>(ArmingState::get()));
    TEST_ASSERT_TRUE(ArmingState::consumePrearmRejected());
}

void test_prearm_succeeds_when_gyro_is_quiet() {
    for (int i = 0; i < 50; ++i) {
        ArmingState::recordGyroZ(0.01f);  // ~0.57 dps, well below threshold
    }
    (void)ArmingState::consumePrearmRejected();

    ArmingState::arm();

    TEST_ASSERT_EQUAL(static_cast<int>(State::Armed),
                      static_cast<int>(ArmingState::get()));
    TEST_ASSERT_FALSE(ArmingState::consumePrearmRejected());
}

void test_prearm_old_loud_samples_age_out() {
    // Push 50 loud samples first, then 50 quiet samples — the ring buffer
    // is 50 deep, so the quiet samples have fully evicted the loud ones.
    for (int i = 0; i < 50; ++i) {
        ArmingState::recordGyroZ(0.05f);
    }
    for (int i = 0; i < 50; ++i) {
        ArmingState::recordGyroZ(0.01f);
    }
    (void)ArmingState::consumePrearmRejected();

    ArmingState::arm();

    TEST_ASSERT_EQUAL(static_cast<int>(State::Armed),
                      static_cast<int>(ArmingState::get()));
}

void test_prearm_buffer_records_negative_values_by_magnitude() {
    // Negative rates above magnitude threshold must also reject.
    for (int i = 0; i < 50; ++i) {
        ArmingState::recordGyroZ(-0.05f);
    }
    (void)ArmingState::consumePrearmRejected();

    ArmingState::arm();

    TEST_ASSERT_EQUAL(static_cast<int>(State::Disarmed),
                      static_cast<int>(ArmingState::get()));
    TEST_ASSERT_TRUE(ArmingState::consumePrearmRejected());
}

void test_prearm_rejection_consumed_once() {
    for (int i = 0; i < 50; ++i) {
        ArmingState::recordGyroZ(0.05f);
    }
    (void)ArmingState::consumePrearmRejected();

    ArmingState::arm();
    TEST_ASSERT_TRUE(ArmingState::consumePrearmRejected());
    // Second consume must return false — single-shot semantics.
    TEST_ASSERT_FALSE(ArmingState::consumePrearmRejected());
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
    RUN_TEST(test_prearm_refuses_when_gyro_is_loud);
    RUN_TEST(test_prearm_succeeds_when_gyro_is_quiet);
    RUN_TEST(test_prearm_old_loud_samples_age_out);
    RUN_TEST(test_prearm_buffer_records_negative_values_by_magnitude);
    RUN_TEST(test_prearm_rejection_consumed_once);

    return UNITY_END();
}
