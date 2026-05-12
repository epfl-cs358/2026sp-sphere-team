#include <unity.h>
#include "OtaPolicy.h"

using OtaPolicy::shouldMarkValid;
using OtaPolicy::shouldRebootForOffline;

void setUp() {}
void tearDown() {}

// shouldMarkValid: returns true exactly when the firmware has been running
// long enough AND hasn't already been marked valid.

void test_mark_valid_not_yet() {
    TEST_ASSERT_FALSE(shouldMarkValid(1000, 0, false, 30000));
}

void test_mark_valid_exactly_at_delay() {
    TEST_ASSERT_TRUE(shouldMarkValid(30000, 0, false, 30000));
}

void test_mark_valid_after_delay() {
    TEST_ASSERT_TRUE(shouldMarkValid(30001, 0, false, 30000));
}

void test_mark_valid_already_marked_noop() {
    TEST_ASSERT_FALSE(shouldMarkValid(60000, 0, true, 30000));
}

void test_mark_valid_with_nonzero_boot() {
    // boot_ms=10000, now=39999, delay=30000 → elapsed=29999 → not yet.
    TEST_ASSERT_FALSE(shouldMarkValid(39999, 10000, false, 30000));
    // ...one ms later → cross threshold.
    TEST_ASSERT_TRUE(shouldMarkValid(40000, 10000, false, 30000));
}

// shouldRebootForOffline: the loop reboots ONLY when truly disconnected
// (no AP, no STA) for longer than the threshold. AP or STA presence vetoes.

void test_reboot_blocked_by_ap() {
    TEST_ASSERT_FALSE(shouldRebootForOffline(true, false, 60000, 0, 30000));
}

void test_reboot_blocked_by_sta() {
    TEST_ASSERT_FALSE(shouldRebootForOffline(false, true, 60000, 0, 30000));
}

void test_reboot_blocked_by_both() {
    TEST_ASSERT_FALSE(shouldRebootForOffline(true, true, 60000, 0, 30000));
}

void test_reboot_when_truly_offline() {
    TEST_ASSERT_TRUE(shouldRebootForOffline(false, false, 60000, 0, 30000));
}

void test_reboot_below_threshold_holds() {
    TEST_ASSERT_FALSE(shouldRebootForOffline(false, false, 20000, 0, 30000));
}

void test_reboot_at_threshold_holds() {
    // Strictly greater than threshold required to reboot.
    TEST_ASSERT_FALSE(shouldRebootForOffline(false, false, 30000, 0, 30000));
}

void test_reboot_just_past_threshold_fires() {
    TEST_ASSERT_TRUE(shouldRebootForOffline(false, false, 30001, 0, 30000));
}

// millis() wraps every ~49.7 days. Unsigned subtraction handles this
// correctly by language guarantee, but pin the contract so a well-meaning
// "fix" to make the subtraction signed can't silently break recovery.

void test_mark_valid_handles_millis_wraparound() {
    // boot=UINT32_MAX-100, now=10 → elapsed = 111 ms (wrapped). Not enough.
    TEST_ASSERT_FALSE(shouldMarkValid(10, UINT32_MAX - 100, false, 30000));
    // boot=UINT32_MAX-100, now=30000 → elapsed = 30101 ms. Should fire.
    TEST_ASSERT_TRUE(shouldMarkValid(30000, UINT32_MAX - 100, false, 30000));
}

void test_reboot_handles_millis_wraparound() {
    // last=UINT32_MAX-100, now=10 → elapsed = 111 ms. Below 30s threshold,
    // so we must NOT reboot just because the clock wrapped.
    TEST_ASSERT_FALSE(shouldRebootForOffline(false, false, 10,
                                             UINT32_MAX - 100, 30000));
}

int main() {
    UNITY_BEGIN();

    RUN_TEST(test_mark_valid_not_yet);
    RUN_TEST(test_mark_valid_exactly_at_delay);
    RUN_TEST(test_mark_valid_after_delay);
    RUN_TEST(test_mark_valid_already_marked_noop);
    RUN_TEST(test_mark_valid_with_nonzero_boot);

    RUN_TEST(test_reboot_blocked_by_ap);
    RUN_TEST(test_reboot_blocked_by_sta);
    RUN_TEST(test_reboot_blocked_by_both);
    RUN_TEST(test_reboot_when_truly_offline);
    RUN_TEST(test_reboot_below_threshold_holds);
    RUN_TEST(test_reboot_at_threshold_holds);
    RUN_TEST(test_reboot_just_past_threshold_fires);

    RUN_TEST(test_mark_valid_handles_millis_wraparound);
    RUN_TEST(test_reboot_handles_millis_wraparound);

    return UNITY_END();
}
