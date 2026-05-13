/*
 * BB-8 Head Firmware — MIT 2026 SP, Team Sphere
 */

#include <unity.h>
#include "CommandParser.h"

void setUp() {}
void tearDown() {}

void test_valid_mid_angle() {
    TEST_ASSERT_EQUAL(90, parseServoCommand("tilt:90"));
}

void test_valid_zero() {
    TEST_ASSERT_EQUAL(0, parseServoCommand("tilt:0"));
}

void test_valid_max() {
    TEST_ASSERT_EQUAL(180, parseServoCommand("tilt:180"));
}

void test_negative_angle_returned_raw() {
    TEST_ASSERT_EQUAL(-5, parseServoCommand("tilt:-5"));
}

void test_non_numeric_value_is_error() {
    TEST_ASSERT_EQUAL(-1, parseServoCommand("tilt:abc"));
}

void test_empty_string_is_error() {
    TEST_ASSERT_EQUAL(-1, parseServoCommand(""));
}

void test_wrong_prefix_is_error() {
    TEST_ASSERT_EQUAL(-1, parseServoCommand("pan:90"));
}

void test_no_colon_is_error() {
    TEST_ASSERT_EQUAL(-1, parseServoCommand("tilt90"));
}

void test_only_prefix_is_error() {
    TEST_ASSERT_EQUAL(-1, parseServoCommand("tilt:"));
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_valid_mid_angle);
    RUN_TEST(test_valid_zero);
    RUN_TEST(test_valid_max);
    RUN_TEST(test_negative_angle_returned_raw);
    RUN_TEST(test_non_numeric_value_is_error);
    RUN_TEST(test_empty_string_is_error);
    RUN_TEST(test_wrong_prefix_is_error);
    RUN_TEST(test_no_colon_is_error);
    RUN_TEST(test_only_prefix_is_error);
    return UNITY_END();
}
