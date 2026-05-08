/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 */

#include <unity.h>
#include <cstring>

#include "CommandFrameParser.h"

void setUp() {}
void tearDown() {}

static FrameParseResult parse(const char* msg) {
    return parseCommandFrame(reinterpret_cast<const uint8_t*>(msg), std::strlen(msg));
}

void test_happy_path() {
    auto r = parse("1.0,2.0,3.0");
    TEST_ASSERT_TRUE(r.ok());
    TEST_ASSERT_EQUAL_FLOAT(1.0f, r.value.vx);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, r.value.vy);
    TEST_ASSERT_EQUAL_FLOAT(3.0f, r.value.omega);
}

void test_negative_values() {
    auto r = parse("-1.5,-0.5,-2.0");
    TEST_ASSERT_TRUE(r.ok());
    TEST_ASSERT_EQUAL_FLOAT(-1.5f, r.value.vx);
    TEST_ASSERT_EQUAL_FLOAT(-0.5f, r.value.vy);
    TEST_ASSERT_EQUAL_FLOAT(-2.0f, r.value.omega);
}

void test_zero_values() {
    auto r = parse("0,0,0");
    TEST_ASSERT_TRUE(r.ok());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, r.value.vx);
}

void test_empty_payload_is_error() {
    auto r = parseCommandFrame(reinterpret_cast<const uint8_t*>(""), 0);
    TEST_ASSERT_FALSE(r.ok());
    TEST_ASSERT_EQUAL(static_cast<int>(FrameParseError::EmptyOrTooLong),
                      static_cast<int>(r.error));
}

void test_too_long_is_error() {
    uint8_t big[64];
    for (auto& b : big) b = '1';
    auto r = parseCommandFrame(big, sizeof(big));
    TEST_ASSERT_FALSE(r.ok());
    TEST_ASSERT_EQUAL(static_cast<int>(FrameParseError::EmptyOrTooLong),
                      static_cast<int>(r.error));
}

void test_missing_first_comma_is_error() {
    auto r = parse("1.0 2.0,3.0");
    TEST_ASSERT_FALSE(r.ok());
    TEST_ASSERT_EQUAL(static_cast<int>(FrameParseError::InvalidVx),
                      static_cast<int>(r.error));
}

void test_non_numeric_vy_is_error() {
    auto r = parse("1.0,abc,3.0");
    TEST_ASSERT_FALSE(r.ok());
    TEST_ASSERT_EQUAL(static_cast<int>(FrameParseError::InvalidVy),
                      static_cast<int>(r.error));
}

void test_missing_omega_is_error() {
    auto r = parse("1.0,2.0,");
    TEST_ASSERT_FALSE(r.ok());
    TEST_ASSERT_EQUAL(static_cast<int>(FrameParseError::InvalidOmega),
                      static_cast<int>(r.error));
}

void test_trailing_extra_field_accepted() {
    auto r = parse("1.0,2.0,3.0,extra,more");
    TEST_ASSERT_TRUE(r.ok());
    TEST_ASSERT_EQUAL_FLOAT(3.0f, r.value.omega);
}

void test_trailing_newline_accepted() {
    auto r = parse("1.0,2.0,3.0\n");
    TEST_ASSERT_TRUE(r.ok());
    TEST_ASSERT_EQUAL_FLOAT(3.0f, r.value.omega);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_happy_path);
    RUN_TEST(test_negative_values);
    RUN_TEST(test_zero_values);
    RUN_TEST(test_empty_payload_is_error);
    RUN_TEST(test_too_long_is_error);
    RUN_TEST(test_missing_first_comma_is_error);
    RUN_TEST(test_non_numeric_vy_is_error);
    RUN_TEST(test_missing_omega_is_error);
    RUN_TEST(test_trailing_extra_field_accepted);
    RUN_TEST(test_trailing_newline_accepted);
    return UNITY_END();
}
