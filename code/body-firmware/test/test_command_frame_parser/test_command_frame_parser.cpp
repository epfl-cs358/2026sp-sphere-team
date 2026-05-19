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
    TEST_ASSERT_EQUAL_FLOAT(1.0f, r.velocity.vx);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, r.velocity.vy);
    TEST_ASSERT_EQUAL_FLOAT(3.0f, r.velocity.omega);
}

void test_negative_values() {
    auto r = parse("-1.5,-0.5,-2.0");
    TEST_ASSERT_TRUE(r.ok());
    TEST_ASSERT_EQUAL_FLOAT(-1.5f, r.velocity.vx);
    TEST_ASSERT_EQUAL_FLOAT(-0.5f, r.velocity.vy);
    TEST_ASSERT_EQUAL_FLOAT(-2.0f, r.velocity.omega);
}

void test_zero_values() {
    auto r = parse("0,0,0");
    TEST_ASSERT_TRUE(r.ok());
    TEST_ASSERT_EQUAL_FLOAT(0.0f, r.velocity.vx);
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
    TEST_ASSERT_EQUAL_FLOAT(3.0f, r.velocity.omega);
}

void test_trailing_newline_accepted() {
    auto r = parse("1.0,2.0,3.0\n");
    TEST_ASSERT_TRUE(r.ok());
    TEST_ASSERT_EQUAL_FLOAT(3.0f, r.velocity.omega);
}

void test_legacy_velocity_unprefixed_still_parses() {
    auto r = parse("1.0,2.0,3.0");
    TEST_ASSERT_TRUE(r.ok());
    TEST_ASSERT_EQUAL(static_cast<int>(FrameKind::Velocity),
                      static_cast<int>(r.kind));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, r.velocity.vx);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, r.velocity.vy);
    TEST_ASSERT_EQUAL_FLOAT(3.0f, r.velocity.omega);
    TEST_ASSERT_EQUAL(static_cast<int>(FrameParseError::None),
                      static_cast<int>(r.error));
}

void test_velocity_with_v_sigil_parses() {
    auto r = parse("v1.0,2.0,3.0");
    TEST_ASSERT_TRUE(r.ok());
    TEST_ASSERT_EQUAL(static_cast<int>(FrameKind::Velocity),
                      static_cast<int>(r.kind));
    TEST_ASSERT_EQUAL_FLOAT(1.0f, r.velocity.vx);
    TEST_ASSERT_EQUAL_FLOAT(2.0f, r.velocity.vy);
    TEST_ASSERT_EQUAL_FLOAT(3.0f, r.velocity.omega);
}

void test_control_arm() {
    auto r = parse("c:arm");
    TEST_ASSERT_TRUE(r.ok());
    TEST_ASSERT_EQUAL(static_cast<int>(FrameKind::Control),
                      static_cast<int>(r.kind));
    TEST_ASSERT_EQUAL(static_cast<int>(ControlVerb::Arm),
                      static_cast<int>(r.control));
}

void test_control_disarm() {
    auto r = parse("c:disarm");
    TEST_ASSERT_TRUE(r.ok());
    TEST_ASSERT_EQUAL(static_cast<int>(FrameKind::Control),
                      static_cast<int>(r.kind));
    TEST_ASSERT_EQUAL(static_cast<int>(ControlVerb::Disarm),
                      static_cast<int>(r.control));
}

void test_control_kill() {
    auto r = parse("c:kill");
    TEST_ASSERT_TRUE(r.ok());
    TEST_ASSERT_EQUAL(static_cast<int>(FrameKind::Control),
                      static_cast<int>(r.kind));
    TEST_ASSERT_EQUAL(static_cast<int>(ControlVerb::Kill),
                      static_cast<int>(r.control));
}

void test_control_clearkill() {
    auto r = parse("c:clearkill");
    TEST_ASSERT_TRUE(r.ok());
    TEST_ASSERT_EQUAL(static_cast<int>(FrameKind::Control),
                      static_cast<int>(r.kind));
    TEST_ASSERT_EQUAL(static_cast<int>(ControlVerb::ClearKill),
                      static_cast<int>(r.control));
}

void test_command_frame_parser_recognizes_armstate_query() {
    auto r = parse("c:armstate?");
    TEST_ASSERT_TRUE(r.ok());
    TEST_ASSERT_EQUAL(static_cast<int>(FrameKind::Control),
                      static_cast<int>(r.kind));
    TEST_ASSERT_EQUAL(static_cast<int>(ControlVerb::QueryArmState),
                      static_cast<int>(r.control));
}

void test_control_unknown_verb() {
    auto r = parse("c:explode");
    TEST_ASSERT_FALSE(r.ok());
    TEST_ASSERT_EQUAL(static_cast<int>(FrameParseError::InvalidControlVerb),
                      static_cast<int>(r.error));
}

void test_control_missing_colon() {
    auto r = parse("carm");
    TEST_ASSERT_FALSE(r.ok());
    TEST_ASSERT_EQUAL(static_cast<int>(FrameParseError::InvalidControlVerb),
                      static_cast<int>(r.error));
}

void test_negative_leading_minus_is_velocity() {
    auto r = parse("-1.0,0,0");
    TEST_ASSERT_TRUE(r.ok());
    TEST_ASSERT_EQUAL(static_cast<int>(FrameKind::Velocity),
                      static_cast<int>(r.kind));
    TEST_ASSERT_EQUAL_FLOAT(-1.0f, r.velocity.vx);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, r.velocity.vy);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, r.velocity.omega);
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
    RUN_TEST(test_legacy_velocity_unprefixed_still_parses);
    RUN_TEST(test_velocity_with_v_sigil_parses);
    RUN_TEST(test_control_arm);
    RUN_TEST(test_control_disarm);
    RUN_TEST(test_control_kill);
    RUN_TEST(test_control_clearkill);
    RUN_TEST(test_command_frame_parser_recognizes_armstate_query);
    RUN_TEST(test_control_unknown_verb);
    RUN_TEST(test_control_missing_colon);
    RUN_TEST(test_negative_leading_minus_is_velocity);
    return UNITY_END();
}
