/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 */

#include <type_traits>
#include <unity.h>
#include "BalanceConfig.h"
#include "RobotConstants.h"

static_assert(std::is_trivially_copyable<BalanceConfig>::value,
              "BalanceConfig must remain trivially-copyable (POD)");

void setUp() {}
void tearDown() {}

void test_default_config_values_match_spec() {
    BalanceConfig cfg = RobotConstants::balanceConfig();

    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.30f, cfg.tiltPerVelocity);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.35f, cfg.maxTiltSetpoint);

    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.50f, cfg.pitchKp);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f,  cfg.pitchKi);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.15f, cfg.pitchKd);

    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.50f, cfg.rollKp);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f,  cfg.rollKi);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.15f, cfg.rollKd);

    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.00f, cfg.maxOutputVelocity);

    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.866f, cfg.envelopeEnterSin);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.819f, cfg.envelopeExitSin);

    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, cfg.gyroPitchSign);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.0f, cfg.gyroRollSign);
}

void test_config_is_trivially_copyable() {
    // Backed by the TU-scope static_assert above; this runtime test exists so
    // the assertion shows up in the Unity report as an explicit checkpoint.
    TEST_ASSERT_TRUE(std::is_trivially_copyable<BalanceConfig>::value);
}

void test_envelope_invariant_in_defaults() {
    BalanceConfig cfg = RobotConstants::balanceConfig();
    TEST_ASSERT_TRUE(cfg.envelopeExitSin < cfg.envelopeEnterSin);
}

int main() {
    UNITY_BEGIN();

    RUN_TEST(test_default_config_values_match_spec);
    RUN_TEST(test_config_is_trivially_copyable);
    RUN_TEST(test_envelope_invariant_in_defaults);

    return UNITY_END();
}
