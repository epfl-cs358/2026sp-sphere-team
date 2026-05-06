#include <unity.h>
#include <cmath>
#include "OmniKinematics.h"
#include "DrivetrainConfig.h"
#include "BodyVelocity.h"

static const float TOLERANCE = 0.01f;
static const float SQRT3_2 = 0.86602540378f;

static DrivetrainConfig defaultConfig() {
    return {.wheelRadius = 0.05f, .robotRadius = 0.1f, .tiltAngle = 0.0f, .maxRPM = 100.0f};
}

static float toRPM(float omega) {
    return omega * 60.0f / (2.0f * static_cast<float>(M_PI));
}

void setUp() {}
void tearDown() {}

void test_zero_velocity() {
    OmniKinematics kin(defaultConfig());
    auto rpms = kin.toWheelRPMs({0.0f, 0.0f, 0.0f});
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 0.0f, rpms[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 0.0f, rpms[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 0.0f, rpms[2]);
}

void test_pure_forward() {
    DrivetrainConfig cfg = defaultConfig();
    cfg.maxRPM = 300.0f;
    OmniKinematics kin(cfg);
    auto rpms = kin.toWheelRPMs({1.0f, 0.0f, 0.0f});

    float expectedRPM1 = toRPM(-20.0f * SQRT3_2);
    float expectedRPM2 = -expectedRPM1;

    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 0.0f, rpms[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, expectedRPM1, rpms[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, expectedRPM2, rpms[2]);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, fabsf(rpms[1]), fabsf(rpms[2]));
}

void test_pure_strafe_right() {
    DrivetrainConfig cfg = defaultConfig();
    cfg.maxRPM = 300.0f;
    OmniKinematics kin(cfg);
    auto rpms = kin.toWheelRPMs({0.0f, 1.0f, 0.0f});

    float rpm0 = toRPM(20.0f);
    float rpm12 = toRPM(-10.0f);

    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, rpm0, rpms[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, rpm12, rpms[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, rpm12, rpms[2]);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, fabsf(rpms[0]) * 0.5f, fabsf(rpms[1]));
}

void test_pure_cw_rotation() {
    OmniKinematics kin(defaultConfig());
    auto rpms = kin.toWheelRPMs({0.0f, 0.0f, 1.0f});

    float expectedRPM = toRPM(2.0f);

    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, expectedRPM, rpms[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, expectedRPM, rpms[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, expectedRPM, rpms[2]);
}

void test_saturation_scaling() {
    OmniKinematics kin(defaultConfig());

    auto rpms = kin.toWheelRPMs({0.0f, 1.0f, 0.0f});

    float maxAbs = 0.0f;
    for (int i = 0; i < 3; i++) {
        if (fabsf(rpms[i]) > maxAbs) maxAbs = fabsf(rpms[i]);
    }
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 100.0f, maxAbs);

    float unsaturated0 = toRPM(20.0f);
    float unsaturated1 = toRPM(-10.0f);
    float expectedRatio = unsaturated1 / unsaturated0;
    float actualRatio = rpms[1] / rpms[0];
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, expectedRatio, actualRatio);
}

void test_tilt_angle_effect() {
    DrivetrainConfig flat = defaultConfig();
    DrivetrainConfig tilted = defaultConfig();
    tilted.tiltAngle = 30.0f * static_cast<float>(M_PI) / 180.0f;

    OmniKinematics kinFlat(flat);
    OmniKinematics kinTilted(tilted);

    BodyVelocity v = {0.1f, 0.0f, 0.0f};

    auto rpmsFlat = kinFlat.toWheelRPMs(v);
    auto rpmsTilted = kinTilted.toWheelRPMs(v);

    float factor = 1.0f / cosf(tilted.tiltAngle);

    for (int i = 0; i < 3; i++) {
        if (fabsf(rpmsFlat[i]) > 0.01f) {
            TEST_ASSERT_FLOAT_WITHIN(0.1f, rpmsFlat[i] * factor, rpmsTilted[i]);
        }
    }
}

void test_combined_velocity() {
    OmniKinematics kin(defaultConfig());

    BodyVelocity forward = {0.1f, 0.0f, 0.0f};
    BodyVelocity rotation = {0.0f, 0.0f, 1.0f};
    BodyVelocity combined = {0.1f, 0.0f, 1.0f};

    auto rpmsForward = kin.toWheelRPMs(forward);
    auto rpmsRotation = kin.toWheelRPMs(rotation);
    auto rpmsCombined = kin.toWheelRPMs(combined);

    for (int i = 0; i < 3; i++) {
        TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, rpmsForward[i] + rpmsRotation[i], rpmsCombined[i]);
    }
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_zero_velocity);
    RUN_TEST(test_pure_forward);
    RUN_TEST(test_pure_strafe_right);
    RUN_TEST(test_pure_cw_rotation);
    RUN_TEST(test_saturation_scaling);
    RUN_TEST(test_tilt_angle_effect);
    RUN_TEST(test_combined_velocity);
    return UNITY_END();
}
