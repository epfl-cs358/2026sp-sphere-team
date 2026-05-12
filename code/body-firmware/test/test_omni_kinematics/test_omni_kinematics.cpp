#include <unity.h>
#include <cmath>
#include "OmniKinematics.h"
#include "DrivetrainConfig.h"
#include "BodyVelocity.h"

static const float TOLERANCE = 0.01f;
static const float SQRT3_2 = 0.86602540378f;

static DrivetrainConfig defaultConfig() {
    constexpr float DEG = static_cast<float>(M_PI) / 180.0f;
    return {
        .wheelRadius = 0.05f,
        .robotRadius = 0.1f,
        .tiltAngle = 0.0f,
        .maxRPM = 100.0f,
        .wheelAngles = {0.0f, 120.0f * DEG, 240.0f * DEG},
    };
}

static DrivetrainConfig pullConfig() {
    constexpr float DEG = static_cast<float>(M_PI) / 180.0f;
    return {
        .wheelRadius = 0.05f,
        .robotRadius = 0.1f,
        .tiltAngle = 0.0f,
        .maxRPM = 100.0f,
        .wheelAngles = {180.0f * DEG, 300.0f * DEG, 60.0f * DEG},
    };
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

void test_pure_strafe_left() {
    DrivetrainConfig cfg = defaultConfig();
    cfg.maxRPM = 300.0f;
    OmniKinematics kin(cfg);
    // vy=+1 = strafe left
    auto rpms = kin.toWheelRPMs({0.0f, 1.0f, 0.0f});

    float rpm0 = toRPM(-20.0f);
    float rpm12 = toRPM(10.0f);

    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, rpm0, rpms[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, rpm12, rpms[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, rpm12, rpms[2]);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, fabsf(rpms[0]) * 0.5f, fabsf(rpms[1]));
}

void test_pure_ccw_rotation() {
    OmniKinematics kin(defaultConfig());
    // omega=+1 = CCW from above
    auto rpms = kin.toWheelRPMs({0.0f, 0.0f, 1.0f});

    float expectedRPM = toRPM(-2.0f);

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

    float unsaturated0 = toRPM(-20.0f);
    float unsaturated1 = toRPM(10.0f);
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

void test_pure_backward() {
    DrivetrainConfig cfg = defaultConfig();
    cfg.maxRPM = 300.0f;
    OmniKinematics kin(cfg);

    auto fwd = kin.toWheelRPMs({1.0f, 0.0f, 0.0f});
    auto bwd = kin.toWheelRPMs({-1.0f, 0.0f, 0.0f});

    for (int i = 0; i < 3; i++) {
        TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, -fwd[i], bwd[i]);
    }
}

void test_pure_strafe_right() {
    DrivetrainConfig cfg = defaultConfig();
    cfg.maxRPM = 300.0f;
    OmniKinematics kin(cfg);

    auto left = kin.toWheelRPMs({0.0f, 1.0f, 0.0f});
    auto right = kin.toWheelRPMs({0.0f, -1.0f, 0.0f});

    for (int i = 0; i < 3; i++) {
        TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, -left[i], right[i]);
    }
}

void test_pure_cw_rotation() {
    OmniKinematics kin(defaultConfig());
    auto ccw = kin.toWheelRPMs({0.0f, 0.0f, 1.0f});
    auto cw = kin.toWheelRPMs({0.0f, 0.0f, -1.0f});

    for (int i = 0; i < 3; i++) {
        TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, -ccw[i], cw[i]);
    }
}

void test_combined_3_axis() {
    DrivetrainConfig cfg = defaultConfig();
    cfg.maxRPM = 1000.0f;
    OmniKinematics kin(cfg);

    auto vx = kin.toWheelRPMs({0.1f, 0.0f, 0.0f});
    auto vy = kin.toWheelRPMs({0.0f, 0.1f, 0.0f});
    auto vw = kin.toWheelRPMs({0.0f, 0.0f, 1.0f});
    auto combined = kin.toWheelRPMs({0.1f, 0.1f, 1.0f});

    for (int i = 0; i < 3; i++) {
        TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, vx[i] + vy[i] + vw[i], combined[i]);
    }
}

// Pull config: motor 0 at 180° (back), motor 1 at 300° (front-right), motor 2 at 60° (front-left).
// Under pure forward vx > 0, motor 0 should idle (back), motors 1 & 2 do the work.

void test_pull_pure_forward() {
    DrivetrainConfig cfg = pullConfig();
    cfg.maxRPM = 300.0f;
    OmniKinematics kin(cfg);
    auto rpms = kin.toWheelRPMs({1.0f, 0.0f, 0.0f});

    // motor 0 at θ=180°: -sin(180°)=0, idle for pure vx
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, 0.0f, rpms[0]);
    // motor 1 at θ=300°: -sin(300°)=+√3/2 → positive RPM
    // motor 2 at θ= 60°: -sin( 60°)=-√3/2 → negative RPM, equal magnitude
    float expectedRPM1 = toRPM(20.0f * SQRT3_2);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, expectedRPM1, rpms[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, -expectedRPM1, rpms[2]);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, fabsf(rpms[1]), fabsf(rpms[2]));
}

void test_pull_pure_strafe_left() {
    DrivetrainConfig cfg = pullConfig();
    cfg.maxRPM = 300.0f;
    OmniKinematics kin(cfg);
    auto rpms = kin.toWheelRPMs({0.0f, 1.0f, 0.0f});

    // motor 0 at θ=180°: -cos(180°)·1 = +1 → +20 rad/s
    // motors 1 & 2 at θ=300°,60°: -cos = -0.5 → -10 rad/s each
    float rpm0 = toRPM(20.0f);
    float rpm12 = toRPM(-10.0f);

    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, rpm0, rpms[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, rpm12, rpms[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, rpm12, rpms[2]);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, fabsf(rpms[0]) * 0.5f, fabsf(rpms[1]));
}

void test_pull_pure_ccw_rotation() {
    OmniKinematics kin(pullConfig());
    auto rpms = kin.toWheelRPMs({0.0f, 0.0f, 1.0f});

    // Rotation term -R·ω is azimuth-independent: all three same magnitude and same sign.
    float expectedRPM = toRPM(-2.0f);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, expectedRPM, rpms[0]);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, expectedRPM, rpms[1]);
    TEST_ASSERT_FLOAT_WITHIN(TOLERANCE, expectedRPM, rpms[2]);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_zero_velocity);
    RUN_TEST(test_pure_forward);
    RUN_TEST(test_pure_strafe_left);
    RUN_TEST(test_pure_ccw_rotation);
    RUN_TEST(test_saturation_scaling);
    RUN_TEST(test_tilt_angle_effect);
    RUN_TEST(test_combined_velocity);
    RUN_TEST(test_pure_backward);
    RUN_TEST(test_pure_strafe_right);
    RUN_TEST(test_pure_cw_rotation);
    RUN_TEST(test_combined_3_axis);
    RUN_TEST(test_pull_pure_forward);
    RUN_TEST(test_pull_pure_strafe_left);
    RUN_TEST(test_pull_pure_ccw_rotation);
    return UNITY_END();
}
