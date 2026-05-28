#include <unity.h>
#include <cmath>
#include "quatToBodyGravity.h"

static constexpr float TOL = 1e-4f;

void setUp() {}
void tearDown() {}

// Hamilton product: r = a * b (scalar-first).
static Quat quatMul(const Quat& a, const Quat& b) {
    Quat r;
    r.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
    r.x = a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y;
    r.y = a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x;
    r.z = a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w;
    return r;
}

void test_identity_quaternion_yields_zero_zero_minus_one() {
    Quat q{1.0f, 0.0f, 0.0f, 0.0f};
    float gx, gy, gz;
    quatToBodyGravity(q, gx, gy, gz);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, gx);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, gy);
    TEST_ASSERT_FLOAT_WITHIN(TOL, -1.0f, gz);
}

void test_pitch_forward_30deg() {
    const float half = 15.0f * (float)M_PI / 180.0f;
    Quat q{std::cos(half), 0.0f, std::sin(half), 0.0f};
    float gx, gy, gz;
    quatToBodyGravity(q, gx, gy, gz);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.5f, gx);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, gy);
    TEST_ASSERT_FLOAT_WITHIN(TOL, -std::cos(30.0f * (float)M_PI / 180.0f), gz);
}

void test_roll_left_30deg() {
    const float half = 15.0f * (float)M_PI / 180.0f;
    Quat q{std::cos(half), std::sin(half), 0.0f, 0.0f};
    float gx, gy, gz;
    quatToBodyGravity(q, gx, gy, gz);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, gx);
    TEST_ASSERT_FLOAT_WITHIN(TOL, -0.5f, gy);
    TEST_ASSERT_FLOAT_WITHIN(TOL, -std::cos(30.0f * (float)M_PI / 180.0f), gz);
}

void test_full_inversion_180deg_about_x() {
    Quat q{0.0f, 1.0f, 0.0f, 0.0f};
    float gx, gy, gz;
    quatToBodyGravity(q, gx, gy, gz);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0f, gz);
}

void test_grid_of_pitch_roll_combinations() {
    const float angles[] = {
        -30.0f * (float)M_PI / 180.0f,
        -15.0f * (float)M_PI / 180.0f,
         0.0f,
         15.0f * (float)M_PI / 180.0f,
         30.0f * (float)M_PI / 180.0f,
    };
    for (float pitch : angles) {
        for (float roll : angles) {
            Quat q_pitch{std::cos(pitch / 2.0f), 0.0f, std::sin(pitch / 2.0f), 0.0f};
            Quat q_roll {std::cos(roll  / 2.0f), std::sin(roll / 2.0f), 0.0f, 0.0f};
            Quat q = quatMul(q_roll, q_pitch);

            float gx, gy, gz;
            quatToBodyGravity(q, gx, gy, gz);

            // Ground truth for q = q_roll * q_pitch (roll about body-x applied
            // after pitch about body-y, scalar-first Hamilton convention).
            const float ex =  std::sin(pitch) * std::cos(roll);
            const float ey = -std::sin(roll);
            const float ez = -std::cos(roll) * std::cos(pitch);

            TEST_ASSERT_FLOAT_WITHIN(TOL, ex, gx);
            TEST_ASSERT_FLOAT_WITHIN(TOL, ey, gy);
            TEST_ASSERT_FLOAT_WITHIN(TOL, ez, gz);
        }
    }
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_identity_quaternion_yields_zero_zero_minus_one);
    RUN_TEST(test_pitch_forward_30deg);
    RUN_TEST(test_roll_left_30deg);
    RUN_TEST(test_full_inversion_180deg_about_x);
    RUN_TEST(test_grid_of_pitch_roll_combinations);
    return UNITY_END();
}
