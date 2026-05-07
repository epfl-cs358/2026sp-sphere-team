#include <unity.h>
#include "PID.h"

static constexpr float TOL = 0.001f;

void setUp() {}
void tearDown() {}

void test_p_only_positive_error() {
    PID pid(1.0f, 0.0f, 0.0f, -1.0f, 1.0f);
    float out = pid.compute(1.0f, 0.5f, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.5f, out);
}

void test_p_only_negative_error() {
    PID pid(1.0f, 0.0f, 0.0f, -1.0f, 1.0f);
    float out = pid.compute(0.0f, 0.7f, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(TOL, -0.7f, out);
}

void test_p_only_clamped() {
    PID pid(1.0f, 0.0f, 0.0f, -1.0f, 1.0f);
    float out = pid.compute(10.0f, 0.0f, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0f, out);
}

void test_i_only_accumulates() {
    PID pid(0.0f, 1.0f, 0.0f, -1.0f, 1.0f);
    float dt = 0.1f;
    float out1 = pid.compute(1.0f, 0.0f, dt);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.1f, out1);

    float out2 = pid.compute(1.0f, 0.0f, dt);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.2f, out2);

    float out3 = pid.compute(1.0f, 0.0f, dt);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.3f, out3);
}

void test_d_only_responds_to_measurement_change() {
    PID pid(0.0f, 0.0f, 1.0f, -1.0f, 1.0f);
    float dt = 0.1f;
    float out1 = pid.compute(0.0f, 0.0f, dt);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out1);

    float out2 = pid.compute(0.0f, 0.5f, dt);
    TEST_ASSERT_FLOAT_WITHIN(TOL, -1.0f, out2);
}

void test_d_only_first_call_zero() {
    PID pid(0.0f, 0.0f, 1.0f, -10.0f, 10.0f);
    float out = pid.compute(5.0f, 3.0f, 0.1f);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out);
}

void test_output_clamped_high() {
    PID pid(10.0f, 0.0f, 0.0f, -1.0f, 1.0f);
    float out = pid.compute(100.0f, 0.0f, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0f, out);
}

void test_output_clamped_low() {
    PID pid(10.0f, 0.0f, 0.0f, -1.0f, 1.0f);
    float out = pid.compute(-100.0f, 0.0f, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(TOL, -1.0f, out);
}

void test_anti_windup() {
    PID pid(0.0f, 1.0f, 0.0f, -1.0f, 1.0f);
    float dt = 0.1f;

    for (int i = 0; i < 100; ++i) {
        pid.compute(10.0f, 0.0f, dt);
    }
    float saturated = pid.compute(10.0f, 0.0f, dt);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0f, saturated);

    float reversed = pid.compute(-10.0f, 0.0f, dt);
    TEST_ASSERT_TRUE(reversed < 1.0f);

    for (int i = 0; i < 3; ++i) {
        reversed = pid.compute(-10.0f, 0.0f, dt);
    }
    TEST_ASSERT_TRUE(reversed < 0.0f);
}

void test_reset_clears_state() {
    PID pid(0.0f, 1.0f, 0.0f, -10.0f, 10.0f);
    float dt = 0.1f;

    pid.compute(1.0f, 0.0f, dt);
    pid.compute(1.0f, 0.0f, dt);
    pid.compute(1.0f, 0.0f, dt);

    pid.reset();

    float out = pid.compute(1.0f, 0.0f, dt);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.1f, out);
}

void test_reset_clears_derivative() {
    PID pid(0.0f, 0.0f, 1.0f, -10.0f, 10.0f);
    float dt = 0.1f;

    pid.compute(0.0f, 5.0f, dt);
    pid.reset();

    float out = pid.compute(0.0f, 3.0f, dt);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out);
}

void test_derivative_on_measurement_no_spike_on_setpoint_change() {
    PID pid(0.0f, 0.0f, 1.0f, -10.0f, 10.0f);
    float dt = 0.1f;

    pid.compute(0.0f, 1.0f, dt);

    float out = pid.compute(100.0f, 1.0f, dt);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out);
}

void test_zero_error_output_zero() {
    PID pid(1.0f, 0.0f, 0.0f, -1.0f, 1.0f);
    float out = pid.compute(5.0f, 5.0f, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out);
}

void test_zero_error_integral_stays() {
    PID pid(0.0f, 1.0f, 0.0f, -10.0f, 10.0f);
    float dt = 0.1f;

    pid.compute(1.0f, 0.0f, dt);
    pid.compute(1.0f, 0.0f, dt);

    float out = pid.compute(0.0f, 0.0f, dt);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.2f, out);

    out = pid.compute(0.0f, 0.0f, dt);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.2f, out);
}

void test_combined_pid() {
    PID pid(2.0f, 0.5f, 0.1f, -10.0f, 10.0f);
    float dt = 0.1f;

    float out1 = pid.compute(1.0f, 0.0f, dt);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 2.05f, out1);

    float out2 = pid.compute(1.0f, 0.3f, dt);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.185f, out2);
}

void test_zero_dt_returns_last_output() {
    PID pid(1.0f, 0.0f, 0.0f, -1.0f, 1.0f);
    float first = pid.compute(1.0f, 0.5f, 0.01f);
    float second = pid.compute(2.0f, 0.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(TOL, first, second);
}

void test_sub_threshold_dt_returns_last_output() {
    PID pid(1.0f, 0.0f, 0.0f, -1.0f, 1.0f);
    float first = pid.compute(1.0f, 0.5f, 0.01f);
    float second = pid.compute(2.0f, 0.0f, 1e-5f);
    TEST_ASSERT_FLOAT_WITHIN(TOL, first, second);
}

void test_large_dt_output_finite_and_clamped() {
    PID pid(0.0f, 1.0f, 0.0f, -1.0f, 1.0f);
    float out = pid.compute(10.0f, 0.0f, 100.0f);
    TEST_ASSERT_TRUE(std::isfinite(out));
    TEST_ASSERT_TRUE(out >= -1.0f && out <= 1.0f);
}

void test_anti_windup_fast_recovery() {
    PID pid(0.0f, 1.0f, 0.0f, -1.0f, 1.0f);
    float dt = 0.1f;

    for (int i = 0; i < 100; ++i) {
        pid.compute(10.0f, 0.0f, dt);
    }

    float out = 1.0f;
    int steps = 0;
    for (int i = 0; i < 10; ++i) {
        out = pid.compute(-10.0f, 0.0f, dt);
        steps++;
        if (out < 0.0f) break;
    }
    TEST_ASSERT_TRUE(steps <= 3);
    TEST_ASSERT_TRUE(out < 0.0f);
}

int main() {
    UNITY_BEGIN();

    RUN_TEST(test_p_only_positive_error);
    RUN_TEST(test_p_only_negative_error);
    RUN_TEST(test_p_only_clamped);

    RUN_TEST(test_i_only_accumulates);

    RUN_TEST(test_d_only_responds_to_measurement_change);
    RUN_TEST(test_d_only_first_call_zero);

    RUN_TEST(test_output_clamped_high);
    RUN_TEST(test_output_clamped_low);

    RUN_TEST(test_anti_windup);

    RUN_TEST(test_reset_clears_state);
    RUN_TEST(test_reset_clears_derivative);

    RUN_TEST(test_derivative_on_measurement_no_spike_on_setpoint_change);

    RUN_TEST(test_zero_error_output_zero);
    RUN_TEST(test_zero_error_integral_stays);

    RUN_TEST(test_combined_pid);

    RUN_TEST(test_zero_dt_returns_last_output);
    RUN_TEST(test_sub_threshold_dt_returns_last_output);
    RUN_TEST(test_large_dt_output_finite_and_clamped);
    RUN_TEST(test_anti_windup_fast_recovery);

    return UNITY_END();
}
