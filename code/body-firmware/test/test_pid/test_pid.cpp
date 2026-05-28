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

void test_4arg_with_finite_diff_rate_matches_3arg() {
    PID pid3(2.0f, 0.5f, 0.1f, -10.0f, 10.0f);
    PID pid4(2.0f, 0.5f, 0.1f, -10.0f, 10.0f);
    float dt = 0.1f;

    // Identical warm-up call on both PIDs so _firstCompute flips together.
    pid3.compute(1.0f, 0.0f, dt);
    pid4.compute(1.0f, 0.0f, dt);

    float measurement_prev = 0.0f;
    float measurement = 0.3f;

    float out3 = pid3.compute(1.0f, measurement, dt);
    float rate = (measurement - measurement_prev) / dt;
    float out4 = pid4.compute(1.0f, measurement, rate, dt);

    TEST_ASSERT_FLOAT_WITHIN(TOL, out3, out4);
}

void test_4arg_with_external_rate_uses_supplied_rate() {
    PID pid(0.0f, 0.0f, 1.0f, -10.0f, 10.0f);

    // Seed state with a first call so _firstCompute flips to false and
    // _prevMeasurement is set to a non-zero value; we want to prove the
    // 4-arg path ignores _prevMeasurement and uses the supplied rate.
    pid.compute(0.0f, 5.0f, 0.1f);

    float out = pid.compute(0.0f, 0.0f, 2.0f, 0.1f);
    TEST_ASSERT_FLOAT_WITHIN(TOL, -2.0f, out);
}

void test_4arg_respects_anti_windup() {
    PID pid(0.0f, 1.0f, 0.0f, -1.0f, 1.0f);
    float dt = 0.1f;
    float rate = 0.0f;

    for (int i = 0; i < 10; ++i) {
        pid.compute(10.0f, 0.0f, rate, dt);
    }
    float saturated = pid.compute(10.0f, 0.0f, rate, dt);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0f, saturated);
}

void test_4arg_respects_deadband() {
    PID pid(1.0f, 1.0f, 1.0f, -10.0f, 10.0f, 0.5f);
    float dt = 0.1f;

    // Seed state so reset is observable.
    pid.compute(1.0f, 0.0f, 0.0f, dt);
    pid.compute(1.0f, 0.0f, 0.0f, dt);

    float out = pid.compute(0.0f, 0.2f, 0.0f, dt);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out);

    // After deadband-triggered reset, the next zero-error call must produce
    // a single dt's worth of integral (0.1) and not the accumulated history.
    float follow = pid.compute(1.0f, 0.0f, 0.0f, dt);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0f + 0.1f, follow);
}

void test_set_gains_changes_output() {
    PID pid(1.0f, 0.0f, 0.0f, -10.0f, 10.0f);
    float dt = 0.01f;

    // Baseline: kp=1, error=1 -> output ~1
    float before = pid.compute(1.0f, 0.0f, dt);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 1.0f, before);

    // Bump kp to 3 and re-issue the same setpoint/measurement.
    pid.setGains(3.0f, 0.0f, 0.0f);
    float after = pid.compute(1.0f, 0.0f, dt);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 3.0f, after);
}

void test_accessors_reflect_last_compute() {
    PID pid(2.0f, 0.5f, 0.1f, -10.0f, 10.0f);
    float dt = 0.1f;

    // Warm-up so _firstCompute flips and the D-term engages on the next call.
    pid.compute(1.0f, 0.0f, dt);

    float out = pid.compute(1.0f, 0.3f, dt);

    TEST_ASSERT_FLOAT_WITHIN(TOL, out, pid.lastP() + pid.lastI() + pid.lastD());
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.7f, pid.lastError());
    TEST_ASSERT_FLOAT_WITHIN(TOL, out, pid.lastOutput());
}

void test_was_deadband_reset_flag() {
    PID pid(1.0f, 1.0f, 1.0f, -10.0f, 10.0f, 0.5f);
    float dt = 0.1f;

    float out = pid.compute(0.0f, 0.2f, dt);
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, out);
    TEST_ASSERT_TRUE(pid.wasDeadbandReset());

    pid.compute(1.0f, 0.0f, dt);
    TEST_ASSERT_FALSE(pid.wasDeadbandReset());
}

void test_was_i_saturated_flag() {
    PID pid(0.0f, 1.0f, 0.0f, -1.0f, 1.0f);
    float dt = 0.1f;

    for (int i = 0; i < 100; ++i) {
        pid.compute(10.0f, 0.0f, dt);
    }
    TEST_ASSERT_TRUE(pid.wasISaturated());

    // Setpoint==measurement==0 → no integral growth, flag must clear.
    pid.compute(0.0f, 0.0f, dt);
    TEST_ASSERT_FALSE(pid.wasISaturated());
}

void test_was_out_saturated_flag() {
    PID pid(10.0f, 0.0f, 0.0f, -1.0f, 1.0f);
    float dt = 0.01f;

    pid.compute(100.0f, 0.0f, dt);
    TEST_ASSERT_TRUE(pid.wasOutSaturated());

    pid.compute(0.05f, 0.0f, dt);
    TEST_ASSERT_FALSE(pid.wasOutSaturated());
}

void test_reset_clears_all_introspection() {
    PID pid(10.0f, 1.0f, 0.1f, -1.0f, 1.0f, 0.5f);
    float dt = 0.1f;

    // Drive into saturation + nonzero terms.
    for (int i = 0; i < 50; ++i) {
        pid.compute(100.0f, 0.0f, dt);
    }
    TEST_ASSERT_TRUE(pid.wasISaturated() || pid.wasOutSaturated());

    pid.reset();

    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, pid.lastP());
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, pid.lastI());
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, pid.lastD());
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, pid.lastError());
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, pid.lastIntegral());
    TEST_ASSERT_FLOAT_WITHIN(TOL, 0.0f, pid.lastOutput());
    TEST_ASSERT_FALSE(pid.wasDeadbandReset());
    TEST_ASSERT_FALSE(pid.wasISaturated());
    TEST_ASSERT_FALSE(pid.wasOutSaturated());
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

    RUN_TEST(test_4arg_with_finite_diff_rate_matches_3arg);
    RUN_TEST(test_4arg_with_external_rate_uses_supplied_rate);
    RUN_TEST(test_4arg_respects_anti_windup);
    RUN_TEST(test_4arg_respects_deadband);

    RUN_TEST(test_set_gains_changes_output);

    RUN_TEST(test_accessors_reflect_last_compute);
    RUN_TEST(test_was_deadband_reset_flag);
    RUN_TEST(test_was_i_saturated_flag);
    RUN_TEST(test_was_out_saturated_flag);
    RUN_TEST(test_reset_clears_all_introspection);

    return UNITY_END();
}
