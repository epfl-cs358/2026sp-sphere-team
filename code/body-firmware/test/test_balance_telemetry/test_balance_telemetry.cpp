/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 */

#include <cmath>
#include <cstdint>
#include <type_traits>

#include <unity.h>
#include "BalanceTelemetry.h"

void setUp() {}
void tearDown() {}

// 1. Default aggregate construction must zero every field.
void test_default_construction_zeros_all_fields() {
    BalanceTelemetry t{};

    // Timing
    TEST_ASSERT_EQUAL_UINT32(0u, t.seq);
    TEST_ASSERT_EQUAL_UINT32(0u, t.t_us);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.dt_measured);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.dt_used);

    // Operator command
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.cmd_vx_raw);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.cmd_vy_raw);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.cmd_omega_raw);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.cmd_vx);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.cmd_vy);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.cmd_omega);
    TEST_ASSERT_EQUAL_UINT32(0u, t.cmd_age_ms);

    // Raw IMU
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.quat_w);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.quat_x);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.quat_y);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.quat_z);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.accel_x);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.accel_y);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.accel_z);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.gyro_x_raw);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.gyro_y_raw);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.gyro_z_raw);

    // Orientation intermediates
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.gx);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.gy);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.gz);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.tilt_mag_sin);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.pitch_actual);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.roll_actual);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.gyro_pitch_rate);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.gyro_roll_rate);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.gyro_yaw_rate);

    // Setpoint shaping
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.pitch_target);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.roll_target);

    // Heading hold — 16 zero-defaulted yaw/heading fields, plus heading_setpoint = NaN
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.heading_integrated);
    TEST_ASSERT_TRUE(std::isnan(t.heading_setpoint));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.heading_err);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.heading_P);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.omega_target_raw);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.omega_target);

    // Yaw-rate inner PID
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.yaw_rate_err);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.yaw_rate_P);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.yaw_rate_I);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.yaw_rate_D);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.yaw_rate_out);

    // Yaw/heading gains + sign
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.yaw_rate_Kp);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.yaw_rate_Ki);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.yaw_rate_Kd);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.heading_Kp);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.gyro_yaw_sign);

    // Pitch PID
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.pitch_err);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.pitch_P);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.pitch_I);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.pitch_D);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.pitch_out_raw);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.pitch_out);

    // Roll PID
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.roll_err);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.roll_P);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.roll_I);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.roll_D);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.roll_out_raw);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.roll_out);

    // Body-frame command output
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.body_vx_cmd);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.body_vy_cmd);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.body_omega_cmd);

    // Wheel arrays
    for (int i = 0; i < 3; ++i) {
        TEST_ASSERT_EQUAL_FLOAT(0.0f, t.wheel_target_rpm[i]);
        TEST_ASSERT_EQUAL_FLOAT(0.0f, t.wheel_meas_rpm[i]);
        TEST_ASSERT_EQUAL_FLOAT(0.0f, t.wheel_P[i]);
        TEST_ASSERT_EQUAL_FLOAT(0.0f, t.wheel_I[i]);
        TEST_ASSERT_EQUAL_FLOAT(0.0f, t.wheel_D[i]);
        TEST_ASSERT_EQUAL_FLOAT(0.0f, t.wheel_out[i]);
    }

    // Live gains
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.pitch_Kp);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.pitch_Ki);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.pitch_Kd);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.roll_Kp);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.roll_Ki);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.roll_Kd);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.pitch_deadband);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.roll_deadband);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.max_output_velocity);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.envelope_enter_sin);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.envelope_exit_sin);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.tilt_per_velocity);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, t.max_tilt_setpoint);

    // State flags
    TEST_ASSERT_EQUAL_UINT8(0u, t.armed_state);
    TEST_ASSERT_EQUAL_UINT8(0u, t.in_fault);
    TEST_ASSERT_EQUAL_UINT8(0u, t.cmd_stale);
    TEST_ASSERT_EQUAL_UINT8(0u, t._pad);

    // Event bitfield
    TEST_ASSERT_EQUAL_UINT32(0u, t.event_flags);
}

// 2. All 16 event-bit constants are mutually exclusive (no shared bits).
void test_event_bit_constants_unique() {
    const uint32_t bits[16] = {
        kEvent_ARMED_EDGE,
        kEvent_DISARMED_EDGE,
        kEvent_KILLED_EDGE,
        kEvent_KILL_CLEARED,
        kEvent_FAULT_ENTER,
        kEvent_FAULT_EXIT,
        kEvent_PITCH_DEADBAND_RESET,
        kEvent_ROLL_DEADBAND_RESET,
        kEvent_PITCH_I_SATURATED,
        kEvent_ROLL_I_SATURATED,
        kEvent_PITCH_OUT_SATURATED,
        kEvent_ROLL_OUT_SATURATED,
        kEvent_GAIN_CHANGED,
        kEvent_CONFIG_SAVED,
        kEvent_CONFIG_RESET,
        kEvent_STEP_INJECTED,
    };

    for (int i = 0; i < 16; ++i) {
        // Each bit must be non-zero (a real single-bit mask).
        TEST_ASSERT_NOT_EQUAL(0u, bits[i]);
        // Each must have exactly one bit set (power of two).
        TEST_ASSERT_EQUAL_UINT32(0u, bits[i] & (bits[i] - 1u));
        for (int j = i + 1; j < 16; ++j) {
            TEST_ASSERT_EQUAL_UINT32(0u, bits[i] & bits[j]);
        }
    }
}

// 3. Bits compose correctly under OR (compositional invariant).
void test_event_bits_compose_with_or() {
    uint32_t flags = kEvent_ARMED_EDGE | kEvent_GAIN_CHANGED;

    // popcount == 2
    int popcount = 0;
    for (int i = 0; i < 32; ++i) {
        if (flags & (1u << i)) ++popcount;
    }
    TEST_ASSERT_EQUAL_INT(2, popcount);

    // Each individual bit must be retrievable.
    TEST_ASSERT_TRUE((flags & kEvent_ARMED_EDGE) != 0u);
    TEST_ASSERT_TRUE((flags & kEvent_GAIN_CHANGED) != 0u);
    // An unrelated bit must NOT be set.
    TEST_ASSERT_TRUE((flags & kEvent_FAULT_ENTER) == 0u);
}

// 4. sizeof sanity bound — leaves room to grow but flags accidental bloat.
void test_sizeof_under_threshold() {
    TEST_ASSERT_TRUE(sizeof(BalanceTelemetry) < 512);
}

// 5. POD traits — must remain standard-layout + trivially-copyable so the
//    snapshot can be copied across queues / DMA'd / memset'd without UB.
void test_pod_traits() {
    static_assert(std::is_standard_layout<BalanceTelemetry>::value,
                  "BalanceTelemetry must be standard-layout");
    static_assert(std::is_trivially_copyable<BalanceTelemetry>::value,
                  "BalanceTelemetry must be trivially-copyable");
    TEST_PASS();
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_default_construction_zeros_all_fields);
    RUN_TEST(test_event_bit_constants_unique);
    RUN_TEST(test_event_bits_compose_with_or);
    RUN_TEST(test_sizeof_under_threshold);
    RUN_TEST(test_pod_traits);
    return UNITY_END();
}
