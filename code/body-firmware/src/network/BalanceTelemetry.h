/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 */

#pragma once

#include <cstdint>

// Per-tick snapshot of the balancing controller's full input/intermediate/
// output state, plus an event-flag bitfield latching cause→effect edges for
// the current tick only.
//
// Lives in src/network/ because it's a wire-format type: serialized as CSV by
// BalanceTelemetryWs and as JSON by BalanceTelemetryHttpApi.
//
// POD only — no constructors, no methods. Must remain standard-layout and
// trivially-copyable so snapshots can move across FreeRTOS queues, be memcpy'd
// into ring buffers, or zero-initialized via aggregate `{}` without UB.

// One-shot event bits (set for the tick on which the edge fires, cleared on
// the next tick). Adjacent to the struct — not nested — so they're usable
// from C interfaces, format strings, and constexpr contexts. Bits 16-31 are
// reserved for future expansion.
static constexpr uint32_t kEvent_ARMED_EDGE           = 1u << 0;
static constexpr uint32_t kEvent_DISARMED_EDGE        = 1u << 1;
static constexpr uint32_t kEvent_KILLED_EDGE          = 1u << 2;
static constexpr uint32_t kEvent_KILL_CLEARED         = 1u << 3;
static constexpr uint32_t kEvent_FAULT_ENTER          = 1u << 4;
static constexpr uint32_t kEvent_FAULT_EXIT           = 1u << 5;
static constexpr uint32_t kEvent_PITCH_DEADBAND_RESET = 1u << 6;
static constexpr uint32_t kEvent_ROLL_DEADBAND_RESET  = 1u << 7;
static constexpr uint32_t kEvent_PITCH_I_SATURATED    = 1u << 8;
static constexpr uint32_t kEvent_ROLL_I_SATURATED     = 1u << 9;
static constexpr uint32_t kEvent_PITCH_OUT_SATURATED  = 1u << 10;
static constexpr uint32_t kEvent_ROLL_OUT_SATURATED   = 1u << 11;
static constexpr uint32_t kEvent_GAIN_CHANGED         = 1u << 12;
static constexpr uint32_t kEvent_CONFIG_SAVED         = 1u << 13;
static constexpr uint32_t kEvent_CONFIG_RESET         = 1u << 14;
static constexpr uint32_t kEvent_STEP_INJECTED        = 1u << 15;

struct BalanceTelemetry {
    // --- timing ---
    uint32_t seq;            // monotonic publish counter (gap detection)
    uint32_t t_us;           // micros() at tick start
    float    dt_measured;    // sec, real delta between ticks
    float    dt_used;        // sec, value actually fed to PID (nominal for now)

    // --- inputs: operator command ---
    float    cmd_vx_raw, cmd_vy_raw, cmd_omega_raw;  // freshest from latch
    float    cmd_vx,     cmd_vy,     cmd_omega;      // after staleness ramp
    uint32_t cmd_age_ms;                              // ms since last fresh

    // --- inputs: raw IMU ---
    float    quat_w, quat_x, quat_y, quat_z;
    float    accel_x, accel_y, accel_z;              // m/s^2
    float    gyro_x_raw, gyro_y_raw, gyro_z_raw;     // rad/s, pre-sign-flip

    // --- intermediates: orientation derivation ---
    float    gx, gy, gz;             // body-frame gravity unit components
    float    tilt_mag_sin;           // sqrt(gx^2 + gy^2)
    float    pitch_actual, roll_actual;
    float    gyro_pitch_rate, gyro_roll_rate;  // post-sign-flip, fed to D

    // --- intermediates: setpoint shaping ---
    float    pitch_target, roll_target;

    // --- pitch PID (every term split) ---
    float    pitch_err, pitch_P, pitch_I, pitch_D, pitch_out_raw, pitch_out;

    // --- roll PID ---
    float    roll_err, roll_P, roll_I, roll_D, roll_out_raw, roll_out;

    // --- outputs: body-frame command sent to drivetrain ---
    float    body_vx_cmd, body_vy_cmd, body_omega_cmd;  // post-sign-flip

    // --- outputs: per-wheel ---
    float    wheel_target_rpm[3];
    float    wheel_meas_rpm[3];
    float    wheel_P[3], wheel_I[3], wheel_D[3], wheel_out[3];

    // --- live gain values (a /balance/set is visible in the row it took effect) ---
    float    pitch_Kp, pitch_Ki, pitch_Kd;
    float    roll_Kp,  roll_Ki,  roll_Kd;
    float    pitch_deadband, roll_deadband;
    float    max_output_velocity;
    float    envelope_enter_sin, envelope_exit_sin;
    float    gyro_pitch_sign, gyro_roll_sign;
    float    tilt_per_velocity, max_tilt_setpoint;

    // --- state flags ---
    uint8_t  armed_state;   // 0=Disarmed, 1=Armed, 2=Killed
    uint8_t  in_fault;      // Schmitt envelope tripped
    uint8_t  cmd_stale;     // staleness ramp active
    uint8_t  _pad;          // explicit padding to 4-byte alignment for event_flags

    // --- event_flags: see kEvent_* constants above ---
    uint32_t event_flags;
};
