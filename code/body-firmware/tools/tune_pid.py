"""Stub — RED phase. Tests must fail until GREEN implementation lands."""

CSV_COLUMNS = [
    "seq", "t_us", "dt_measured", "dt_used",
    "cmd_vx_raw", "cmd_vy_raw", "cmd_omega_raw",
    "cmd_vx", "cmd_vy", "cmd_omega", "cmd_age_ms",
    "quat_w", "quat_x", "quat_y", "quat_z",
    "accel_x", "accel_y", "accel_z",
    "gyro_x_raw", "gyro_y_raw", "gyro_z_raw",
    "gx", "gy", "gz", "tilt_mag_sin",
    "pitch_actual", "roll_actual",
    "gyro_pitch_rate", "gyro_roll_rate",
    "pitch_target", "roll_target",
    "pitch_err", "pitch_P", "pitch_I", "pitch_D", "pitch_out_raw", "pitch_out",
    "roll_err",  "roll_P",  "roll_I",  "roll_D",  "roll_out_raw",  "roll_out",
    "body_vx_cmd", "body_vy_cmd", "body_omega_cmd",
    "wheel_target_rpm_0", "wheel_target_rpm_1", "wheel_target_rpm_2",
    "wheel_meas_rpm_0",   "wheel_meas_rpm_1",   "wheel_meas_rpm_2",
    "wheel_P_0", "wheel_P_1", "wheel_P_2",
    "wheel_I_0", "wheel_I_1", "wheel_I_2",
    "wheel_D_0", "wheel_D_1", "wheel_D_2",
    "wheel_out_0", "wheel_out_1", "wheel_out_2",
    "pitch_Kp", "pitch_Ki", "pitch_Kd",
    "roll_Kp",  "roll_Ki",  "roll_Kd",
    "pitch_deadband", "roll_deadband",
    "max_output_velocity",
    "envelope_enter_sin", "envelope_exit_sin",
    "gyro_pitch_sign", "gyro_roll_sign",
    "tilt_per_velocity", "max_tilt_setpoint",
    "armed_state", "in_fault", "cmd_stale",
    "event_flags",
]

EVENT_BITS = {
    "ARMED_EDGE": 1 << 0, "DISARMED_EDGE": 1 << 1, "KILLED_EDGE": 1 << 2, "KILL_CLEARED": 1 << 3,
    "FAULT_ENTER": 1 << 4, "FAULT_EXIT": 1 << 5,
    "PITCH_DEADBAND_RESET": 1 << 6, "ROLL_DEADBAND_RESET": 1 << 7,
    "PITCH_I_SATURATED": 1 << 8, "ROLL_I_SATURATED": 1 << 9,
    "PITCH_OUT_SATURATED": 1 << 10, "ROLL_OUT_SATURATED": 1 << 11,
    "GAIN_CHANGED": 1 << 12, "CONFIG_SAVED": 1 << 13, "CONFIG_RESET": 1 << 14,
    "STEP_INJECTED": 1 << 15,
}


if __name__ == "__main__":
    import sys
    sys.stderr.write("tune_pid.py not yet implemented (RED phase)\n")
    sys.exit(2)
