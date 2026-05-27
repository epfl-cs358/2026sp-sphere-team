/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 */

#include "BalancingDrivetrainController.h"

#include <cmath>

#include "BalanceTuner.h"
#include "RobotConstants.h"
#include "quatToBodyGravity.h"

namespace {

inline float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

}  // namespace

BalancingDrivetrainController::BalancingDrivetrainController(
    Drivetrain<BodyVelocity>& drive,
    IMU<IMUReading>& imu,
    std::atomic<const BalanceConfig*>& configSlot)
    : DrivetrainController<BodyVelocity, IMUReading, BodyVelocity>(drive, imu),
      _pitchPid(configSlot.load(std::memory_order_acquire)->pitchKp,
                configSlot.load(std::memory_order_acquire)->pitchKi,
                configSlot.load(std::memory_order_acquire)->pitchKd,
                -configSlot.load(std::memory_order_acquire)->maxOutputVelocity,
                +configSlot.load(std::memory_order_acquire)->maxOutputVelocity),
      _rollPid(configSlot.load(std::memory_order_acquire)->rollKp,
               configSlot.load(std::memory_order_acquire)->rollKi,
               configSlot.load(std::memory_order_acquire)->rollKd,
               -configSlot.load(std::memory_order_acquire)->maxOutputVelocity,
               +configSlot.load(std::memory_order_acquire)->maxOutputVelocity),
      _yawRatePid(0.0f, 0.0f, 0.0f, -RobotConstants::OMEGA_MAX, +RobotConstants::OMEGA_MAX),
      _configSlot(configSlot) {}

void BalancingDrivetrainController::update(const BodyVelocity& cmd,
                                           const IMUReading& imuData,
                                           float dt) {
    const BalanceConfig& cfg = *_configSlot.load(std::memory_order_acquire);

    // Top-of-update IMU validity guard. A NaN quaternion or gyro.z would
    // poison every downstream math op (quatToBodyGravity → tilt → PID), and
    // the PID's BB8_ASSERT would abort. Skipping the drivetrain write is
    // safer than driving on garbage; persistent NaNs surface as a continuous
    // kEvent_IMU_INVALID stream in telemetry.
    const Quat& q = imuData.orientation;
    if (!std::isfinite(q.w) || !std::isfinite(q.x) || !std::isfinite(q.y) ||
        !std::isfinite(q.z) || !std::isfinite(imuData.gyro.z)) {
        _lastTelemetry = BalanceTelemetry{};
        _lastTelemetry.dt_used     = dt;
        _lastTelemetry.event_flags = kEvent_IMU_INVALID;
        _lastTelemetry.in_fault    = _inFault ? 1 : 0;
        return;
    }

    // Push live-tuned gains into the PIDs every tick. setGains is a 3-float
    // mutation; cost is negligible vs. allowing tuner edits to take effect.
    _pitchPid.setGains(cfg.pitchKp, cfg.pitchKi, cfg.pitchKd);
    _rollPid.setGains(cfg.rollKp, cfg.rollKi, cfg.rollKd);
    _yawRatePid.setGains(cfg.yawRateKp, cfg.yawRateKi, cfg.yawRateKd);
    _pitchPid.setDeadband(cfg.pitchDeadband);
    _rollPid.setDeadband(cfg.rollDeadband);

    // Telemetry is rebuilt every tick. Fields untouched here (wheel_*, seq,
    // t_us, dt_measured, cmd_age_ms, armed_state, cmd_stale) are owned by the
    // control task and overwritten before publishing.
    _lastTelemetry = BalanceTelemetry{};
    _lastTelemetry.dt_used = dt;
    _lastTelemetry.cmd_vx        = cmd.vx;
    _lastTelemetry.cmd_vy        = cmd.vy;
    _lastTelemetry.cmd_omega     = cmd.omega;
    _lastTelemetry.cmd_vx_raw    = cmd.vx;
    _lastTelemetry.cmd_vy_raw    = cmd.vy;
    _lastTelemetry.cmd_omega_raw = cmd.omega;
    _lastTelemetry.quat_w    = imuData.orientation.w;
    _lastTelemetry.quat_x    = imuData.orientation.x;
    _lastTelemetry.quat_y    = imuData.orientation.y;
    _lastTelemetry.quat_z    = imuData.orientation.z;
    // gravity-removed; raw accel would duplicate info in the quaternion.
    _lastTelemetry.accel_x   = imuData.linearAccel.x;
    _lastTelemetry.accel_y   = imuData.linearAccel.y;
    _lastTelemetry.accel_z   = imuData.linearAccel.z;
    _lastTelemetry.gyro_x_raw = imuData.gyro.x;
    _lastTelemetry.gyro_y_raw = imuData.gyro.y;
    _lastTelemetry.gyro_z_raw = imuData.gyro.z;
    _lastTelemetry.pitch_Kp = cfg.pitchKp;
    _lastTelemetry.pitch_Ki = cfg.pitchKi;
    _lastTelemetry.pitch_Kd = cfg.pitchKd;
    _lastTelemetry.roll_Kp  = cfg.rollKp;
    _lastTelemetry.roll_Ki  = cfg.rollKi;
    _lastTelemetry.roll_Kd  = cfg.rollKd;
    _lastTelemetry.yaw_rate_Kp = cfg.yawRateKp;
    _lastTelemetry.yaw_rate_Ki = cfg.yawRateKi;
    _lastTelemetry.yaw_rate_Kd = cfg.yawRateKd;
    _lastTelemetry.heading_Kp  = cfg.headingKp;
    _lastTelemetry.pitch_deadband      = cfg.pitchDeadband;
    _lastTelemetry.roll_deadband       = cfg.rollDeadband;
    _lastTelemetry.max_output_velocity = cfg.maxOutputVelocity;
    _lastTelemetry.envelope_enter_sin  = cfg.envelopeEnterSin;
    _lastTelemetry.envelope_exit_sin   = cfg.envelopeExitSin;
    _lastTelemetry.gyro_pitch_sign     = cfg.gyroPitchSign;
    _lastTelemetry.gyro_roll_sign      = cfg.gyroRollSign;
    _lastTelemetry.gyro_yaw_sign       = cfg.gyroYawSign;
    _lastTelemetry.tilt_per_velocity   = cfg.tiltPerVelocity;
    _lastTelemetry.max_tilt_setpoint   = cfg.maxTiltSetpoint;

    float gx, gy, gz;
    quatToBodyGravity(imuData.orientation, gx, gy, gz);
    float tiltMagSin = std::sqrt(gx * gx + gy * gy);
    _lastTelemetry.gx = gx;
    _lastTelemetry.gy = gy;
    _lastTelemetry.gz = gz;
    _lastTelemetry.tilt_mag_sin = tiltMagSin;

    // Schmitt fault gate. On entry, reset both balance PIDs so windup from
    // before the fault doesn't kick the wheels when the controller re-engages.
    if (!_inFault && tiltMagSin > cfg.envelopeEnterSin) {
        _inFault = true;
        _pitchPid.reset();
        _rollPid.reset();
        _yawRatePid.reset();
        _yawSpinActive = false;
        _yawSpinElapsedSec = 0.0f;
    }
    if (_inFault) {
        if (tiltMagSin <= cfg.envelopeExitSin) {
            _inFault = false;
        } else {
            _drivetrain.drive(BodyVelocity{0.0f, 0.0f, 0.0f});
            _finalizeTelemetry();
            return;
        }
    }

    float pitch_target = clampf(cfg.tiltPerVelocity * cmd.vx,
                                -cfg.maxTiltSetpoint, +cfg.maxTiltSetpoint);
    float roll_target  = clampf(cfg.tiltPerVelocity * cmd.vy,
                                -cfg.maxTiltSetpoint, +cfg.maxTiltSetpoint);

    float pitch_actual = std::atan2(gx, -gz);
    float roll_actual  = std::atan2(gy, -gz);

    float gyro_pitch_rate = cfg.gyroPitchSign * imuData.gyro.y;
    float gyro_roll_rate  = cfg.gyroRollSign  * imuData.gyro.x;
    float gyro_yaw_rate   = cfg.gyroYawSign   * imuData.gyro.z;

    _lastTelemetry.pitch_target    = pitch_target;
    _lastTelemetry.roll_target     = roll_target;
    _lastTelemetry.pitch_actual    = pitch_actual;
    _lastTelemetry.roll_actual     = roll_actual;
    _lastTelemetry.gyro_pitch_rate = gyro_pitch_rate;
    _lastTelemetry.gyro_roll_rate  = gyro_roll_rate;
    _lastTelemetry.gyro_yaw_rate   = gyro_yaw_rate;

    float vx_out = _pitchPid.compute(pitch_target, pitch_actual, gyro_pitch_rate, dt);
    float vy_out = _rollPid.compute(roll_target,  roll_actual,  gyro_roll_rate,  dt);

    _lastTelemetry.pitch_err     = _pitchPid.lastError();
    _lastTelemetry.pitch_P       = _pitchPid.lastP();
    _lastTelemetry.pitch_I       = _pitchPid.lastI();
    _lastTelemetry.pitch_D       = _pitchPid.lastD();
    _lastTelemetry.pitch_out_raw = _pitchPid.lastOutput();
    _lastTelemetry.roll_err      = _rollPid.lastError();
    _lastTelemetry.roll_P        = _rollPid.lastP();
    _lastTelemetry.roll_I        = _rollPid.lastI();
    _lastTelemetry.roll_D        = _rollPid.lastD();
    _lastTelemetry.roll_out_raw  = _rollPid.lastOutput();

    // Explicit clamp after PID; redundant with the PID's own clamp but the
    // spec algorithm calls for it as a safety net against gain glitches.
    vx_out = clampf(vx_out, -cfg.maxOutputVelocity, +cfg.maxOutputVelocity);
    vy_out = clampf(vy_out, -cfg.maxOutputVelocity, +cfg.maxOutputVelocity);
    _lastTelemetry.pitch_out = vx_out;
    _lastTelemetry.roll_out  = vy_out;

    // Yaw-spin recovery: edge-triggered latch on sustained over-threshold
    // |gyro_yaw_rate|. Exit symmetrically — once the elapsed-counter rolls
    // past the trip window in the OTHER direction, clear the latch.
    const float yawSpinTripSec = static_cast<float>(RobotConstants::YAW_SPIN_TRIP_MS) / 1000.0f;
    const bool aboveSpinThreshold = std::fabs(gyro_yaw_rate) > RobotConstants::YAW_SPIN_THRESHOLD;
    if (aboveSpinThreshold) {
        if (_yawSpinActive) {
            _yawSpinElapsedSec = yawSpinTripSec;
        } else {
            _yawSpinElapsedSec += dt;
            if (_yawSpinElapsedSec >= yawSpinTripSec) {
                _yawSpinActive = true;
                _lastTelemetry.event_flags |= kEvent_YAW_SPIN_RECOVERY;
            }
        }
    } else {
        if (_yawSpinActive) {
            _yawSpinElapsedSec -= dt;
            if (_yawSpinElapsedSec <= 0.0f) {
                _yawSpinActive = false;
                _yawSpinElapsedSec = 0.0f;
            }
        } else {
            _yawSpinElapsedSec = 0.0f;
        }
    }

    // Ship-safe regression guarantee: with all yaw gains == 0, the inner PID
    // would just emit zero (Kp*err = 0). Bypass it entirely so cmd.omega keeps
    // its pre-Commit-3 passthrough behavior — operators can disable closed-loop
    // yaw by zeroing all three gains without losing manual stick control.
    const bool yawLoopDisabled =
        (cfg.yawRateKp == 0.0f) && (cfg.yawRateKi == 0.0f) && (cfg.yawRateKd == 0.0f);
    float omega_out;
    if (_yawSpinActive) {
        omega_out = 0.0f;
        _yawRatePid.reset();
    } else if (yawLoopDisabled) {
        _yawRatePid.reset();
        omega_out = cmd.omega;
    } else {
        omega_out = _yawRatePid.compute(cmd.omega, gyro_yaw_rate, dt);
    }

    _lastTelemetry.yaw_rate_err = _yawRatePid.lastError();
    _lastTelemetry.yaw_rate_P   = _yawRatePid.lastP();
    _lastTelemetry.yaw_rate_I   = _yawRatePid.lastI();
    _lastTelemetry.yaw_rate_D   = _yawRatePid.lastD();
    _lastTelemetry.yaw_rate_out = omega_out;

    // Sphere sign convention (B1, resolved): for BB-8's internal drive,
    // tilting the body forward (pitch > 0) requires the shell to roll
    // BACKWARD to push the payload back over its base. Standard PID gives
    // `vx_out = Kp * (target - actual)`, which is positive-feedback under
    // the REP-103-clean kinematics. Negate at the boundary so positive Kp
    // gains in BalanceConfig remain physically intuitive ("how hard does
    // the controller push back against tilt") rather than forcing operators
    // to set negative gains (which BalanceTuner rejects). Prior to the
    // OmniKinematics.h vx sign fix this negation lived implicitly in the
    // kinematics — the two cancelled and the controller looked right by
    // coincidence.
    _lastTelemetry.body_vx_cmd    = -vx_out;
    _lastTelemetry.body_vy_cmd    = -vy_out;
    _lastTelemetry.body_omega_cmd = omega_out;
    _drivetrain.drive(BodyVelocity{-vx_out, -vy_out, omega_out});
    _finalizeTelemetry();
}

// Common tail: OR in fault-edge, PID-saturation, deadband-reset, and
// tuner-pending event bits, set in_fault, and advance _prevInFault. Called
// from both the normal exit and the in-fault early return so event bits and
// state always reflect the just-completed tick.
void BalancingDrivetrainController::_finalizeTelemetry() {
    // OR onto preserved bits set inline during update() (yaw-spin edge).
    uint32_t flags = _lastTelemetry.event_flags;
    if (!_prevInFault && _inFault) flags |= kEvent_FAULT_ENTER;
    if (_prevInFault && !_inFault) flags |= kEvent_FAULT_EXIT;
    if (_pitchPid.wasISaturated())     flags |= kEvent_PITCH_I_SATURATED;
    if (_rollPid.wasISaturated())      flags |= kEvent_ROLL_I_SATURATED;
    if (_pitchPid.wasOutSaturated())   flags |= kEvent_PITCH_OUT_SATURATED;
    if (_rollPid.wasOutSaturated())    flags |= kEvent_ROLL_OUT_SATURATED;
    if (_pitchPid.wasDeadbandReset())  flags |= kEvent_PITCH_DEADBAND_RESET;
    if (_rollPid.wasDeadbandReset())   flags |= kEvent_ROLL_DEADBAND_RESET;
    if (_yawRatePid.wasISaturated())   flags |= kEvent_YAW_RATE_I_SATURATED;
    if (_yawRatePid.wasOutSaturated()) flags |= kEvent_YAW_RATE_OUT_SATURATED;
    if (_tuner != nullptr)             flags |= _tuner->consumePending();
    _lastTelemetry.event_flags = flags;
    _lastTelemetry.in_fault    = _inFault ? 1 : 0;
    _prevInFault = _inFault;
}

void BalancingDrivetrainController::stop() {
    _drivetrain.stop();
    _pitchPid.reset();
    _rollPid.reset();
    _yawRatePid.reset();
    _inFault = false;
    _yawSpinActive = false;
    _yawSpinElapsedSec = 0.0f;
}

void BalancingDrivetrainController::resetIntegrators() {
    _pitchPid.reset();
    _rollPid.reset();
    _yawRatePid.reset();
}

// Disarmed→Armed edge. Wheel PIDs accumulated _prevMeasurement and _integral
// while update(dt) ran during Disarmed (chasing target=0 against real encoder
// readings). Clear them so the first armed tick computes outputs cleanly.
// Resetting the balance PIDs too is defensive — they should already be clean
// from the prior Armed→Disarmed edge, but a setGains tuning edit mid-Disarmed
// could leave them in a state we'd rather not start armed from.
void BalancingDrivetrainController::onArmed() {
    _drivetrain.resetPids();
    _pitchPid.reset();
    _rollPid.reset();
    _yawRatePid.reset();
    _yawSpinActive = false;
    _yawSpinElapsedSec = 0.0f;
}

// Armed→Disarmed edge. Mirrors onArmed so wheel PIDs don't carry I/D state
// into the next Disarmed period either. Subsumes the old resetIntegrators()
// call site in the main loop.
void BalancingDrivetrainController::onDisarmed() {
    _pitchPid.reset();
    _rollPid.reset();
    _yawRatePid.reset();
    _yawSpinActive = false;
    _yawSpinElapsedSec = 0.0f;
    _drivetrain.resetPids();
}
