/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 */

#include "BalancingDrivetrainController.h"

#include <cmath>

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
      _configSlot(configSlot) {}

void BalancingDrivetrainController::update(const BodyVelocity& cmd,
                                           const IMUReading& imuData,
                                           float dt) {
    const BalanceConfig& cfg = *_configSlot.load(std::memory_order_acquire);

    // Push live-tuned gains into the PIDs every tick. setGains is a 3-float
    // mutation; cost is negligible vs. allowing tuner edits to take effect.
    _pitchPid.setGains(cfg.pitchKp, cfg.pitchKi, cfg.pitchKd);
    _rollPid.setGains(cfg.rollKp, cfg.rollKi, cfg.rollKd);

    float gx, gy, gz;
    quatToBodyGravity(imuData.orientation, gx, gy, gz);
    float tiltMagSin = std::sqrt(gx * gx + gy * gy);

    // Schmitt fault gate. On entry, reset both balance PIDs so windup from
    // before the fault doesn't kick the wheels when the controller re-engages.
    if (!_inFault && tiltMagSin > cfg.envelopeEnterSin) {
        _inFault = true;
        _pitchPid.reset();
        _rollPid.reset();
    }
    if (_inFault) {
        if (tiltMagSin <= cfg.envelopeExitSin) {
            _inFault = false;
        } else {
            _drivetrain.drive(BodyVelocity{0.0f, 0.0f, 0.0f});
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

    float vx_out = _pitchPid.compute(pitch_target, pitch_actual, gyro_pitch_rate, dt);
    float vy_out = _rollPid.compute(roll_target,  roll_actual,  gyro_roll_rate,  dt);

    // Explicit clamp after PID; redundant with the PID's own clamp but the
    // spec algorithm calls for it as a safety net against gain glitches.
    vx_out = clampf(vx_out, -cfg.maxOutputVelocity, +cfg.maxOutputVelocity);
    vy_out = clampf(vy_out, -cfg.maxOutputVelocity, +cfg.maxOutputVelocity);

    _drivetrain.drive(BodyVelocity{vx_out, vy_out, cmd.omega});
}

void BalancingDrivetrainController::stop() {
    _drivetrain.stop();
    _pitchPid.reset();
    _rollPid.reset();
    _inFault = false;
}
