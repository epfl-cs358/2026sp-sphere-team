/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 */

#include <unity.h>
#include <atomic>
#include <cmath>
#include <limits>

#include <fff.h>
DEFINE_FFF_GLOBALS;

#include "Preferences.h"
// FFF fake bodies for Preferences (BalanceTuner pulls in BalanceConfigStorage
// which in turn references prefs_* shims).
DEFINE_FAKE_VALUE_FUNC(bool, prefs_begin, const char*, bool);
DEFINE_FAKE_VALUE_FUNC(size_t, prefs_getBytes, const char*, void*, size_t);
DEFINE_FAKE_VALUE_FUNC(size_t, prefs_putBytes, const char*, const void*, size_t);
DEFINE_FAKE_VOID_FUNC(prefs_end);

#include "BalancingDrivetrainController.h"
#include "BalancingDrivetrainController.cpp"
#include "BalanceConfig.h"
#include "ArmingState.h"
#include "ArmingState.cpp"
#include "BalanceConfigStorage.h"
#include "BalanceConfigStorage.cpp"
#include "BalanceTelemetry.h"
#include "BalanceTuner.h"
#include "BalanceTuner.cpp"
#include "BodyVelocity.h"
#include "IMUReading.h"
#include "Drivetrain.h"
#include "IMU.h"
#include "Quat.h"
#include "RobotConstants.h"
#include "test_mock_motor.h"

class MockDrivetrain : public Drivetrain<BodyVelocity> {
public:
    MockDrivetrain() : Drivetrain<BodyVelocity>(_m0, _m1, _m2) {}

    void drive(const BodyVelocity& v) override {
        lastDrive = v;
        driveCallCount++;
    }
    void update(float /*dt*/) override {}
    void stop() override { stopCallCount++; }
    void resetPids() override { resetPidsCallCount++; }

    BodyVelocity lastDrive{};
    int driveCallCount = 0;
    int stopCallCount = 0;
    int resetPidsCallCount = 0;

private:
    MockMotor _m0, _m1, _m2;
};

class MockIMU : public IMU<IMUReading> {
public:
    bool begin() override { return true; }
    IMUReading read() override { return IMUReading{}; }
};

// ---- helpers -------------------------------------------------------------

static BalanceConfig makeTestConfig() {
    // Small, predictable gains. tiltPerVelocity matches RobotConstants default so
    // the "at_target_tilt_settles_to_zero" test math is the documented 0.30 * vx.
    return {
        .tiltPerVelocity   = 0.30f,
        .maxTiltSetpoint   = 0.35f,
        .pitchKp = 2.0f, .pitchKi = 0.0f, .pitchKd = 0.0f,
        .rollKp  = 2.0f, .rollKi  = 0.0f, .rollKd  = 0.0f,
        .pitchDeadband     = 0.0f,
        .rollDeadband      = 0.0f,
        .maxOutputVelocity = 1.0f,
        .envelopeEnterSin  = std::sin(60.0f * 3.14159265f / 180.0f),
        .envelopeExitSin   = std::sin(55.0f * 3.14159265f / 180.0f),
        .gyroPitchSign     = 1.0f,
        .gyroRollSign      = 1.0f,
        .yawRateKp = 0.0f, .yawRateKi = 0.0f,
        .headingKp         = 0.0f,
        .gyroYawSign       = 1.0f,
    };
}

static Quat upright() {
    // Identity quaternion: body axes == world axes; gravity in body frame = (0, 0, -1).
    return Quat{1.0f, 0.0f, 0.0f, 0.0f};
}

// Rotation about body-Y by +rad (forward tilt). Yields gx = sin(rad), gz = -cos(rad).
// pitch_actual = atan2(gx, -gz) = rad.
static Quat pitchedForward(float rad) {
    Quat q;
    q.w = std::cos(rad * 0.5f);
    q.x = 0.0f;
    q.y = std::sin(rad * 0.5f);
    q.z = 0.0f;
    return q;
}

// Rotation about body-X by -rad (left tilt). Yields gy = +sin(rad), gz = -cos(rad).
// roll_actual = atan2(gy, -gz) = rad (positive when leaned left in our convention).
static Quat rolledLeft(float rad) {
    Quat q;
    q.w = std::cos(rad * 0.5f);
    q.x = -std::sin(rad * 0.5f);
    q.y = 0.0f;
    q.z = 0.0f;
    return q;
}

// ---- fixtures ------------------------------------------------------------

static MockDrivetrain* drivetrain = nullptr;
static MockIMU* imu = nullptr;
static BalanceConfig* cfgBuf = nullptr;
static std::atomic<const BalanceConfig*>* slot = nullptr;
static BalancingDrivetrainController* controller = nullptr;

void setUp() {
    drivetrain = new MockDrivetrain();
    imu = new MockIMU();
    cfgBuf = new BalanceConfig(makeTestConfig());
    slot = new std::atomic<const BalanceConfig*>(cfgBuf);
    controller = new BalancingDrivetrainController(*drivetrain, *imu, *slot);
}

void tearDown() {
    delete controller;
    delete slot;
    delete cfgBuf;
    delete imu;
    delete drivetrain;
    controller = nullptr;
    slot = nullptr;
    cfgBuf = nullptr;
    imu = nullptr;
    drivetrain = nullptr;
}

// ---- tests ---------------------------------------------------------------

static IMUReading makeIMU(const Quat& q) {
    IMUReading r{};
    r.orientation = q;
    r.gyro = Vec3{0.0f, 0.0f, 0.0f};
    return r;
}

void test_zero_command_level_platform_emits_zero() {
    BodyVelocity cmd{0.0f, 0.0f, 0.0f};
    IMUReading imuData = makeIMU(upright());
    controller->update(cmd, imuData, 0.01f);

    TEST_ASSERT_EQUAL(1, drivetrain->driveCallCount);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, drivetrain->lastDrive.vx);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, drivetrain->lastDrive.vy);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, drivetrain->lastDrive.omega);
}

void test_zero_command_pitched_forward_drives_shell_backward() {
    // Sphere B1 convention: pitched forward (gx > 0) → controller commands
    // shell to roll BACKWARD to push the payload back over its base.
    // Post-kinematics-fix, shell-backward corresponds to vx_out > 0 emitted
    // to the drivetrain (kinematics maps +vx_out → shell forward; controller
    // negates output so payload-forward-tilt → wheel command rolls payload back).
    BodyVelocity cmd{0.0f, 0.0f, 0.0f};
    float tilt = 10.0f * 3.14159265f / 180.0f;
    IMUReading imuData = makeIMU(pitchedForward(tilt));
    controller->update(cmd, imuData, 0.01f);

    TEST_ASSERT_EQUAL(1, drivetrain->driveCallCount);
    TEST_ASSERT_TRUE(drivetrain->lastDrive.vx > 0.0f);
}

void test_zero_command_rolled_left_drives_shell_right() {
    // Mirror of pitch: rolled left (gy > 0) → command rolls shell RIGHT to
    // push payload back to vertical. Post-fix, vy_out > 0.
    BodyVelocity cmd{0.0f, 0.0f, 0.0f};
    float tilt = 10.0f * 3.14159265f / 180.0f;
    IMUReading imuData = makeIMU(rolledLeft(tilt));
    controller->update(cmd, imuData, 0.01f);

    TEST_ASSERT_EQUAL(1, drivetrain->driveCallCount);
    TEST_ASSERT_TRUE(drivetrain->lastDrive.vy > 0.0f);
}

void test_forward_command_level_emits_negative_vx() {
    // Operator commands +vx (forward) on level platform: pitch_target > 0
    // (lean forward), pitch_actual = 0, error > 0. Under the sphere B1
    // convention the controller negates its output, so vx_out < 0 — wheels
    // are commanded to push the SHELL backward (which tips the payload
    // forward, initiating the desired forward translation). Steady-state
    // would equilibrate at the leaned-forward angle with output ≈ 0.
    BodyVelocity cmd{0.5f, 0.0f, 0.0f};
    IMUReading imuData = makeIMU(upright());

    // Several iterations to overcome any first-call derivative quirks.
    for (int i = 0; i < 3; ++i) {
        controller->update(cmd, imuData, 0.01f);
    }

    TEST_ASSERT_TRUE(drivetrain->lastDrive.vx < 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, drivetrain->lastDrive.vy);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, drivetrain->lastDrive.omega);
}

void test_forward_command_at_target_tilt_settles_to_zero() {
    // pitch_target = 0.30 * 0.5 = 0.15 rad. Build actual quat at that pitch.
    BodyVelocity cmd{0.5f, 0.0f, 0.0f};
    float targetTilt = 0.30f * 0.5f;
    IMUReading imuData = makeIMU(pitchedForward(targetTilt));
    controller->update(cmd, imuData, 0.01f);

    // Error has converged; no integral built; expect ~0.
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, drivetrain->lastDrive.vx);
}

void test_enter_fault_above_enter_envelope() {
    // Warm up _pitchPid with some integral via a config that has Ki > 0.
    BalanceConfig hot = makeTestConfig();
    hot.pitchKi = 1.0f;
    hot.rollKi  = 1.0f;
    *cfgBuf = hot;

    BodyVelocity cmd{0.5f, 0.5f, 0.0f};
    IMUReading level = makeIMU(upright());
    for (int i = 0; i < 5; ++i) {
        controller->update(cmd, level, 0.01f);
    }

    // Trip fault at 65°.
    float bigTilt = 65.0f * 3.14159265f / 180.0f;
    IMUReading bad = makeIMU(pitchedForward(bigTilt));
    controller->update(cmd, bad, 0.01f);

    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, drivetrain->lastDrive.vx);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, drivetrain->lastDrive.vy);

    // Now exit the envelope and call at level: integral must have been reset on
    // fault entry, so a single call with zero command/level produces zero.
    BodyVelocity zero{0.0f, 0.0f, 0.0f};
    controller->update(zero, level, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, drivetrain->lastDrive.vx);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, drivetrain->lastDrive.vy);
}

void test_in_fault_between_thresholds_stays_in_fault() {
    BodyVelocity cmd{0.5f, 0.0f, 0.0f};
    // Trip fault at 65°.
    IMUReading bad = makeIMU(pitchedForward(65.0f * 3.14159265f / 180.0f));
    controller->update(cmd, bad, 0.01f);

    // 57° is below 60° enter but above 55° exit → still in fault.
    IMUReading mid = makeIMU(pitchedForward(57.0f * 3.14159265f / 180.0f));
    controller->update(cmd, mid, 0.01f);

    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, drivetrain->lastDrive.vx);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, drivetrain->lastDrive.vy);
}

void test_exit_fault_below_exit_envelope() {
    BodyVelocity cmd{0.5f, 0.0f, 0.0f};
    // Enter fault at 65°.
    IMUReading bad = makeIMU(pitchedForward(65.0f * 3.14159265f / 180.0f));
    controller->update(cmd, bad, 0.01f);

    // Drop below 55° → exit fault, normal operation resumes.
    IMUReading recovered = makeIMU(pitchedForward(50.0f * 3.14159265f / 180.0f));
    controller->update(cmd, recovered, 0.01f);

    TEST_ASSERT_TRUE(drivetrain->lastDrive.vx != 0.0f);
}

void test_output_clamp_at_max_velocity() {
    BalanceConfig huge = makeTestConfig();
    huge.pitchKp = 1e6f;
    huge.rollKp  = 1e6f;
    huge.maxOutputVelocity = 1.0f;
    *cfgBuf = huge;

    BodyVelocity cmd{0.5f, 0.0f, 0.0f};
    IMUReading level = makeIMU(upright());
    controller->update(cmd, level, 0.01f);

    // After controller-output negation (B1 sphere convention) saturated
    // positive PID output appears as -maxOutputVelocity at the drive() call.
    TEST_ASSERT_TRUE(std::fabs(drivetrain->lastDrive.vx) <= 1.0f + 1e-6f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -1.0f, drivetrain->lastDrive.vx);
}

// Baseline pin: stop() has THREE observable effects — clears PIDs, calls
// drivetrain stop(), and clears _inFault. This is intentionally too aggressive
// for the Disarmed path; resetIntegrators() (next test) will provide the
// smaller-surface alternative that only does the first. Authored as part of
// Slice A's TDD paper trail: documents the "before" contract so the new
// resetIntegrators contract is meaningfully discriminating.
void test_stop_clears_pids_calls_drivetrain_stop_and_clears_fault_latch() {
    BalanceConfig hot = makeTestConfig();
    hot.pitchKi = 1.0f;
    hot.rollKi  = 1.0f;
    *cfgBuf = hot;

    // Wind up PIDs.
    BodyVelocity cmd{0.5f, 0.5f, 0.0f};
    IMUReading level = makeIMU(upright());
    for (int i = 0; i < 10; ++i) {
        controller->update(cmd, level, 0.01f);
    }
    TEST_ASSERT_TRUE(std::fabs(drivetrain->lastDrive.vx) > 0.0f);

    // Force fault latch on via the test hook.
    controller->_setFaultForTest(true);
    TEST_ASSERT_TRUE(controller->_faultForTest());

    int driveCountsBeforeStop = drivetrain->driveCallCount;
    controller->stop();

    // Effect 1: drivetrain stop() was called.
    TEST_ASSERT_EQUAL(1, drivetrain->stopCallCount);
    // Effect 2: _inFault latch is cleared.
    TEST_ASSERT_FALSE(controller->_faultForTest());
    // Effect 3: PIDs are reset — a zero-cmd level frame produces zero output
    // (would be non-zero if integrator survived).
    BodyVelocity zero{0.0f, 0.0f, 0.0f};
    controller->update(zero, level, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, drivetrain->lastDrive.vx);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, drivetrain->lastDrive.vy);
    TEST_ASSERT_TRUE(drivetrain->driveCallCount > driveCountsBeforeStop);
}

// B3: re-arm from disarm must not slam wheels from PID windup. resetIntegrators()
// is the smaller-surface alternative to stop(): clears PID integrators but does
// NOT call drivetrain.stop() and does NOT touch the _inFault latch.
void test_resetIntegrators_clears_pid_state_without_stopping_drive() {
    BalanceConfig hot = makeTestConfig();
    hot.pitchKi = 1.0f;
    hot.rollKi  = 1.0f;
    *cfgBuf = hot;

    // Wind up the pitch integrator with a forward command on level platform.
    BodyVelocity cmd{0.5f, 0.0f, 0.0f};
    IMUReading level = makeIMU(upright());
    for (int i = 0; i < 5; ++i) {
        controller->update(cmd, level, 0.01f);
    }
    TEST_ASSERT_TRUE(std::fabs(drivetrain->lastDrive.vx) > 0.0f);

    int stopCountBefore = drivetrain->stopCallCount;
    controller->resetIntegrators();

    // Drivetrain stop() must NOT have been invoked (this is the discriminator
    // vs stop()).
    TEST_ASSERT_EQUAL(stopCountBefore, drivetrain->stopCallCount);

    // Feed an upright zero-command frame; output must be zero (no integral).
    BodyVelocity zero{0.0f, 0.0f, 0.0f};
    controller->update(zero, level, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, drivetrain->lastDrive.vx);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, drivetrain->lastDrive.vy);
}

// Locks in the contract that resetIntegrators() does NOT clear the _inFault
// latch. Direct fault injection via _setFaultForTest avoids coupling to
// quatToBodyGravity (whose sign convention is the deferred B1 question).
void test_resetIntegrators_preserves_fault_latch() {
    controller->_setFaultForTest(true);
    TEST_ASSERT_TRUE(controller->_faultForTest());

    controller->resetIntegrators();

    TEST_ASSERT_TRUE(controller->_faultForTest());
}

// onArmed() is the Disarmed→Armed edge hook. Must reset the wheel-level
// drivetrain PIDs (so D-spike from stale _prevMeasurement can't kick on the
// first armed tick) AND the balance PIDs (defensive; setGains during
// Disarmed could otherwise leave them in a state we'd rather not enter armed
// from). Must NOT call drivetrain.stop() — that would brake motors.
void test_onArmed_resets_drivetrain_pids_and_balance_integrators() {
    BalanceConfig hot = makeTestConfig();
    hot.pitchKi = 1.0f;
    hot.rollKi  = 1.0f;
    *cfgBuf = hot;

    // Wind up the balance integrators with a non-zero command on level.
    BodyVelocity cmd{0.5f, 0.5f, 0.0f};
    IMUReading level = makeIMU(upright());
    for (int i = 0; i < 5; ++i) {
        controller->update(cmd, level, 0.01f);
    }

    int stopCountBefore = drivetrain->stopCallCount;
    controller->onArmed();

    // Wheel PIDs reset, no brake.
    TEST_ASSERT_EQUAL(1, drivetrain->resetPidsCallCount);
    TEST_ASSERT_EQUAL(stopCountBefore, drivetrain->stopCallCount);

    // Balance integrators reset — zero command on level produces ~0 output.
    BodyVelocity zero{0.0f, 0.0f, 0.0f};
    controller->update(zero, level, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, drivetrain->lastDrive.vx);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, drivetrain->lastDrive.vy);
}

// onDisarmed() is the Armed→Disarmed edge hook. Same semantics as the old
// resetIntegrators() call site PLUS a wheel-PID reset so the next Disarmed
// period starts with clean inner state.
void test_onDisarmed_resets_balance_and_drivetrain_pids() {
    BalanceConfig hot = makeTestConfig();
    hot.pitchKi = 1.0f;
    hot.rollKi  = 1.0f;
    *cfgBuf = hot;

    BodyVelocity cmd{0.5f, 0.0f, 0.0f};
    IMUReading level = makeIMU(upright());
    for (int i = 0; i < 5; ++i) {
        controller->update(cmd, level, 0.01f);
    }
    TEST_ASSERT_TRUE(std::fabs(drivetrain->lastDrive.vx) > 0.0f);

    int stopCountBefore = drivetrain->stopCallCount;
    controller->onDisarmed();

    TEST_ASSERT_EQUAL(1, drivetrain->resetPidsCallCount);
    TEST_ASSERT_EQUAL(stopCountBefore, drivetrain->stopCallCount);

    BodyVelocity zero{0.0f, 0.0f, 0.0f};
    controller->update(zero, level, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, drivetrain->lastDrive.vx);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, drivetrain->lastDrive.vy);
}

// ---- telemetry tests (Wave 2, Task #4) ------------------------------------

void test_last_telemetry_populated_after_update() {
    BodyVelocity cmd{0.3f, -0.2f, 0.1f};
    float tilt = 10.0f * 3.14159265f / 180.0f;
    IMUReading imuData = makeIMU(pitchedForward(tilt));
    imuData.linearAccel = Vec3{1.5f, 2.5f, -9.8f};
    imuData.gyro  = Vec3{0.11f, 0.22f, 0.33f};

    controller->update(cmd, imuData, 0.01f);
    const BalanceTelemetry& t = controller->lastTelemetry();

    // Timing.
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.01f, t.dt_used);

    // Raw inputs (controller mirrors cmd into both raw and post-ramp slots).
    TEST_ASSERT_FLOAT_WITHIN(1e-6f,  0.3f, t.cmd_vx);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -0.2f, t.cmd_vy);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f,  0.1f, t.cmd_omega);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f,  0.3f, t.cmd_vx_raw);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -0.2f, t.cmd_vy_raw);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f,  0.1f, t.cmd_omega_raw);

    // Quaternion mirror.
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, imuData.orientation.w, t.quat_w);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, imuData.orientation.x, t.quat_x);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, imuData.orientation.y, t.quat_y);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, imuData.orientation.z, t.quat_z);

    // Accel + raw gyro mirrors.
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 1.5f,  t.accel_x);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 2.5f,  t.accel_y);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, -9.8f, t.accel_z);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.11f, t.gyro_x_raw);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.22f, t.gyro_y_raw);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.33f, t.gyro_z_raw);

    // pitch_actual is the actual pitch angle — non-zero since we tilted.
    TEST_ASSERT_TRUE(std::fabs(t.pitch_actual) > 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, tilt, t.pitch_actual);

    // Live gain mirrors (cfgBuf is the active config).
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, cfgBuf->pitchKp,           t.pitch_Kp);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, cfgBuf->pitchKi,           t.pitch_Ki);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, cfgBuf->pitchKd,           t.pitch_Kd);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, cfgBuf->rollKp,            t.roll_Kp);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, cfgBuf->rollKi,            t.roll_Ki);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, cfgBuf->rollKd,            t.roll_Kd);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, cfgBuf->maxOutputVelocity, t.max_output_velocity);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, cfgBuf->envelopeEnterSin,  t.envelope_enter_sin);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, cfgBuf->envelopeExitSin,   t.envelope_exit_sin);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, cfgBuf->tiltPerVelocity,   t.tilt_per_velocity);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, cfgBuf->maxTiltSetpoint,   t.max_tilt_setpoint);

    // body_*_cmd mirrors the negated drivetrain command.
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, drivetrain->lastDrive.vx,    t.body_vx_cmd);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, drivetrain->lastDrive.vy,    t.body_vy_cmd);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, drivetrain->lastDrive.omega, t.body_omega_cmd);
}

void test_last_telemetry_pitch_pid_terms_match_computation() {
    // Configure simple gains, well within output limits.
    BalanceConfig c = makeTestConfig();
    c.pitchKp = 0.5f;
    c.pitchKi = 0.0f;
    c.pitchKd = 0.0f;
    *cfgBuf = c;

    BodyVelocity cmd{0.0f, 0.0f, 0.0f};
    float tilt = 5.0f * 3.14159265f / 180.0f;
    IMUReading imuData = makeIMU(pitchedForward(tilt));
    controller->update(cmd, imuData, 0.01f);

    const BalanceTelemetry& t = controller->lastTelemetry();
    // pitch_out_raw = P + I + D for an unsaturated tick.
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, t.pitch_P + t.pitch_I + t.pitch_D, t.pitch_out_raw);
    // P term = Kp * err, err = target - actual = 0 - tilt = -tilt.
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.5f * (-tilt), t.pitch_P);
}

void test_fault_enter_bit_set_exactly_once() {
    BodyVelocity cmd{0.0f, 0.0f, 0.0f};
    float bigTilt = 65.0f * 3.14159265f / 180.0f;
    IMUReading bad = makeIMU(pitchedForward(bigTilt));
    controller->update(cmd, bad, 0.01f);
    TEST_ASSERT_TRUE(controller->lastTelemetry().event_flags & kEvent_FAULT_ENTER);
    TEST_ASSERT_EQUAL(1, controller->lastTelemetry().in_fault);

    // Second tick at the same tilt: still in fault, no re-entry edge.
    controller->update(cmd, bad, 0.01f);
    TEST_ASSERT_FALSE(controller->lastTelemetry().event_flags & kEvent_FAULT_ENTER);
    TEST_ASSERT_FALSE(controller->lastTelemetry().event_flags & kEvent_FAULT_EXIT);
    TEST_ASSERT_EQUAL(1, controller->lastTelemetry().in_fault);

    // Drop below exit threshold: FAULT_EXIT bit fires, in_fault clears.
    IMUReading recovered = makeIMU(upright());
    controller->update(cmd, recovered, 0.01f);
    TEST_ASSERT_TRUE(controller->lastTelemetry().event_flags & kEvent_FAULT_EXIT);
    TEST_ASSERT_EQUAL(0, controller->lastTelemetry().in_fault);
}

void test_in_fault_field_reflects_state() {
    BodyVelocity cmd{0.0f, 0.0f, 0.0f};
    IMUReading level = makeIMU(upright());
    controller->update(cmd, level, 0.01f);
    TEST_ASSERT_EQUAL(0, controller->lastTelemetry().in_fault);

    IMUReading bad = makeIMU(pitchedForward(65.0f * 3.14159265f / 180.0f));
    controller->update(cmd, bad, 0.01f);
    TEST_ASSERT_EQUAL(1, controller->lastTelemetry().in_fault);

    controller->update(cmd, level, 0.01f);
    TEST_ASSERT_EQUAL(0, controller->lastTelemetry().in_fault);
}

void test_pitch_out_saturated_bit_propagates() {
    BalanceConfig huge = makeTestConfig();
    huge.pitchKp = 1e6f;
    huge.maxOutputVelocity = 1.0f;
    *cfgBuf = huge;

    BodyVelocity cmd{0.5f, 0.0f, 0.0f};
    IMUReading level = makeIMU(upright());
    controller->update(cmd, level, 0.01f);

    TEST_ASSERT_TRUE(controller->lastTelemetry().event_flags & kEvent_PITCH_OUT_SATURATED);
}

void test_set_tuner_consumes_pending_events() {
    BalanceTuner tuner;
    tuner.begin(*cfgBuf);
    // Reset the controller's atomic-slot to point at the tuner's owned slot:
    // resetToDefaults mutates the tuner's spare and republishes. We assert on
    // the bit, not on the slot identity.
    controller->setTuner(&tuner);
    tuner.resetToDefaults();

    BodyVelocity cmd{0.0f, 0.0f, 0.0f};
    IMUReading level = makeIMU(upright());
    controller->update(cmd, level, 0.01f);
    TEST_ASSERT_TRUE(controller->lastTelemetry().event_flags & kEvent_CONFIG_RESET);

    // Single-consume: next tick must clear the bit.
    controller->update(cmd, level, 0.01f);
    TEST_ASSERT_FALSE(controller->lastTelemetry().event_flags & kEvent_CONFIG_RESET);
}

void test_set_tuner_null_skips_consume() {
    // No setTuner call. Update must not crash; event_flags must be a sane
    // subset (PID + fault only — nothing tuner-driven).
    BodyVelocity cmd{0.0f, 0.0f, 0.0f};
    IMUReading level = makeIMU(upright());
    controller->update(cmd, level, 0.01f);

    TEST_ASSERT_FALSE(controller->lastTelemetry().event_flags & kEvent_CONFIG_RESET);
    TEST_ASSERT_FALSE(controller->lastTelemetry().event_flags & kEvent_GAIN_CHANGED);
    TEST_ASSERT_FALSE(controller->lastTelemetry().event_flags & kEvent_CONFIG_SAVED);
}

void test_stop_resets_pid_state_and_calls_drivetrain_stop() {
    // Warm-up with Ki > 0 so an integral builds up.
    BalanceConfig hot = makeTestConfig();
    hot.pitchKi = 1.0f;
    hot.rollKi  = 1.0f;
    *cfgBuf = hot;

    BodyVelocity cmd{0.5f, 0.5f, 0.0f};
    IMUReading level = makeIMU(upright());
    for (int i = 0; i < 10; ++i) {
        controller->update(cmd, level, 0.01f);
    }

    controller->stop();
    TEST_ASSERT_EQUAL(1, drivetrain->stopCallCount);

    // After stop, a zero-command level-platform update must emit zero (no leftover integral).
    BodyVelocity zero{0.0f, 0.0f, 0.0f};
    controller->update(zero, level, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, drivetrain->lastDrive.vx);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, drivetrain->lastDrive.vy);
}

// ---- yaw-rate inner PID tests (Commit 3) ---------------------------------

void test_yaw_passthrough_when_kp_zero() {
    // Ship-safe regression: cfg.yawRateKp = 0 ⇒ cmd.omega passes through.
    BodyVelocity cmd{0.0f, 0.0f, 1.5f};
    IMUReading imuData = makeIMU(upright());
    controller->update(cmd, imuData, 0.01f);

    TEST_ASSERT_EQUAL(1, drivetrain->driveCallCount);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.5f, drivetrain->lastDrive.omega);
}

void test_yaw_closed_loop_counteracts_spin() {
    // cfg.yawRateKp = 0.5, cmd.omega = 0, gyro.z = +1 rad/s, gyroYawSign = +1.
    // Inner PID drives drivetrain omega NEGATIVE to null measured rate.
    BalanceConfig c = makeTestConfig();
    c.yawRateKp = 0.5f;
    *cfgBuf = c;

    BodyVelocity cmd{0.0f, 0.0f, 0.0f};
    IMUReading imuData = makeIMU(upright());
    imuData.gyro = Vec3{0.0f, 0.0f, 1.0f};
    controller->update(cmd, imuData, 0.01f);

    TEST_ASSERT_TRUE(drivetrain->lastDrive.omega < 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -0.5f, drivetrain->lastDrive.omega);
}

void test_yaw_gyro_sign_inverts_loop() {
    BalanceConfig c = makeTestConfig();
    c.yawRateKp    = 0.5f;
    c.gyroYawSign  = -1.0f;
    *cfgBuf = c;

    BodyVelocity cmd{0.0f, 0.0f, 0.0f};
    IMUReading imuData = makeIMU(upright());
    imuData.gyro = Vec3{0.0f, 0.0f, 1.0f};
    controller->update(cmd, imuData, 0.01f);

    TEST_ASSERT_TRUE(drivetrain->lastDrive.omega > 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.5f, drivetrain->lastDrive.omega);
}

void test_nan_orientation_skips_drive() {
    BodyVelocity cmd{0.1f, 0.2f, 0.3f};
    IMUReading imuData = makeIMU(upright());
    imuData.orientation.w = std::numeric_limits<float>::quiet_NaN();

    int driveCountBefore = drivetrain->driveCallCount;
    controller->update(cmd, imuData, 0.01f);

    TEST_ASSERT_EQUAL(driveCountBefore, drivetrain->driveCallCount);
    TEST_ASSERT_TRUE(controller->lastTelemetry().event_flags & kEvent_IMU_INVALID);
}

void test_nan_gyro_z_skips_drive() {
    BodyVelocity cmd{0.1f, 0.2f, 0.3f};
    IMUReading imuData = makeIMU(upright());
    imuData.gyro = Vec3{0.0f, 0.0f, std::numeric_limits<float>::quiet_NaN()};

    int driveCountBefore = drivetrain->driveCallCount;
    controller->update(cmd, imuData, 0.01f);

    TEST_ASSERT_EQUAL(driveCountBefore, drivetrain->driveCallCount);
    TEST_ASSERT_TRUE(controller->lastTelemetry().event_flags & kEvent_IMU_INVALID);
}

// gyro.x feeds the roll PID as the explicit rate argument; without the
// guard the inner PID's BB8_ASSERT(isfinite(rate)) would abort firmware.
void test_nan_gyro_x_skips_drive() {
    BodyVelocity cmd{0.1f, 0.2f, 0.3f};
    IMUReading imuData = makeIMU(upright());
    imuData.gyro = Vec3{std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f};

    int driveCountBefore = drivetrain->driveCallCount;
    controller->update(cmd, imuData, 0.01f);

    TEST_ASSERT_EQUAL(driveCountBefore, drivetrain->driveCallCount);
    TEST_ASSERT_TRUE(controller->lastTelemetry().event_flags & kEvent_IMU_INVALID);
}

// gyro.y feeds the pitch PID as the explicit rate argument; same story
// as gyro.x — guard or the PID assert kills the firmware.
void test_nan_gyro_y_skips_drive() {
    BodyVelocity cmd{0.1f, 0.2f, 0.3f};
    IMUReading imuData = makeIMU(upright());
    imuData.gyro = Vec3{0.0f, std::numeric_limits<float>::quiet_NaN(), 0.0f};

    int driveCountBefore = drivetrain->driveCallCount;
    controller->update(cmd, imuData, 0.01f);

    TEST_ASSERT_EQUAL(driveCountBefore, drivetrain->driveCallCount);
    TEST_ASSERT_TRUE(controller->lastTelemetry().event_flags & kEvent_IMU_INVALID);
}

void test_yaw_spin_recovery_triggers_after_100ms() {
    BalanceConfig c = makeTestConfig();
    c.yawRateKp = 0.5f;
    *cfgBuf = c;

    BodyVelocity cmd{0.0f, 0.0f, 0.0f};
    IMUReading imuData = makeIMU(upright());
    imuData.gyro = Vec3{0.0f, 0.0f, 15.0f};  // 15 rad/s > 12.566 threshold

    bool recoveryFired = false;
    for (int i = 0; i < 11; ++i) {
        controller->update(cmd, imuData, 0.01f);
        if (controller->lastTelemetry().event_flags & kEvent_YAW_SPIN_RECOVERY) {
            recoveryFired = true;
        }
    }
    TEST_ASSERT_TRUE(recoveryFired);
    // Once spin is active, omega forced to 0.
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, drivetrain->lastDrive.omega);
}

// kEvent_YAW_SPIN_RECOVERY is the rising-edge bit for the spin latch.
// Once _yawSpinActive is true, subsequent ticks at the same sustained
// rate must NOT re-OR the bit — counters would double-count otherwise.
void test_yaw_spin_recovery_fires_exactly_once_across_ticks() {
    BalanceConfig c = makeTestConfig();
    c.yawRateKp = 0.5f;
    *cfgBuf = c;

    BodyVelocity cmd{0.0f, 0.0f, 0.0f};
    IMUReading imuData = makeIMU(upright());
    imuData.gyro = Vec3{0.0f, 0.0f, 15.0f};

    int fireCount = 0;
    int fireTick  = -1;
    for (int i = 0; i < 20; ++i) {
        controller->update(cmd, imuData, 0.01f);
        if (controller->lastTelemetry().event_flags & kEvent_YAW_SPIN_RECOVERY) {
            ++fireCount;
            if (fireTick < 0) fireTick = i;
        }
    }
    TEST_ASSERT_EQUAL_INT(1, fireCount);
    // YAW_SPIN_TRIP_MS=100 with dt=10ms ⇒ trip on the 11th tick (index 10).
    TEST_ASSERT_EQUAL_INT(10, fireTick);
}

void test_yaw_spin_recovery_clears_when_rate_drops() {
    BalanceConfig c = makeTestConfig();
    c.yawRateKp = 0.5f;
    *cfgBuf = c;

    BodyVelocity cmd{0.0f, 0.0f, 0.0f};
    IMUReading spin = makeIMU(upright());
    spin.gyro = Vec3{0.0f, 0.0f, 15.0f};
    for (int i = 0; i < 11; ++i) {
        controller->update(cmd, spin, 0.01f);
    }

    // Drop to quiet: recovery should clear after sustained below-threshold.
    IMUReading quiet = makeIMU(upright());
    quiet.gyro = Vec3{0.0f, 0.0f, 1.0f};
    for (int i = 0; i < 11; ++i) {
        controller->update(cmd, quiet, 0.01f);
    }

    // After recovery clears, the inner PID resumes: cmd.omega=0, gyro.z=+1
    // expected omega_out = -0.5.
    TEST_ASSERT_TRUE(drivetrain->lastDrive.omega < 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, -0.5f, drivetrain->lastDrive.omega);
}

void test_yaw_pid_resets_on_arm_disarm_stop_and_fault() {
    BalanceConfig c = makeTestConfig();
    c.yawRateKp = 0.5f;
    c.yawRateKi = 1.0f;
    *cfgBuf = c;

    BodyVelocity cmd{0.0f, 0.0f, 0.0f};
    IMUReading imuData = makeIMU(upright());
    imuData.gyro = Vec3{0.0f, 0.0f, 1.0f};

    // Wind up yaw integrator.
    auto windUp = [&]() {
        for (int i = 0; i < 5; ++i) {
            controller->update(cmd, imuData, 0.01f);
        }
        TEST_ASSERT_TRUE(std::fabs(controller->lastTelemetry().yaw_rate_I) > 0.0f);
    };

    windUp();
    controller->onArmed();
    IMUReading level = makeIMU(upright());
    controller->update(BodyVelocity{0,0,0}, level, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, controller->lastTelemetry().yaw_rate_I);

    windUp();
    controller->onDisarmed();
    controller->update(BodyVelocity{0,0,0}, level, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, controller->lastTelemetry().yaw_rate_I);

    windUp();
    controller->stop();
    controller->update(BodyVelocity{0,0,0}, level, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, controller->lastTelemetry().yaw_rate_I);

    // Fault entry: tilt past envelope must reset yaw PID too.
    windUp();
    IMUReading bad = makeIMU(pitchedForward(65.0f * 3.14159265f / 180.0f));
    bad.gyro = Vec3{0.0f, 0.0f, 0.0f};
    controller->update(BodyVelocity{0,0,0}, bad, 0.01f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, controller->lastTelemetry().yaw_rate_I);
}

// ---- heading-hold outer loop tests (Commit 4) ----------------------------

// Heading integrator must accumulate ∫gyro.z * dt every tick, regardless of
// stick mode. With cmd.omega = 1.0 (out of deadband) and gyro.z = 0.1 rad/s
// over 100 ticks at dt=0.01s, ∫ ≈ 0.1 rad. Tracks truth even when outer
// loop is bypassed.
void test_heading_integrator_tracks_gyro_z() {
    BalanceConfig c = makeTestConfig();
    c.yawRateKp = 0.5f;
    c.headingKp = 1.0f;
    *cfgBuf = c;

    BodyVelocity cmd{0.0f, 0.0f, 1.0f};  // out of deadband
    IMUReading imuData = makeIMU(upright());
    imuData.gyro = Vec3{0.0f, 0.0f, 0.1f};

    for (int i = 0; i < 100; ++i) {
        controller->update(cmd, imuData, 0.01f);
    }

    TEST_ASSERT_FLOAT_WITHIN(0.005f, 0.1f,
                             controller->lastTelemetry().heading_integrated);
}

// Out-of-deadband: setpoint invalidated (NaN in telemetry), latched false.
// In-deadband: setpoint snapshotted to integrator value, latched true.
void test_stick_out_of_deadband_unlatches_setpoint() {
    BalanceConfig c = makeTestConfig();
    c.headingKp = 1.0f;
    c.yawRateKp = 0.5f;  // engage the inner loop so the outer loop runs
    *cfgBuf = c;

    // First, in-deadband with onArmed reset: setpoint == 0 (latched at arm).
    controller->onArmed();
    BodyVelocity zero{0.0f, 0.0f, 0.0f};
    IMUReading level = makeIMU(upright());
    controller->update(zero, level, 0.01f);
    TEST_ASSERT_FALSE(std::isnan(controller->lastTelemetry().heading_setpoint));

    // Stick out of deadband for several ticks → NaN.
    BodyVelocity drive{0.0f, 0.0f, 1.5f};
    for (int i = 0; i < 5; ++i) {
        controller->update(drive, level, 0.01f);
    }
    TEST_ASSERT_TRUE(std::isnan(controller->lastTelemetry().heading_setpoint));
}

// Returning to deadband after a drive: setpoint re-latches at the current
// integrator value, and kEvent_HEADING_LATCHED fires on the relatch tick.
void test_stick_returns_to_deadband_relatches_at_current_integrator() {
    BalanceConfig c = makeTestConfig();
    c.headingKp = 1.0f;
    c.yawRateKp = 0.5f;  // engage the inner loop so the outer loop runs
    *cfgBuf = c;

    controller->onArmed();
    IMUReading imuData = makeIMU(upright());
    imuData.gyro = Vec3{0.0f, 0.0f, 0.2f};

    // Drive out of deadband while integrator accumulates.
    BodyVelocity drive{0.0f, 0.0f, 1.5f};
    for (int i = 0; i < 10; ++i) {
        controller->update(drive, imuData, 0.01f);
    }
    const float integratorBeforeRelatch =
        controller->lastTelemetry().heading_integrated;
    TEST_ASSERT_TRUE(std::isnan(controller->lastTelemetry().heading_setpoint));

    // Return to deadband: snapshot fires and event bit is set this tick.
    BodyVelocity zero{0.0f, 0.0f, 0.0f};
    controller->update(zero, imuData, 0.01f);
    TEST_ASSERT_FALSE(std::isnan(controller->lastTelemetry().heading_setpoint));
    TEST_ASSERT_FLOAT_WITHIN(
        0.005f, integratorBeforeRelatch + 0.2f * 0.01f,
        controller->lastTelemetry().heading_setpoint);
    TEST_ASSERT_TRUE(controller->lastTelemetry().event_flags &
                     kEvent_HEADING_LATCHED);

    // Subsequent in-deadband tick: bit clears (edge-only).
    controller->update(zero, makeIMU(upright()), 0.01f);
    TEST_ASSERT_FALSE(controller->lastTelemetry().event_flags &
                      kEvent_HEADING_LATCHED);
}

// LP filter step-response check. Out of the heading deadband, omega_clamped
// = cmd.omega — so a constant cmd.omega is a clean unit step into the LP.
// With τ=0.15 s, the LP should reach 1 - exp(-1) ≈ 0.632 of the step after
// one time constant (15 ticks at dt=0.01 s).
void test_lp_filter_reaches_63_percent_after_one_tau() {
    BalanceConfig c = makeTestConfig();
    c.headingKp = 1.0f;
    // Outer loop must be active (passthrough mode bypasses the LP entirely):
    // any non-zero yawRate gain engages it.
    c.yawRateKp = 0.5f;
    c.yawRateKi = 0.0f;
    *cfgBuf = c;

    controller->onArmed();
    BodyVelocity drive{0.0f, 0.0f, 1.0f};  // step, well outside deadband
    IMUReading level = makeIMU(upright());

    float lastOmegaTarget = 0.0f;
    for (int i = 0; i < 15; ++i) {
        controller->update(drive, level, 0.01f);
        lastOmegaTarget = controller->lastTelemetry().omega_target;
    }
    TEST_ASSERT_FLOAT_WITHIN(0.06f, 0.632f, lastOmegaTarget);
}

// Heading P output must be clamped to ±OMEGA_MAX even when Kp * err is huge.
void test_outer_loop_clamped_to_omega_max() {
    BalanceConfig c = makeTestConfig();
    c.headingKp = 100.0f;
    c.yawRateKp = 0.5f;
    *cfgBuf = c;

    controller->onArmed();
    // Force integrator far from setpoint=0 via a single very-high-rate tick
    // before checking the clamp. Easier: directly set integrator by running
    // many ticks at controllable gyro.
    IMUReading drift = makeIMU(upright());
    drift.gyro = Vec3{0.0f, 0.0f, 10.0f};  // 10 rad/s * 0.01 = 0.1 rad/tick
    // After 100 ticks, integrator ≈ 10.0 rad. Then heading_P_raw = 100 * (-10)
    // = -1000, must clamp to -OMEGA_MAX.
    BodyVelocity zero{0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 100; ++i) {
        controller->update(zero, drift, 0.01f);
    }
    // omega_target_raw is the post-clamp value (pre-LP). It should equal
    // -OMEGA_MAX exactly.
    TEST_ASSERT_FLOAT_WITHIN(
        1e-4f, -RobotConstants::OMEGA_MAX,
        controller->lastTelemetry().omega_target_raw);
}

// onArmed() resets integrator, setpoint to 0, latches the hold from tick 1.
void test_onArmed_resets_heading_state() {
    BalanceConfig c = makeTestConfig();
    c.headingKp = 1.0f;
    c.yawRateKp = 0.5f;
    *cfgBuf = c;

    // Run drift first.
    IMUReading drift = makeIMU(upright());
    drift.gyro = Vec3{0.0f, 0.0f, 0.5f};
    BodyVelocity zero{0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 20; ++i) {
        controller->update(zero, drift, 0.01f);
    }
    TEST_ASSERT_TRUE(std::fabs(controller->lastTelemetry().heading_integrated) > 0.01f);

    controller->onArmed();
    IMUReading level = makeIMU(upright());
    controller->update(zero, level, 0.01f);

    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f,
                             controller->lastTelemetry().heading_integrated);
    TEST_ASSERT_FALSE(std::isnan(controller->lastTelemetry().heading_setpoint));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f,
                             controller->lastTelemetry().heading_setpoint);
}

// Tilt-fault entry must NOT zero _headingIntegrator (it must keep tracking
// truth so when fault clears, the held heading is still accurate).
// _omegaTargetFiltered IS reset on fault entry.
void test_tilt_fault_preserves_heading_integrator() {
    BalanceConfig c = makeTestConfig();
    c.headingKp = 1.0f;
    c.yawRateKp = 0.5f;
    *cfgBuf = c;

    controller->onArmed();
    // Build integrator to a known value.
    IMUReading drift = makeIMU(upright());
    drift.gyro = Vec3{0.0f, 0.0f, 0.5f};
    BodyVelocity zero{0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 100; ++i) {
        controller->update(zero, drift, 0.01f);
    }
    const float integratorBeforeFault =
        controller->lastTelemetry().heading_integrated;
    TEST_ASSERT_TRUE(std::fabs(integratorBeforeFault) > 0.4f);

    // Trip fault at 65° (gyro stays zeroed to isolate integrator behavior).
    IMUReading bad = makeIMU(pitchedForward(65.0f * 3.14159265f / 180.0f));
    bad.gyro = Vec3{0.0f, 0.0f, 0.0f};
    controller->update(zero, bad, 0.01f);

    // Integrator unchanged (gyro.z=0 in fault tick, plus no reset).
    TEST_ASSERT_FLOAT_WITHIN(
        0.01f, integratorBeforeFault,
        controller->lastTelemetry().heading_integrated);
}

// Final-pass review fix: tilt-fault early-return zeros _lastTelemetry then
// only repopulates a subset of fields. Heading state (setpoint/err/P) and
// omega_target_* must be mirrored so an operator inspecting telemetry during
// a fault sees the still-held cascade state, not zeros/NaN that would
// falsely suggest heading hold was lost.
void test_tilt_fault_preserves_heading_telemetry() {
    BalanceConfig c = makeTestConfig();
    c.headingKp = 1.0f;
    c.yawRateKp = 0.5f;
    *cfgBuf = c;

    controller->onArmed();
    // Integrate heading to 0.3 rad over 100 ticks (gyro.z=0.3 * dt=0.01).
    IMUReading drift = makeIMU(upright());
    drift.gyro = Vec3{0.0f, 0.0f, 0.3f};
    BodyVelocity zero{0.0f, 0.0f, 0.0f};
    for (int i = 0; i < 100; ++i) {
        controller->update(zero, drift, 0.01f);
    }
    // Confirm latched at 0 (setpoint snapshot taken on first armed tick when
    // stick is in deadband) and integrator has tracked to ~0.3.
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f,
                             controller->lastTelemetry().heading_setpoint);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.3f,
                             controller->lastTelemetry().heading_integrated);

    // Trip fault at 65°; gyro.z=0 so integrator stays put.
    IMUReading bad = makeIMU(pitchedForward(65.0f * 3.14159265f / 180.0f));
    bad.gyro = Vec3{0.0f, 0.0f, 0.0f};
    controller->update(zero, bad, 0.01f);

    TEST_ASSERT_EQUAL(1, controller->lastTelemetry().in_fault);
    // Critical: heading_setpoint must still read 0.0 (NOT NaN — latch
    // survives fault), and heading_err must read -0.3 (the still-held error).
    TEST_ASSERT_FALSE(std::isnan(controller->lastTelemetry().heading_setpoint));
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f,
                             controller->lastTelemetry().heading_setpoint);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -0.3f,
                             controller->lastTelemetry().heading_err);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -0.3f,
                             controller->lastTelemetry().heading_P);
    // omega_target_* during fault: drivetrain gets zeros, and
    // _omegaTargetFiltered was reset to 0 on fault entry.
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f,
                             controller->lastTelemetry().omega_target_raw);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f,
                             controller->lastTelemetry().omega_target);
}

// Passthrough (yawRateKp=0) currently leaves omega_target_* zero in telemetry
// because the closed-loop branch is the only one writing them. Operator
// reading telemetry while in passthrough should see cmd.omega mirrored.
void test_passthrough_mirrors_cmd_omega_to_telemetry() {
    BalanceConfig c = makeTestConfig();
    c.yawRateKp = 0.0f;
    c.yawRateKi = 0.0f;
    *cfgBuf = c;

    BodyVelocity cmd{0.0f, 0.0f, 1.5f};
    IMUReading imuData = makeIMU(upright());
    controller->update(cmd, imuData, 0.01f);

    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.5f,
                             controller->lastTelemetry().omega_target);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.5f,
                             controller->lastTelemetry().omega_target_raw);
}

// Yaw-spin recovery clamps the commanded omega to zero. Locks the contract
// that this is reflected in telemetry explicitly (not via default-init).
void test_yaw_spin_zeros_omega_target_explicitly() {
    BalanceConfig c = makeTestConfig();
    c.yawRateKp = 0.5f;
    *cfgBuf = c;

    BodyVelocity cmd{0.0f, 0.0f, 0.0f};
    IMUReading imuData = makeIMU(upright());
    imuData.gyro = Vec3{0.0f, 0.0f, 15.0f};

    bool recoveryFired = false;
    for (int i = 0; i < 11; ++i) {
        controller->update(cmd, imuData, 0.01f);
        if (controller->lastTelemetry().event_flags & kEvent_YAW_SPIN_RECOVERY) {
            recoveryFired = true;
        }
    }
    TEST_ASSERT_TRUE(recoveryFired);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f,
                             controller->lastTelemetry().omega_target);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f,
                             controller->lastTelemetry().omega_target_raw);
}

int main() {
    UNITY_BEGIN();
    RUN_TEST(test_zero_command_level_platform_emits_zero);
    RUN_TEST(test_zero_command_pitched_forward_drives_shell_backward);
    RUN_TEST(test_zero_command_rolled_left_drives_shell_right);
    RUN_TEST(test_forward_command_level_emits_negative_vx);
    RUN_TEST(test_forward_command_at_target_tilt_settles_to_zero);
    RUN_TEST(test_enter_fault_above_enter_envelope);
    RUN_TEST(test_in_fault_between_thresholds_stays_in_fault);
    RUN_TEST(test_exit_fault_below_exit_envelope);
    RUN_TEST(test_output_clamp_at_max_velocity);
    RUN_TEST(test_stop_clears_pids_calls_drivetrain_stop_and_clears_fault_latch);
    RUN_TEST(test_resetIntegrators_clears_pid_state_without_stopping_drive);
    RUN_TEST(test_resetIntegrators_preserves_fault_latch);
    RUN_TEST(test_onArmed_resets_drivetrain_pids_and_balance_integrators);
    RUN_TEST(test_onDisarmed_resets_balance_and_drivetrain_pids);
    RUN_TEST(test_last_telemetry_populated_after_update);
    RUN_TEST(test_last_telemetry_pitch_pid_terms_match_computation);
    RUN_TEST(test_fault_enter_bit_set_exactly_once);
    RUN_TEST(test_in_fault_field_reflects_state);
    RUN_TEST(test_pitch_out_saturated_bit_propagates);
    RUN_TEST(test_set_tuner_consumes_pending_events);
    RUN_TEST(test_set_tuner_null_skips_consume);
    RUN_TEST(test_stop_resets_pid_state_and_calls_drivetrain_stop);
    RUN_TEST(test_yaw_passthrough_when_kp_zero);
    RUN_TEST(test_yaw_closed_loop_counteracts_spin);
    RUN_TEST(test_yaw_gyro_sign_inverts_loop);
    RUN_TEST(test_nan_orientation_skips_drive);
    RUN_TEST(test_nan_gyro_z_skips_drive);
    RUN_TEST(test_nan_gyro_x_skips_drive);
    RUN_TEST(test_nan_gyro_y_skips_drive);
    RUN_TEST(test_yaw_spin_recovery_triggers_after_100ms);
    RUN_TEST(test_yaw_spin_recovery_fires_exactly_once_across_ticks);
    RUN_TEST(test_yaw_spin_recovery_clears_when_rate_drops);
    RUN_TEST(test_yaw_pid_resets_on_arm_disarm_stop_and_fault);
    RUN_TEST(test_heading_integrator_tracks_gyro_z);
    RUN_TEST(test_stick_out_of_deadband_unlatches_setpoint);
    RUN_TEST(test_stick_returns_to_deadband_relatches_at_current_integrator);
    RUN_TEST(test_lp_filter_reaches_63_percent_after_one_tau);
    RUN_TEST(test_outer_loop_clamped_to_omega_max);
    RUN_TEST(test_onArmed_resets_heading_state);
    RUN_TEST(test_tilt_fault_preserves_heading_integrator);
    RUN_TEST(test_tilt_fault_preserves_heading_telemetry);
    RUN_TEST(test_passthrough_mirrors_cmd_omega_to_telemetry);
    RUN_TEST(test_yaw_spin_zeros_omega_target_explicitly);
    return UNITY_END();
}
