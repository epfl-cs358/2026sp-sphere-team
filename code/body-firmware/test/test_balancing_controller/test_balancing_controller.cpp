/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 */

#include <unity.h>
#include <atomic>
#include <cmath>

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
        .maxOutputVelocity = 1.0f,
        .envelopeEnterSin  = std::sin(60.0f * 3.14159265f / 180.0f),
        .envelopeExitSin   = std::sin(55.0f * 3.14159265f / 180.0f),
        .gyroPitchSign     = 1.0f,
        .gyroRollSign      = 1.0f,
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
    return UNITY_END();
}
