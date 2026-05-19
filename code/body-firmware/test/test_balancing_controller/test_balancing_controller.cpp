/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 */

#include <unity.h>
#include <atomic>
#include <cmath>

#include "BalancingDrivetrainController.h"
#include "BalancingDrivetrainController.cpp"
#include "BalanceConfig.h"
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

    BodyVelocity lastDrive{};
    int driveCallCount = 0;
    int stopCallCount = 0;

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

void test_zero_command_pitched_forward_drives_backward() {
    BodyVelocity cmd{0.0f, 0.0f, 0.0f};
    float tilt = 10.0f * 3.14159265f / 180.0f;
    IMUReading imuData = makeIMU(pitchedForward(tilt));
    controller->update(cmd, imuData, 0.01f);

    TEST_ASSERT_EQUAL(1, drivetrain->driveCallCount);
    TEST_ASSERT_TRUE(drivetrain->lastDrive.vx < 0.0f);
}

void test_zero_command_rolled_left_drives_right() {
    // "Drives right" means vy_out < 0 (vy positive = left in BodyVelocity).
    BodyVelocity cmd{0.0f, 0.0f, 0.0f};
    float tilt = 10.0f * 3.14159265f / 180.0f;
    IMUReading imuData = makeIMU(rolledLeft(tilt));
    controller->update(cmd, imuData, 0.01f);

    TEST_ASSERT_EQUAL(1, drivetrain->driveCallCount);
    TEST_ASSERT_TRUE(drivetrain->lastDrive.vy < 0.0f);
}

void test_forward_command_level_emits_forward_vx_only() {
    BodyVelocity cmd{0.5f, 0.0f, 0.0f};
    IMUReading imuData = makeIMU(upright());

    // Several iterations to overcome any first-call derivative quirks.
    for (int i = 0; i < 3; ++i) {
        controller->update(cmd, imuData, 0.01f);
    }

    TEST_ASSERT_TRUE(drivetrain->lastDrive.vx > 0.0f);
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

    TEST_ASSERT_TRUE(std::fabs(drivetrain->lastDrive.vx) <= 1.0f + 1e-6f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.0f, drivetrain->lastDrive.vx);
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
    RUN_TEST(test_zero_command_pitched_forward_drives_backward);
    RUN_TEST(test_zero_command_rolled_left_drives_right);
    RUN_TEST(test_forward_command_level_emits_forward_vx_only);
    RUN_TEST(test_forward_command_at_target_tilt_settles_to_zero);
    RUN_TEST(test_enter_fault_above_enter_envelope);
    RUN_TEST(test_in_fault_between_thresholds_stays_in_fault);
    RUN_TEST(test_exit_fault_below_exit_envelope);
    RUN_TEST(test_output_clamp_at_max_velocity);
    RUN_TEST(test_stop_clears_pids_calls_drivetrain_stop_and_clears_fault_latch);
    RUN_TEST(test_resetIntegrators_clears_pid_state_without_stopping_drive);
    RUN_TEST(test_resetIntegrators_preserves_fault_latch);
    RUN_TEST(test_stop_resets_pid_state_and_calls_drivetrain_stop);
    return UNITY_END();
}
