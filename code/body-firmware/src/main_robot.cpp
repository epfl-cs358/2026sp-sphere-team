/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini, with assistance from Claude (Anthropic)
 *
 * End-to-end teleop: WebSocket producer (Core 0) → CommandLatch →
 * 100Hz control task (Core 1) → BalancingDrivetrainController → OmniDrivetrain.
 * Dead-window policy: instant stop on producer disconnect, linear ramp during
 * 200ms staleness window, hard stop after.
 */

#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_task_wdt.h>

#include "BodyVelocity.h"
#include "OmniDrivetrain.h"
#include "BNO055IMU.h"
#include "L298NDriver.h"
#include "FIT0186Motor.h"
#include "PID.h"
#include "RobotConstants.h"
#include "pins.h"

#include "sync/CommandLatch.h"
#include "WebSocketCommandProducer.h"
#include "BalancingDrivetrainController.h"
#include "BalanceTuner.h"
#include "BalanceConfigStorage.h"
#include "ArmingState.h"
#include "BalanceHttpApi.h"
#include "ArmingHttpApi.h"
#include "BalanceTelemetry.h"
#include "BalanceTelemetryWs.h"
#include "BalanceTelemetryHttpApi.h"
#include "RemoteSerial.h"

#include "BringUp.h"
OTA_SAFE_MODE_FOR("bb8-robot");

namespace {

using RobotConstants::STALENESS_TIMEOUT_MS;
using RobotConstants::CONTROL_PERIOD_MS;

constexpr uint16_t WS_PORT                  = 80;
constexpr uint32_t TWDT_TIMEOUT_S           = 1;     // tighter than Arduino default ~5s
constexpr uint32_t LOOP_TICK_MS             = 100;   // 10 Hz housekeeping
constexpr uint32_t CAL_PRINT_PERIOD_MS      = 500;
constexpr uint32_t CAL_POLL_PERIOD_MS       = 50;

constexpr uint32_t CONTROL_TASK_STACK_BYTES = 8192;
constexpr UBaseType_t CONTROL_TASK_PRIORITY = 4;
constexpr BaseType_t  CONTROL_TASK_CORE     = 1;

constexpr int I2C_SDA = 21;
constexpr int I2C_SCL = 22;

DrivetrainConfig g_config = RobotConstants::drivetrainConfig();

L298NDriver g_driver0(MOTOR0_PINS);
L298NDriver g_driver1(MOTOR1_PINS);
L298NDriver g_driver2(MOTOR2_PINS);

FIT0186Motor<5> g_motor0(g_driver0, MOTOR0_ENCODER);
FIT0186Motor<5> g_motor1(g_driver1, MOTOR1_ENCODER);
FIT0186Motor<5> g_motor2(g_driver2, MOTOR2_ENCODER);

PID g_pid0(0.005f, 0.002f, 0.0005f, -1.0f, 1.0f, 5.0f);
PID g_pid1(0.005f, 0.002f, 0.0005f, -1.0f, 1.0f, 5.0f);
PID g_pid2(0.005f, 0.002f, 0.0005f, -1.0f, 1.0f, 5.0f);

OmniDrivetrain g_drivetrain(g_motor0, g_motor1, g_motor2, g_config, g_pid0, g_pid1, g_pid2);

BNO055IMU g_imu(0x28, &Wire,
                IMUField::Quaternion | IMUField::Euler |
                IMUField::Gyro | IMUField::Calibration);

// Constructed in setup() after Wire/IMU are up, so FreeRTOS objects
// (queue inside the latch, server/task inside the producer) are created
// in a known-good runtime context rather than during C++ static init.
CommandLatch<BodyVelocity>*      g_latch      = nullptr;
WebSocketCommandProducer*        g_producer   = nullptr;
BalancingDrivetrainController*   g_controller = nullptr;
BalanceTuner                     g_tuner;

const char* armingStateName(ArmingState::State s) {
    switch (s) {
        case ArmingState::State::Disarmed: return "Disarmed";
        case ArmingState::State::Armed:    return "Armed";
        case ArmingState::State::Killed:   return "Killed";
    }
    return "?";
}

// RemoteSerial verb dispatcher: arm / disarm / kill / clearkill / armstate.
// Whitespace-trimmed; unknown verbs echo back to the browser. Mirrors the
// WebSocket producer's control-frame routing for bench operation without UI.
//
// arm/disarm/kill produce no echo here — ArmingState::* logs the transition
// itself via arming_log() (Serial + RemoteSerial), so a second print would
// duplicate. armstate is a pure query and clearkill from non-Killed is a
// FSM no-op, so both still echo the current state explicitly.
void handleRemoteSerialLine(const String& line) {
    String verb = line;
    verb.trim();
    if (verb == "arm") {
        ArmingState::arm();
    } else if (verb == "disarm") {
        ArmingState::disarm();
    } else if (verb == "kill") {
        ArmingState::kill();
    } else if (verb == "clearkill") {
        ArmingState::clearKill();
        RemoteSerial::printf("[arming] %s\n", armingStateName(ArmingState::get()));
    } else if (verb == "armstate") {
        RemoteSerial::printf("[arming] %s\n", armingStateName(ArmingState::get()));
    } else {
        // Fall through to the balance tuner: it consumes lines that begin with
        // `balance ` and silently ignores anything else, so truly-unknown verbs
        // become no-ops here.
        g_tuner.handle(line);
    }
}

// Block until the BNO055 is calibrated enough to fuse a stable orientation.
// Producer and control task are deliberately left un-started by the caller so
// no commands flow and no PWM is written while we wait. OtaSafeMode must be
// initialized first so the robot stays reflashable while gated here.
//
// IMUPLUS mode: isCalibrated() = (gyro == 3 && accel == 3). Mag/sys ignored.
//
// Two paths:
//   - Warm boot (NVS hit, offsets restored): chip's cal counters lag the
//     offsets by ~10–20s but the fusion is already good. Only wait for gyro
//     to settle — sub-second on a stationary chip.
//   - Cold boot (no NVS): operator must do 6 stable accel orientations,
//     ~10s each. Total ~60s with a deliberate cube fixture.
void waitForCalibration() {
    if (g_imu.wasRestored()) {
        RemoteSerial::println("[cal] warm boot — waiting for gyro to settle.");
        while (g_imu.read().calibration.gyro < 3) {
            BringUp::tick();
            delay(CAL_POLL_PERIOD_MS);
        }
        RemoteSerial::println("[cal] gyro settled — arming teleop.");
        return;
    }

    RemoteSerial::println("[cal] cold boot — full IMUPLUS calibration required.");
    RemoteSerial::println("[cal]   gyro:  hold still");
    RemoteSerial::println("[cal]   accel: 6 distinct stable orientations, hold each ~10s");
    RemoteSerial::println("[cal]   (mag and sys are ignored in IMUPLUS — stay at 0 forever)");

    uint32_t last_print = 0;
    while (!g_imu.isCalibrated()) {
        BringUp::tick();

        // read() refreshes the calibration field and, once fully calibrated,
        // persists offsets to flash via the IMU's saved-this-boot latch.
        IMUReading r = g_imu.read();

        const uint32_t now = millis();
        if (now - last_print >= CAL_PRINT_PERIOD_MS) {
            RemoteSerial::printf("[cal] sys=%u gyro=%u accel=%u mag=%u\n",
                                 r.calibration.sys, r.calibration.gyro,
                                 r.calibration.accel, r.calibration.mag);
            last_print = now;
        }

        delay(CAL_POLL_PERIOD_MS);
    }
    RemoteSerial::println("[cal] fully calibrated — offsets saved, arming teleop.");
}

void controlTask(void* /*arg*/) {
    // Subscribe to the Task Watchdog. If this loop hangs (e.g. I2C bus
    // stretch on the IMU), TWDT panics and resets — preferable to motors
    // stuck at last PWM with no operator recovery.
    esp_task_wdt_add(NULL);

    BodyVelocity last_known{};
    uint32_t     last_fresh_ms = 0;
    bool         have_seen_fresh = false;
    ArmingState::State prev_arming = ArmingState::get();

    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(CONTROL_PERIOD_MS);

    UBaseType_t stack_hwm_min = static_cast<UBaseType_t>(-1);
    uint32_t    hwm_iter = 0;

    static uint32_t prev_us = 0;
    static float    dt_min = 1e9f;
    static float    dt_max = 0.0f;
    static float    dt_sum = 0.0f;
    static uint16_t dt_n   = 0;
    static uint32_t telem_seq = 0;

    while (true) {
        // 1) Pull freshest command from latch.
        auto fresh = g_latch->read();
        const uint32_t now = millis();
        if (fresh.has_value()) {
            last_known      = *fresh;
            last_fresh_ms   = now;
            have_seen_fresh = true;
        }

        // 2) Arming-edge dispatch. Runs BEFORE the staleness ramp so the
        //    Armed-edge zeroing of last_known prevents a stale-replay on the
        //    first armed tick.
        //      Disarmed->Armed: seed last_fresh_ms so the velocity ramp
        //                       starts coherent on the first armed tick,
        //                       mark have_seen_fresh, clear last_known so
        //                       the ramp can't replay a prior cycle's
        //                       command. Then controller->onArmed() clears
        //                       wheel-level + balance PID state that drifted
        //                       while update(dt) ran during Disarmed.
        //      Armed->Disarmed: onDisarmed() clears wheel-level + balance
        //                       PID state so re-arm doesn't slam wheels (B3).
        //      *->Killed:       single hard brake on entry; subsequent ticks
        //                       are no-ops (avoids re-clearing _inFault every
        //                       tick which would defeat the Schmitt gate).
        const auto arming = ArmingState::get();
        if (arming != prev_arming) {
            if (arming == ArmingState::State::Armed) {
                last_fresh_ms   = now;
                have_seen_fresh = true;
                last_known      = BodyVelocity{0.0f, 0.0f, 0.0f};
                g_controller->onArmed();
            } else if (prev_arming == ArmingState::State::Armed &&
                       arming      == ArmingState::State::Disarmed) {
                g_controller->onDisarmed();
            } else if (arming == ArmingState::State::Killed) {
                g_controller->stop();
            }
        }

        // 3) Drive command via staleness alone. The producer's connected()
        //    flag can lie on half-open TCP; age < 200ms is the actual safety
        //    net so we don't gate on connected() anymore.
        BodyVelocity drive_cmd{0.0f, 0.0f, 0.0f};
        if (have_seen_fresh) {
            const uint32_t age = now - last_fresh_ms;
            if (age < STALENESS_TIMEOUT_MS) {
                const float scale = 1.0f - static_cast<float>(age) /
                                            static_cast<float>(STALENESS_TIMEOUT_MS);
                drive_cmd = { last_known.vx * scale,
                              last_known.vy * scale,
                              last_known.omega * scale };
            }
        }

        const uint32_t now_us = micros();
        const float dt_measured = (prev_us == 0)
            ? (CONTROL_PERIOD_MS / 1000.0f)
            : (now_us - prev_us) * 1e-6f;
        prev_us = now_us;

        if (dt_n < 100) {
            if (dt_measured < dt_min) dt_min = dt_measured;
            if (dt_measured > dt_max) dt_max = dt_measured;
            dt_sum += dt_measured;
            dt_n++;
            if (dt_n == 100) {
                const float mean = dt_sum / dt_n;
                RemoteSerial::printf(
                    "[loop] dt min=%.3fms mean=%.3fms max=%.3fms jitter=%.3fms\n",
                    dt_min * 1000, mean * 1000, dt_max * 1000,
                    (dt_max - dt_min) * 1000);
                dt_min = 1e9f; dt_max = 0.0f; dt_sum = 0.0f; dt_n = 0;
            }
        }

        // 4) Arming-gated dispatch. Killed brakes immediately (edge already
        //    called stop() once); Armed runs the controller against the
        //    (possibly ramped-to-zero) drive_cmd; Disarmed hard-zeros and
        //    lets the per-wheel PIDs hold motors at zero. OTA composes by
        //    driving ArmingState::disarm() from OtaSafeMode::onStart().
        //    Short-term producer silence (>200ms) ramps drive_cmd to zero
        //    via STALENESS_TIMEOUT_MS in step 3; the controller stays armed.
        const float dt = static_cast<float>(CONTROL_PERIOD_MS) / 1000.0f;
        if (arming == ArmingState::State::Killed) {
            drive_cmd = {0.0f, 0.0f, 0.0f};
        } else if (arming == ArmingState::State::Armed) {
            IMUReading imu_reading = g_imu.read();
            g_controller->update(drive_cmd, imu_reading, dt);
        } else {
            drive_cmd = {0.0f, 0.0f, 0.0f};
            g_drivetrain.drive(drive_cmd);
        }
        g_drivetrain.update(dt);

        BalanceTelemetry t{};
        if (arming == ArmingState::State::Armed) {
            t = g_controller->lastTelemetry();
        } else {
            // Read raw IMU even when disarmed — operator wants to verify before arming.
            IMUReading imu_reading = g_imu.read();
            t.quat_w = imu_reading.orientation.w;
            t.quat_x = imu_reading.orientation.x;
            t.quat_y = imu_reading.orientation.y;
            t.quat_z = imu_reading.orientation.z;
            t.accel_x = imu_reading.linearAccel.x;
            t.accel_y = imu_reading.linearAccel.y;
            t.accel_z = imu_reading.linearAccel.z;
            t.gyro_x_raw = imu_reading.gyro.x;
            t.gyro_y_raw = imu_reading.gyro.y;
            t.gyro_z_raw = imu_reading.gyro.z;
            const BalanceConfig snap = g_tuner.snapshot();
            t.pitch_Kp = snap.pitchKp; t.pitch_Ki = snap.pitchKi; t.pitch_Kd = snap.pitchKd;
            t.roll_Kp  = snap.rollKp;  t.roll_Ki  = snap.rollKi;  t.roll_Kd  = snap.rollKd;
            t.yaw_rate_Kp = snap.yawRateKp;
            t.yaw_rate_Ki = snap.yawRateKi;
            t.heading_Kp  = snap.headingKp;
            t.pitch_deadband = snap.pitchDeadband;
            t.roll_deadband  = snap.rollDeadband;
            t.max_output_velocity = snap.maxOutputVelocity;
            t.envelope_enter_sin  = snap.envelopeEnterSin;
            t.envelope_exit_sin   = snap.envelopeExitSin;
            t.gyro_pitch_sign     = snap.gyroPitchSign;
            t.gyro_roll_sign      = snap.gyroRollSign;
            t.gyro_yaw_sign       = snap.gyroYawSign;
            t.tilt_per_velocity   = snap.tiltPerVelocity;
            t.max_tilt_setpoint   = snap.maxTiltSetpoint;
        }

        t.seq = telem_seq++;
        t.t_us = now_us;
        t.dt_measured = dt_measured;
        t.dt_used     = dt;
        t.cmd_vx_raw = last_known.vx;
        t.cmd_vy_raw = last_known.vy;
        t.cmd_omega_raw = last_known.omega;
        t.cmd_vx = drive_cmd.vx;
        t.cmd_vy = drive_cmd.vy;
        t.cmd_omega = drive_cmd.omega;
        t.cmd_age_ms = have_seen_fresh ? (now - last_fresh_ms) : 0;
        t.cmd_stale  = (have_seen_fresh &&
                        (now - last_fresh_ms) >= STALENESS_TIMEOUT_MS) ? 1 : 0;
        t.armed_state = (arming == ArmingState::State::Armed)  ? 1
                      : (arming == ArmingState::State::Killed) ? 2 : 0;

        {
            const auto wt = g_drivetrain.getWheelTelemetry();
            for (int i = 0; i < 3; ++i) {
                t.wheel_target_rpm[i] = wt[i].target_rpm;
                t.wheel_meas_rpm[i]   = wt[i].meas_rpm;
                t.wheel_P[i] = wt[i].P;
                t.wheel_I[i] = wt[i].I;
                t.wheel_D[i] = wt[i].D;
                t.wheel_out[i] = wt[i].out;
            }
        }

        if (arming != prev_arming) {
            if (arming == ArmingState::State::Armed) t.event_flags |= kEvent_ARMED_EDGE;
            if (prev_arming == ArmingState::State::Armed &&
                arming      == ArmingState::State::Disarmed) {
                t.event_flags |= kEvent_DISARMED_EDGE;
            }
            if (arming == ArmingState::State::Killed) t.event_flags |= kEvent_KILLED_EDGE;
            if (prev_arming == ArmingState::State::Killed &&
                arming      != ArmingState::State::Killed) {
                t.event_flags |= kEvent_KILL_CLEARED;
            }
        }

        BalanceTelemetryWs::publish(t);

        prev_arming = arming;

        esp_task_wdt_reset();

        if (hwm_iter < 100) {
            UBaseType_t hwm = uxTaskGetStackHighWaterMark(NULL);
            if (hwm < stack_hwm_min) stack_hwm_min = hwm;
            if (++hwm_iter == 100) {
                Serial.printf("[ctrl] stack HWM min over 100 iters: %u words free\n",
                              stack_hwm_min);
            }
        }

        vTaskDelayUntil(&lastWake, period);
    }
}

}  // namespace

void setup() {
    // Network/OTA FIRST. If anything below this fails, the chip stays
    // reachable via STA (bb8-robot.local) or the always-on recovery
    // SoftAP (bb8-robot-recovery at 192.168.4.1).
    BringUp::begin();
    ArmingState::begin();
    RemoteSerial::onMessage(handleRemoteSerialLine);
    RemoteSerial::println("BB-8 main_robot starting.");
    RemoteSerial::println("[arming] boot state: Disarmed");

    Wire.begin(I2C_SDA, I2C_SCL);

    g_motor0.begin();
    g_motor1.begin();
    g_motor2.begin();

    if (!g_imu.begin()) {
        // Degraded mode: skip producer/controller/control-task creation and
        // return. loop() keeps calling OtaSafeMode::tick() so the chip stays
        // reachable for an OTA reflash that fixes the wiring or driver.
        RemoteSerial::println("[imu] init failed — entering OTA-only degraded mode.");
        return;
    }

    g_latch      = new CommandLatch<BodyVelocity>();
    g_producer   = new WebSocketCommandProducer(*g_latch, WS_PORT);

    BalanceConfig boot_cfg;
    if (!BalanceConfigStorage::load(boot_cfg)) {
        boot_cfg = RobotConstants::balanceConfig();
        RemoteSerial::println("[balance] no persisted config; using defaults");
    }
    g_tuner.begin(boot_cfg);
    BalanceHttpApi::registerRoutes(RemoteSerial::server(), g_tuner);
    ArmingHttpApi::registerRoutes(RemoteSerial::server());
    BalanceTelemetryWs::init(RemoteSerial::server());
    BalanceTelemetryHttpApi::registerRoutes(RemoteSerial::server());
    g_controller = new BalancingDrivetrainController(g_drivetrain, g_imu, g_tuner.slot());
    g_controller->setTuner(&g_tuner);

    // Gate: no producer, no control task, no PWM until the IMU is fully
    // calibrated. Motors stay at PWM=0 from begin() throughout this wait.
    // Build with -DSKIP_IMU_CAL to bypass for bench testing.
#ifndef SKIP_IMU_CAL
    waitForCalibration();
#else
    RemoteSerial::println("[cal] SKIP_IMU_CAL set — arming teleop without calibration.");
#endif

    g_producer->start();

    // Tighten the global Task Watchdog timeout. Affects IDLE tasks too, but
    // 1s is comfortably above their normal slack.
    esp_task_wdt_init(TWDT_TIMEOUT_S, true);

    xTaskCreatePinnedToCore(
        &controlTask,
        "control",
        CONTROL_TASK_STACK_BYTES,
        nullptr,
        CONTROL_TASK_PRIORITY,
        nullptr,
        CONTROL_TASK_CORE);

    RemoteSerial::println("BB-8 ready.");
}

void loop() {
    BringUp::tick();
    vTaskDelay(pdMS_TO_TICKS(LOOP_TICK_MS));
}
