/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini, with assistance from Claude (Anthropic)
 *
 * Balance-controller bench harness. Mirrors main_robot.cpp but strips the
 * WebSocket producer and instead accepts velocity commands over RemoteSerial:
 *   cmd <vx> <vy> <omega>
 * Plus arming verbs and BalanceTuner `balance ...` commands. The same
 * CommandLatch is kept because the RemoteSerial AsyncTCP task (Core 0)
 * writes and the control task (Core 1) reads — cross-core float assignment
 * is not atomic on Xtensa LX6.
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
#include "BalancingDrivetrainController.h"
#include "BalanceTuner.h"
#include "BalanceConfig.h"
#include "BalanceConfigStorage.h"
#include "ArmingState.h"
#include "BalanceHttpApi.h"
#include "ArmingHttpApi.h"
#include "RemoteSerial.h"

#include "BringUp.h"
OTA_SAFE_MODE_FOR("bb8-balance-test");

namespace {

using RobotConstants::STALENESS_TIMEOUT_MS;
using RobotConstants::CONTROL_PERIOD_MS;

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

BalanceTuner g_tuner;

// Constructed in setup() after Wire/IMU are up, so FreeRTOS objects
// (queue inside the latch) are created in a known-good runtime context
// rather than during C++ static init.
CommandLatch<BodyVelocity>*      g_remote_latch = nullptr;
BalancingDrivetrainController*   g_controller   = nullptr;

const char* armingStateName(ArmingState::State s) {
    switch (s) {
        case ArmingState::State::Disarmed: return "Disarmed";
        case ArmingState::State::Armed:    return "Armed";
        case ArmingState::State::Killed:   return "Killed";
    }
    return "?";
}

// RemoteSerial verb dispatcher. Trims input, then:
//   1. `cmd <vx> <vy> <omega>` → write to g_remote_latch (Core 0 → Core 1).
//   2. arming verbs (arm/disarm/kill/clearkill/armstate) → ArmingState.
//   3. fallthrough → BalanceTuner::handle (ignores lines without `balance `).
void handleLine(const String& line) {
    String trimmed = line;
    trimmed.trim();

    if (trimmed.startsWith("cmd ") || trimmed == "cmd") {
        // Token by space: cmd vx vy omega.
        const char* s = trimmed.c_str() + 3;
        char* end = nullptr;
        float vx = strtof(s, &end);
        if (end == s) {
            RemoteSerial::println("[cmd] usage: cmd <vx> <vy> <omega>");
            return;
        }
        const char* s2 = end;
        float vy = strtof(s2, &end);
        if (end == s2) {
            RemoteSerial::println("[cmd] usage: cmd <vx> <vy> <omega>");
            return;
        }
        const char* s3 = end;
        float omega = strtof(s3, &end);
        if (end == s3) {
            RemoteSerial::println("[cmd] usage: cmd <vx> <vy> <omega>");
            return;
        }
        if (g_remote_latch) {
            g_remote_latch->write({vx, vy, omega});
        }
        RemoteSerial::printf("[cmd] vx=%.3f vy=%.3f omega=%.3f\n", vx, vy, omega);
        return;
    }

    // arm/disarm/kill produce no echo here — ArmingState::* logs the
    // transition itself via arming_log(), so a second print would duplicate.
    if (trimmed == "arm") {
        ArmingState::arm();
        return;
    }
    if (trimmed == "disarm") {
        ArmingState::disarm();
        return;
    }
    if (trimmed == "kill") {
        ArmingState::kill();
        return;
    }
    if (trimmed == "clearkill") {
        ArmingState::clearKill();
        RemoteSerial::printf("[arming] %s\n", armingStateName(ArmingState::get()));
        return;
    }
    if (trimmed == "armstate") {
        RemoteSerial::printf("[arming] %s\n", armingStateName(ArmingState::get()));
        return;
    }

    // Falls through to the tuner. handle() ignores anything not starting
    // with `balance `, so unrelated lines get silently dropped here.
    g_tuner.handle(trimmed);
}

// Block until the BNO055 is calibrated enough to fuse a stable orientation.
// Control task is deliberately left un-started by the caller so no PWM is
// written while we wait. OtaSafeMode must be up first so the robot stays
// reflashable while gated here. See main_robot.cpp for full discussion of
// the IMUPLUS warm-/cold-boot split.
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
    esp_task_wdt_add(NULL);

    BodyVelocity last_known{};
    uint32_t     last_fresh_ms = 0;
    bool         have_seen_fresh = false;
    ArmingState::State prev_arming = ArmingState::get();

    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(CONTROL_PERIOD_MS);

    UBaseType_t stack_hwm_min = static_cast<UBaseType_t>(-1);
    uint32_t    hwm_iter = 0;

    while (true) {
        auto fresh = g_remote_latch->read();
        const uint32_t now = millis();
        if (fresh.has_value()) {
            last_known      = *fresh;
            last_fresh_ms   = now;
            have_seen_fresh = true;
        }

        // Arming-edge dispatch — mirrors main_robot.cpp. Disarmed→Armed and
        // Armed→Disarmed clear wheel-level + balance PID state that drifted
        // while update(dt) ran unarmed; entry to Killed brakes once.
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
    // Network/OTA FIRST. If anything below fails, the chip stays reachable
    // via STA (bb8-robot.local) or the always-on recovery SoftAP.
    BringUp::begin();
    ArmingState::begin();
    RemoteSerial::onMessage(handleLine);
    RemoteSerial::println("BB-8 main_balance_test starting.");
    RemoteSerial::println("[arming] boot state: Disarmed");

    Wire.begin(I2C_SDA, I2C_SCL);

    g_motor0.begin();
    g_motor1.begin();
    g_motor2.begin();

    if (!g_imu.begin()) {
        RemoteSerial::println("[imu] init failed — entering OTA-only degraded mode.");
        return;
    }

    g_remote_latch = new CommandLatch<BodyVelocity>();

    BalanceConfig boot_cfg;
    if (!BalanceConfigStorage::load(boot_cfg)) {
        boot_cfg = RobotConstants::balanceConfig();
        RemoteSerial::println("[balance] no persisted config; using defaults");
    } else {
        RemoteSerial::println("[balance] loaded persisted config from NVS");
    }
    g_tuner.begin(boot_cfg);
    BalanceHttpApi::registerRoutes(RemoteSerial::server(), g_tuner);
    ArmingHttpApi::registerRoutes(RemoteSerial::server());
    g_controller = new BalancingDrivetrainController(g_drivetrain, g_imu, g_tuner.slot());

    // Gate: no control task, no PWM until the IMU is fully calibrated.
    // Motors stay at PWM=0 from begin() throughout this wait.
    // Build with -DSKIP_IMU_CAL to bypass for bench testing.
#ifndef SKIP_IMU_CAL
    waitForCalibration();
#else
    RemoteSerial::println("[cal] SKIP_IMU_CAL set — arming teleop without calibration.");
#endif

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

    RemoteSerial::println("BB-8 balance bench ready.");
}

void loop() {
    BringUp::tick();
    vTaskDelay(pdMS_TO_TICKS(LOOP_TICK_MS));
}
