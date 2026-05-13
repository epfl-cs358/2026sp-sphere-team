/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini, with assistance from Claude (Anthropic)
 *
 * End-to-end teleop: WebSocket producer (Core 0) → CommandLatch →
 * 100Hz control task (Core 1) → PassthroughDrivetrainController → OmniDrivetrain.
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
#include "PassthroughDrivetrainController.h"

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
PassthroughDrivetrainController* g_controller = nullptr;

// Block until the BNO055 reports full calibration on all four subsystems.
// Producer and control task are deliberately left un-started by the caller so
// no commands flow and no PWM is written while we wait. OtaSafeMode must be
// initialized first so the robot stays reflashable while gated here.
void waitForCalibration() {
    RemoteSerial::println("[cal] waiting for BNO055 full calibration before arming.");
    RemoteSerial::println("[cal]   gyro:  hold still");
    RemoteSerial::println("[cal]   accel: 6 distinct orientations, hold each ~2s");
    RemoteSerial::println("[cal]   mag:   slow figure-8 in the air");

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
    RemoteSerial::println("[cal] fully calibrated — arming teleop.");
}

void controlTask(void* /*arg*/) {
    // Subscribe to the Task Watchdog. If this loop hangs (e.g. I2C bus
    // stretch on the IMU), TWDT panics and resets — preferable to motors
    // stuck at last PWM with no operator recovery.
    esp_task_wdt_add(NULL);

    BodyVelocity last_known{};
    uint32_t     last_fresh_ms = 0;
    bool         have_seen_fresh = false;

    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(CONTROL_PERIOD_MS);

    UBaseType_t stack_hwm_min = static_cast<UBaseType_t>(-1);
    uint32_t    hwm_iter = 0;

    while (true) {
        // 1) Pull freshest command from latch.
        auto fresh = g_latch->read();
        const uint32_t now = millis();
        if (fresh.has_value()) {
            last_known      = *fresh;
            last_fresh_ms   = now;
            have_seen_fresh = true;
        }

        // 2) Drive command via staleness alone. The producer's connected()
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

        // 3) Hard-zero the command while an OTA upload is in progress, so
        //    motors halt while flash is being overwritten. Replaces the
        //    previous ArduinoOTA.onStart hook that lived in this TU.
        if (OtaSafeMode::isUpdating()) {
            drive_cmd = {0.0f, 0.0f, 0.0f};
        }

        // 4) Tick controller.
        IMUReading imu_reading = g_imu.read();
        g_controller->update(drive_cmd, imu_reading);

        // 5) Tick motor controllers (RPM PID).
        const float dt = static_cast<float>(CONTROL_PERIOD_MS) / 1000.0f;
        g_drivetrain.update(dt);

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
    RemoteSerial::println("BB-8 main_robot starting.");

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
    g_controller = new PassthroughDrivetrainController(g_drivetrain, g_imu);

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
