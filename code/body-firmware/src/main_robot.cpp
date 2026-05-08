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
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "config/wifi_credentials.h"

#include "BodyVelocity.h"
#include "OmniDrivetrain.h"
#include "BNO055IMU.h"
#include "L298NDriver.h"
#include "FIT0186Motor.h"
#include "PID.h"
#include "RobotConstants.h"
#include "pins.h"

#include "CommandLatch.h"
#include "WebSocketCommandProducer.h"
#include "PassthroughDrivetrainController.h"

namespace {

constexpr uint32_t STALENESS_TIMEOUT_MS = 200;
constexpr uint32_t CONTROL_PERIOD_MS    = 10;     // 100 Hz
constexpr uint16_t WS_PORT              = 81;
constexpr uint32_t WIFI_TIMEOUT_MS      = 30000;

constexpr uint32_t CONTROL_TASK_STACK_BYTES = 4096;
constexpr UBaseType_t CONTROL_TASK_PRIORITY = 4;
constexpr BaseType_t  CONTROL_TASK_CORE     = 1;

// I2C pins for BNO055 (matches main_imu.cpp)
constexpr int I2C_SDA = 26;
constexpr int I2C_SCL = 25;

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

CommandLatch<BodyVelocity> g_latch;
WebSocketCommandProducer    g_producer(g_latch, WS_PORT);
PassthroughDrivetrainController g_controller(g_drivetrain, g_imu);

void connectWifi() {
    WiFi.mode(WIFI_STA);
    WiFi.persistent(false);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    Serial.print("Connecting to WiFi");
    uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - start > WIFI_TIMEOUT_MS) {
            Serial.println("\nFATAL: WiFi connect timeout — restarting.");
            ESP.restart();
        }
        delay(500);
        Serial.print('.');
    }
    Serial.println();
    Serial.print("WiFi connected. IP: ");
    Serial.println(WiFi.localIP());
}

void controlTask(void* /*arg*/) {
    BodyVelocity last_known{};
    uint32_t     last_fresh_ms = 0;
    bool         have_seen_fresh = false;

    TickType_t lastWake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(CONTROL_PERIOD_MS);

    while (true) {
        // 1) Pull freshest command from latch.
        auto fresh = g_latch.read();
        const uint32_t now = millis();
        if (fresh.has_value()) {
            last_known      = *fresh;
            last_fresh_ms   = now;
            have_seen_fresh = true;
        }

        // 2) Compute drive command via dead-window policy.
        BodyVelocity drive_cmd{0.0f, 0.0f, 0.0f};
        if (g_producer.connected() && have_seen_fresh) {
            const uint32_t age = now - last_fresh_ms;
            if (age < STALENESS_TIMEOUT_MS) {
                const float scale = 1.0f - static_cast<float>(age) /
                                            static_cast<float>(STALENESS_TIMEOUT_MS);
                drive_cmd = { last_known.vx * scale,
                              last_known.vy * scale,
                              last_known.omega * scale };
            }
        }

        // 3) Tick controller.
        IMUReading imu_reading = g_imu.read();
        g_controller.update(drive_cmd, imu_reading);

        // 4) Tick motor controllers (RPM PID).
        const float dt = static_cast<float>(CONTROL_PERIOD_MS) / 1000.0f;
        g_drivetrain.update(dt);

        vTaskDelayUntil(&lastWake, period);
    }
}

}  // namespace

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println("BB-8 main_robot starting.");

    Wire.begin(I2C_SDA, I2C_SCL);

    g_motor0.begin();
    g_motor1.begin();
    g_motor2.begin();

    if (!g_imu.begin()) {
        Serial.println("FATAL: BNO055 init failed — restarting.");
        ESP.restart();
    }

    connectWifi();

    g_producer.start();

    xTaskCreatePinnedToCore(
        &controlTask,
        "control",
        CONTROL_TASK_STACK_BYTES,
        nullptr,
        CONTROL_TASK_PRIORITY,
        nullptr,
        CONTROL_TASK_CORE);

    Serial.println("BB-8 ready.");
}

void loop() {
    // Everything runs in dedicated FreeRTOS tasks. Keep loop() empty so the
    // default Arduino loopTask doesn't compete with our control loop.
    vTaskDelay(pdMS_TO_TICKS(1000));
}
