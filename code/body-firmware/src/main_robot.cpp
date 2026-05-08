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
#include <ArduinoOTA.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_task_wdt.h>

#include "config/wifi_credentials.h"

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

namespace {

// Loop timing constants live in RobotConstants (shared with the rest of the
// drivetrain stack); deployment-only constants stay local.
using RobotConstants::STALENESS_TIMEOUT_MS;
using RobotConstants::CONTROL_PERIOD_MS;

constexpr uint16_t WS_PORT                 = 80;
constexpr uint32_t WIFI_TIMEOUT_MS         = 30000;
constexpr uint32_t WIFI_OFFLINE_REBOOT_MS  = 30000;  // reboot if offline this long
constexpr uint32_t TWDT_TIMEOUT_S          = 1;      // tighter than Arduino default ~5s
constexpr uint32_t LOOP_TICK_MS            = 100;    // 10 Hz housekeeping (OTA + wifi)
constexpr const char* OTA_HOSTNAME         = "bb8-robot";

constexpr uint32_t CONTROL_TASK_STACK_BYTES = 8192;
constexpr UBaseType_t CONTROL_TASK_PRIORITY = 4;
constexpr BaseType_t  CONTROL_TASK_CORE     = 1;

// I2C pins for BNO055 (matches main_imu.cpp)
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

// Constructed in setup() after WiFi/IMU/Wire are up, so FreeRTOS objects
// (queue inside the latch, server/task inside the producer) are created
// in a known-good runtime context rather than during C++ static init.
CommandLatch<BodyVelocity>*      g_latch      = nullptr;
WebSocketCommandProducer*        g_producer   = nullptr;
PassthroughDrivetrainController* g_controller = nullptr;

volatile uint32_t g_lastWifiConnectedMs = 0;

void onWifiEvent(WiFiEvent_t event) {
    switch (event) {
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            g_lastWifiConnectedMs = millis();
            Serial.print("[wifi] got IP ");
            Serial.println(WiFi.localIP());
            break;
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            Serial.println("[wifi] disconnected");
            break;
        default:
            break;
    }
}

void initOta() {
    ArduinoOTA.setHostname(OTA_HOSTNAME);
    ArduinoOTA.setPassword(OTA_PASSWORD);

    ArduinoOTA.onStart([]() {
        // Stop accepting new commands. Once the producer is gone, the staleness
        // ramp in controlTask will drive motors to zero within 200 ms before
        // the firmware actually overwrites flash.
        Serial.println("[ota] update starting; halting teleop");
        if (g_producer) g_producer->stop();
    });
    ArduinoOTA.onEnd([]() {
        Serial.println("[ota] update complete; rebooting");
    });
    ArduinoOTA.onError([](ota_error_t err) {
        Serial.printf("[ota] error %u\n", err);
    });

    ArduinoOTA.begin();
    Serial.printf("[ota] ready on %s.local\n", OTA_HOSTNAME);
}

void connectWifi() {
    WiFi.mode(WIFI_STA);
    WiFi.persistent(false);
    WiFi.setAutoReconnect(true);
    WiFi.onEvent(onWifiEvent);
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

        // 3) Tick controller.
        IMUReading imu_reading = g_imu.read();
        g_controller->update(drive_cmd, imu_reading);

        // 4) Tick motor controllers (RPM PID).
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

    g_latch      = new CommandLatch<BodyVelocity>();
    g_producer   = new WebSocketCommandProducer(*g_latch, WS_PORT);
    g_controller = new PassthroughDrivetrainController(g_drivetrain, g_imu);

    g_producer->start();

    initOta();

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

    Serial.println("BB-8 ready.");
}

void loop() {
    // Housekeeping: OTA poll + WiFi reboot watchdog. Control runs in dedicated
    // tasks. Tick at 10 Hz so OTA initiate requests are answered promptly.
    ArduinoOTA.handle();

    if (WiFi.status() == WL_CONNECTED) {
        g_lastWifiConnectedMs = millis();
    } else if (millis() - g_lastWifiConnectedMs > WIFI_OFFLINE_REBOOT_MS) {
        Serial.println("FATAL: WiFi offline >30s — restarting.");
        ESP.restart();
    }
    vTaskDelay(pdMS_TO_TICKS(LOOP_TICK_MS));
}
