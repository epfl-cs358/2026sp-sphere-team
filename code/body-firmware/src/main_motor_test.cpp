/*
 * BB-8 Body Firmware — Motor + Encoder Test
 * Serial commands:
 *   "<motor> <speed>"  — set motor 0-2 to speed -1.0..1.0
 *   "stop"             — brake all motors
 *   "rpm"              — toggle continuous RPM display
 */

#include <Arduino.h>
#include "pins.h"
#include "L298NDriver.h"
#include "FIT0186Motor.h"

#include "OtaSafeMode.h"
OTA_SAFE_MODE_FOR("bb8-motor-test");

static constexpr int NUM_MOTORS = 3;
static constexpr unsigned long UPDATE_INTERVAL_MS = 20;

L298NDriver drivers[NUM_MOTORS] = {
    L298NDriver(MOTOR0_PINS),
    L298NDriver(MOTOR1_PINS),
    L298NDriver(MOTOR2_PINS),
};

FIT0186Motor<5> motors[NUM_MOTORS] = {
    FIT0186Motor<5>(drivers[0], MOTOR0_ENCODER),
    FIT0186Motor<5>(drivers[1], MOTOR1_ENCODER),
    FIT0186Motor<5>(drivers[2], MOTOR2_ENCODER),
};

static bool showRPM = false;
static unsigned long lastUpdate = 0;

void setup() {
    Serial.begin(115200);
    OtaSafeMode::begin();
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].begin();
    }
    Serial.println("Motor+encoder test ready. Send: <motor 0-2> <speed -1.0 to 1.0>");
    Serial.println("Send 'stop' to brake all, 'rpm' to toggle RPM display.");
}

void loop() {
    OtaSafeMode::tick();
    unsigned long now = millis();
    if (now - lastUpdate >= UPDATE_INTERVAL_MS) {
        lastUpdate = now;
        for (int i = 0; i < NUM_MOTORS; i++) {
            motors[i].update();
        }
        if (showRPM) {
            Serial.printf("RPM: [0]=%.1f  [1]=%.1f  [2]=%.1f\n",
                motors[0].getFilteredRPM(),
                motors[1].getFilteredRPM(),
                motors[2].getFilteredRPM());
        }
    }

    if (!Serial.available()) return;

    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line == "stop") {
        for (int i = 0; i < NUM_MOTORS; i++) {
            motors[i].brake();
        }
        Serial.println("All motors braked.");
        return;
    }

    if (line == "rpm") {
        showRPM = !showRPM;
        Serial.printf("RPM display %s\n", showRPM ? "ON" : "OFF");
        return;
    }

    int motor = line.substring(0, line.indexOf(' ')).toInt();
    float speed = line.substring(line.indexOf(' ') + 1).toFloat();

    if (motor < 0 || motor >= NUM_MOTORS) {
        Serial.println("ERR: motor must be 0-2");
        return;
    }
    if (speed < -1.0f || speed > 1.0f) {
        Serial.println("ERR: speed must be -1.0 to 1.0");
        return;
    }

    motors[motor].setSpeed(speed);
    Serial.printf("Motor %d -> %.2f\n", motor, speed);
}
