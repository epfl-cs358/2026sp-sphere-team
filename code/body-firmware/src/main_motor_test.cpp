/*
 * BB-8 Body Firmware — Motor + Encoder Test
 *
 * Layout (pull config from RobotConstants::drivetrainConfig()):
 *   motor 0 — back        (wheel azimuth 180°)
 *   motor 1 — front-right (wheel azimuth 300°)
 *   motor 2 — front-left  (wheel azimuth  60°)
 *
 * Commands (accepted on USB Serial AND browser at /webserial):
 *   "<motor> <speed>"  — set motor 0-2 to speed -1.0..1.0
 *   "stop"             — brake all motors
 *   "rpm"              — toggle continuous RPM display
 */

#include <Arduino.h>
#include "pins.h"
#include "L298NDriver.h"
#include "FIT0186Motor.h"

#include "BringUp.h"
OTA_SAFE_MODE_FOR("bb8-robot");

static constexpr int NUM_MOTORS = 3;
static constexpr unsigned long UPDATE_INTERVAL_MS = 20;

static const char* const MOTOR_LABEL[NUM_MOTORS] = {
    "back", "front-right", "front-left",
};

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

static void handleCommand(const String& raw) {
    String line = raw;
    line.trim();
    if (line.length() == 0) return;

    if (line == "stop") {
        for (int i = 0; i < NUM_MOTORS; i++) motors[i].brake();
        RemoteSerial::println("All motors braked.");
        return;
    }

    if (line == "rpm") {
        showRPM = !showRPM;
        RemoteSerial::printf("RPM display %s\n", showRPM ? "ON" : "OFF");
        return;
    }

    int sp = line.indexOf(' ');
    if (sp < 0) {
        RemoteSerial::println("ERR: format is <motor 0-2> <speed -1.0..1.0>");
        return;
    }

    int motor = line.substring(0, sp).toInt();
    float speed = line.substring(sp + 1).toFloat();

    if (motor < 0 || motor >= NUM_MOTORS) {
        RemoteSerial::println("ERR: motor must be 0-2");
        return;
    }
    if (speed < -1.0f || speed > 1.0f) {
        RemoteSerial::println("ERR: speed must be -1.0 to 1.0");
        return;
    }

    motors[motor].setSpeed(speed);
    RemoteSerial::printf("Motor %d (%s) -> %.2f\n", motor, MOTOR_LABEL[motor], speed);
}

void setup() {
    BringUp::begin();

    // Bring each driver up AND force its PWM channels to zero before anything
    // else runs. begin() only configures pinMode; setSpeed(0.0f) guarantees
    // both fwd/rev pins are written to 0 (no stale LEDC state from before
    // the soft-reset can leak into the wheel).
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].begin();
        motors[i].setSpeed(0.0f);
    }

    RemoteSerial::onMessage(handleCommand);

    RemoteSerial::println("Motor+encoder test ready.");
    RemoteSerial::println("Layout (pull): 0=back, 1=front-right, 2=front-left");
    RemoteSerial::println("Commands: <motor 0-2> <speed -1.0..1.0> | stop | rpm");
}

void loop() {
    BringUp::tick();

    unsigned long now = millis();
    if (now - lastUpdate >= UPDATE_INTERVAL_MS) {
        lastUpdate = now;
        for (int i = 0; i < NUM_MOTORS; i++) motors[i].update();
        if (showRPM) {
            RemoteSerial::printf("RPM: [0]=%.1f [1]=%.1f [2]=%.1f\n",
                motors[0].getFilteredRPM(),
                motors[1].getFilteredRPM(),
                motors[2].getFilteredRPM());
        }
    }

    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        handleCommand(line);
    }
}
