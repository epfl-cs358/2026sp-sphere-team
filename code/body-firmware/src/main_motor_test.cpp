/*
 * BB-8 Body Firmware — Motor Spin Test
 * Serial input: "<motor> <speed>" e.g. "0 0.5" or "1 -1.0"
 * "stop" to brake all motors
 */

#include <Arduino.h>
#include "pins.h"
#include "L298NDriver.h"

static constexpr int NUM_MOTORS = 3;

L298NDriver drivers[NUM_MOTORS] = {
    L298NDriver(MOTOR0_PINS),
    L298NDriver(MOTOR1_PINS),
    L298NDriver(MOTOR2_PINS),
};

void setup() {
    Serial.begin(115200);
    for (int i = 0; i < NUM_MOTORS; i++) {
        drivers[i].begin();
    }
    Serial.println("Motor test ready. Send: <motor 0-2> <speed -1.0 to 1.0>");
    Serial.println("Send 'stop' to brake all.");
}

void loop() {
    if (!Serial.available()) return;

    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line == "stop") {
        for (int i = 0; i < NUM_MOTORS; i++) {
            drivers[i].brake();
        }
        Serial.println("All motors braked.");
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

    drivers[motor].setOutput(speed);
    Serial.printf("Motor %d -> %.2f\n", motor, speed);
}
