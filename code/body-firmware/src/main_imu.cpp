/*
 * BB-8 Body Firmware — BNO055 IMU Test
 * Prints sensor readings to serial every 100ms.
 * Serial commands:
 *   "all"   — show all fields
 *   "euler" — show euler angles only
 *   "quat"  — show quaternion only
 *   "gyro"  — show gyroscope only
 *   "cal"   — show calibration status only
 *   "accel" — show linear acceleration only
 */

#include <Arduino.h>
#include <Wire.h>
#include "BNO055IMU.h"
#include "pins.h"

static IMUField activeFields = IMUField::Euler | IMUField::Calibration;
static BNO055IMU sensor(0x28, &Wire, IMUField::Quaternion | IMUField::Euler |
                     IMUField::Gyro | IMUField::LinearAccel | IMUField::Calibration);

static void killMotorPins() {
    constexpr uint8_t pins[] = {
        MOTOR0_PINS.fwd, MOTOR0_PINS.rev,
        MOTOR1_PINS.fwd, MOTOR1_PINS.rev,
        MOTOR2_PINS.fwd, MOTOR2_PINS.rev,
    };
    for (auto p : pins) {
        pinMode(p, OUTPUT);
        digitalWrite(p, LOW);
    }
}

void setup() {
    Serial.begin(115200);
    killMotorPins();
    Wire.begin(26, 25);

    Serial.println("BNO055 IMU test — initializing...");
    if (!sensor.begin()) {
        Serial.println("ERR: BNO055 not detected. Check wiring (SDA=26, SCL=25).");
        while (true) delay(1000);
    }
    Serial.println("BNO055 ready. Commands: all, euler, quat, gyro, cal, accel");
}

void loop() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        if (cmd == "all")   activeFields = IMUField::Euler | IMUField::Quaternion | IMUField::Gyro | IMUField::LinearAccel | IMUField::Calibration;
        else if (cmd == "euler") activeFields = IMUField::Euler;
        else if (cmd == "quat")  activeFields = IMUField::Quaternion;
        else if (cmd == "gyro")  activeFields = IMUField::Gyro;
        else if (cmd == "cal")   activeFields = IMUField::Calibration;
        else if (cmd == "accel") activeFields = IMUField::LinearAccel;
        else Serial.println("ERR: unknown command");
    }

    IMUReading r = sensor.read();

    if (activeFields & IMUField::Euler)
        Serial.printf("Euler: h=%.1f r=%.1f p=%.1f  ", r.euler.heading, r.euler.roll, r.euler.pitch);

    if (activeFields & IMUField::Quaternion)
        Serial.printf("Quat: w=%.3f x=%.3f y=%.3f z=%.3f  ", r.orientation.w, r.orientation.x, r.orientation.y, r.orientation.z);

    if (activeFields & IMUField::Gyro)
        Serial.printf("Gyro: x=%.1f y=%.1f z=%.1f  ", r.gyro.x, r.gyro.y, r.gyro.z);

    if (activeFields & IMUField::LinearAccel)
        Serial.printf("Accel: x=%.2f y=%.2f z=%.2f  ", r.linearAccel.x, r.linearAccel.y, r.linearAccel.z);

    if (activeFields & IMUField::Calibration)
        Serial.printf("Cal: sys=%d gyro=%d accel=%d mag=%d", r.calibration.sys, r.calibration.gyro, r.calibration.accel, r.calibration.mag);

    Serial.println();
    delay(100);
}
