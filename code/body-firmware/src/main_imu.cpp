/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini, with assistance from Claude (Anthropic)
 */

#include <Arduino.h>

#include "Motor.h"
#include "CommandBuffer.h"
#include "Drivetrain.h"
#include "IMU.h"
#include "InputSource.h"
#include "DrivetrainController.h"
#include "PID.h"

void setup() {
    Serial.begin(115200);
}

void loop() {
}
