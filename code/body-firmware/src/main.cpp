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

// TODO: Define concrete types
// struct Velocity { ... };
// struct Command  { ... };
// struct IMUData  { ... };

// TODO: Implement concrete subclasses
// class BB8Motor      : public Motor { ... };
// class BB8Drivetrain : public Drivetrain<Velocity> { ... };
// class BNO055IMU     : public IMU<IMUData> { ... };
// class WSInput       : public InputSource<Command> { ... };
// class BB8Controller : public DrivetrainController<Command, IMUData, Velocity> { ... };
// class BB8CmdBuffer  : public CommandBuffer<Command> { ... };

// TODO: Instantiate hardware
// BB8Motor motor0(...);
// BB8Motor motor1(...);
// BB8Motor motor2(...);
// BB8CmdBuffer commandBuffer;

// TODO: FreeRTOS tasks
// Core 1 — real-time control loop
// void controlTask(void* param) {
//     for (;;) {
//         auto cmd = commandBuffer.read();
//         auto imu = myImu.read();
//         if (cmd) controller.update(*cmd, imu);
//         controller.drivetrain.update();
//         vTaskDelay(pdMS_TO_TICKS(1));
//     }
// }
//
// Core 0 — communications
// void commsTask(void* param) {
//     for (;;) {
//         if (input.hasCommand()) {
//             commandBuffer.write(input.getCommand());
//         }
//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
// }

void setup() {
    Serial.begin(115200);
    Serial.println("BB-8 Body Firmware");

    // TODO: Create FreeRTOS tasks pinned to cores
    // xTaskCreatePinnedToCore(controlTask, "control", 4096, nullptr, 1, nullptr, 1);
    // xTaskCreatePinnedToCore(commsTask,   "comms",   4096, nullptr, 1, nullptr, 0);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
