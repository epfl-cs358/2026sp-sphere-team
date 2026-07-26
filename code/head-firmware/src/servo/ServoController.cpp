#include "ServoController.h"
#include "CommandParser.h"
#include <algorithm>

ServoController::ServoController(IServo& servo, int pin)
    : _servo(servo), _pin(pin) {}

void ServoController::begin() {
    _servo.attach(_pin);
    _servo.write(90);
}

void ServoController::handleCommand(const char* msg) {
    int angle = parseServoCommand(msg);
    if (angle == -1) return;
    angle = std::max(0, std::min(180, angle));
    _servo.write(angle);
}
