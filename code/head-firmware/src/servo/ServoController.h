#pragma once
#include "IServo.h"

class ServoController {
public:
    ServoController(IServo& servo, int pin);

    // Attaches the servo and moves to the center position (90°).
    void begin();

    // Parses a "tilt:<angle>" command and drives the servo.
    // Out-of-range angles are clamped to [0, 180]. Malformed messages are ignored.
    void handleCommand(const char* msg);

private:
    IServo& _servo;
    int _pin;
};
