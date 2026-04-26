/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini
 */

#pragma once

#include "Driver.h"
#include "MotorConfig.h"
#include <BTS7960.h>

class BTS7960Driver : public Driver {
public:
    explicit BTS7960Driver(DriverPins pins);

    void begin() override;
    void setOutput(float value) override;
    void brake() override;

private:
    BTS7960 _hbridge;
};
