/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini
 */

#pragma once

#include "Driver.h"
#include <BTS7960.h>
#include <cstdint>

struct BTS7960Pins {
    uint8_t l_en;
    uint8_t r_en;
    uint8_t l_pwm;
    uint8_t r_pwm;
};

class BTS7960Driver : public Driver {
public:
    explicit BTS7960Driver(BTS7960Pins pins);

    void begin() override;
    void setOutput(float value) override;
    void brake() override;

private:
    BTS7960 _hbridge;
};
