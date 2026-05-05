/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini
 */

#pragma once

#include "Driver.h"
#include <cstdint>

struct L298NPins {
    uint8_t fwd;
    uint8_t rev;
};

class L298NDriver : public Driver {
public:
    explicit L298NDriver(L298NPins pins);

    void begin() override;
    void setOutput(float value) override;
    void brake() override;

private:
    L298NPins _pins;
};
