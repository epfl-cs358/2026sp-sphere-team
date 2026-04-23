#pragma once
#include <cstdint>
#include <fff.h>

DECLARE_FAKE_VOID_FUNC(BTS7960_Enable);
DECLARE_FAKE_VOID_FUNC(BTS7960_Disable);
DECLARE_FAKE_VOID_FUNC(BTS7960_Stop);
DECLARE_FAKE_VOID_FUNC(BTS7960_TurnLeft, uint8_t);
DECLARE_FAKE_VOID_FUNC(BTS7960_TurnRight, uint8_t);

class BTS7960 {
public:
    BTS7960(int, int, int, int) {}
    BTS7960(int, int, int) {}
    void Enable() { BTS7960_Enable(); }
    void Disable() { BTS7960_Disable(); }
    void Stop() { BTS7960_Stop(); }
    void TurnLeft(uint8_t pwm) { BTS7960_TurnLeft(pwm); }
    void TurnRight(uint8_t pwm) { BTS7960_TurnRight(pwm); }
};

#define RESET_BTS7960_FAKES() do { \
    RESET_FAKE(BTS7960_Enable); \
    RESET_FAKE(BTS7960_Disable); \
    RESET_FAKE(BTS7960_Stop); \
    RESET_FAKE(BTS7960_TurnLeft); \
    RESET_FAKE(BTS7960_TurnRight); \
    FFF_RESET_HISTORY(); \
} while(0)
