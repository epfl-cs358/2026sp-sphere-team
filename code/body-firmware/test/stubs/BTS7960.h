#pragma once
#include <cstdint>

class BTS7960 {
public:
    BTS7960(int, int, int, int) {}
    BTS7960(int, int, int) {}
    void Enable() {}
    void Disable() {}
    void Stop() {}
    void TurnLeft(int8_t) {}
    void TurnRight(int8_t) {}
};
