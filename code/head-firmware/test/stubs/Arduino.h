#pragma once
#include <cstdint>
#include <cstdio>

using byte = uint8_t;

#define OUTPUT 0x01
#define HIGH   0x01
#define LOW    0x00

inline unsigned long _micros_value = 0;
inline unsigned long micros() { return _micros_value; }
inline void _set_micros(unsigned long v) { _micros_value = v; }
inline void delay(unsigned long) {}

#ifdef ARDUINO_GPIO_FAKES
#include <fff.h>
DECLARE_FAKE_VOID_FUNC(pinMode, uint8_t, uint8_t);
DECLARE_FAKE_VOID_FUNC(digitalWrite, uint8_t, uint8_t);
DECLARE_FAKE_VOID_FUNC(analogWrite, uint8_t, uint8_t);

#define RESET_ARDUINO_FAKES() do { \
    RESET_FAKE(pinMode); \
    RESET_FAKE(digitalWrite); \
    RESET_FAKE(analogWrite); \
    FFF_RESET_HISTORY(); \
} while(0)
#else
inline void pinMode(uint8_t, uint8_t) {}
inline void digitalWrite(uint8_t, uint8_t) {}
inline void analogWrite(uint8_t, uint8_t) {}
#endif
