#pragma once
#include <cstdint>
#include <cstdio>
#include <fff.h>

using byte = uint8_t;

constexpr uint8_t OUTPUT = 1;
constexpr uint8_t INPUT = 0;
constexpr uint8_t HIGH = 1;
constexpr uint8_t LOW = 0;

DECLARE_FAKE_VOID_FUNC(pinMode, uint8_t, uint8_t);
DECLARE_FAKE_VOID_FUNC(digitalWrite, uint8_t, uint8_t);

inline unsigned long _millis_value = 0;
inline unsigned long millis() { return _millis_value; }
inline void _set_millis(unsigned long v) { _millis_value = v; }

inline unsigned long _micros_value = 0;
inline unsigned long micros() { return _micros_value; }
inline void _set_micros(unsigned long v) { _micros_value = v; }

inline void delay(unsigned long) {}

#define RESET_ARDUINO_FAKES() do { \
    RESET_FAKE(pinMode);           \
    RESET_FAKE(digitalWrite);      \
    _millis_value = 0;             \
    _micros_value = 0;             \
} while(0)
