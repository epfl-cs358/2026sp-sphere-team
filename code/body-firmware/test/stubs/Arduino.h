#pragma once
#include <cstdint>
#include <cstdio>

using byte = uint8_t;

inline unsigned long _micros_value = 0;
inline unsigned long micros() { return _micros_value; }
inline void _set_micros(unsigned long v) { _micros_value = v; }
inline void delay(unsigned long) {}
