#pragma once

// Native-test stub. Real RemoteSerial dual-writes to Serial + WebSerial on
// ESP32. Here we drop output on the floor — tests don't assert on log lines.

#include <Arduino.h>
#include <cstdarg>
#include <cstdio>

namespace RemoteSerial {

inline void begin() {}
inline void tick() {}

inline void print(const String& /*s*/)   {}
inline void println(const String& /*s*/) {}

inline void printf(const char* /*fmt*/, ...) {}

inline void onMessage(void (*)(const String&)) {}

}  // namespace RemoteSerial
