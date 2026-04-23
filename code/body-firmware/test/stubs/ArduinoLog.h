#pragma once
#include <cstdio>

#define CR "\n"

class ArduinoLog {
public:
    void begin(int, void*) {}
    template <typename... Args> void fatal(const char* fmt, Args...) { printf("[FATAL] "); printf("%s\n", fmt); }
    template <typename... Args> void error(const char* fmt, Args...) { printf("[ERROR] "); printf("%s\n", fmt); }
    template <typename... Args> void warning(const char* fmt, Args...) { printf("[WARN] "); printf("%s\n", fmt); }
    template <typename... Args> void notice(const char* fmt, Args...) { printf("[NOTICE] "); printf("%s\n", fmt); }
    template <typename... Args> void trace(const char* fmt, Args...) { printf("[TRACE] "); printf("%s\n", fmt); }
    template <typename... Args> void verbose(const char* fmt, Args...) { printf("[VERBOSE] "); printf("%s\n", fmt); }
};

inline ArduinoLog Log;
