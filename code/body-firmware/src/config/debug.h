/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini
 */

#pragma once

#include <ArduinoLog.h>

#ifdef BB8_DEBUG

#ifndef BB8_ASSERT_HANDLER
#define BB8_ASSERT_HANDLER(msg, file, line) do { \
    Log.fatal("ASSERT FAILED: %s (%s:%d)" CR, msg, file, line); \
    abort(); \
} while(0)
#endif

#define BB8_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            BB8_ASSERT_HANDLER(msg, __FILE__, __LINE__); \
        } \
    } while (0)
#else
#define BB8_ASSERT(cond, msg) ((void)0)
#endif
