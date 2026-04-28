/*
 * BB-8 Head Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini, with assistance from Claude (Anthropic)
 */

#pragma once

#ifdef BB8_DEBUG

#include <cstdio>
#include <cstdlib>

#ifndef BB8_ASSERT_HANDLER
#define BB8_ASSERT_HANDLER(msg, file, line) do { \
    fprintf(stderr, "ASSERT FAILED: %s (%s:%d)\n", msg, file, line); \
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
