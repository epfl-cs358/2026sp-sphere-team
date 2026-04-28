/*
 * BB-8 Head Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini, with assistance from Claude (Anthropic)
 */

#pragma once

#include <cstdint>
#include <cstring>

struct FrameHeader {
    static constexpr size_t SIZE = 8;

    static void encode(uint8_t* buf, uint32_t sequence, uint32_t timestampMs) {
        memcpy(buf, &sequence, 4);
        memcpy(buf + 4, &timestampMs, 4);
    }

    static void decode(const uint8_t* buf, uint32_t& sequence, uint32_t& timestampMs) {
        memcpy(&sequence, buf, 4);
        memcpy(&timestampMs, buf + 4, 4);
    }
};
