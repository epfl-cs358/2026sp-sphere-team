/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini, with assistance from Claude (Anthropic)
 *
 * CommandFrameParser — extracted so the parsing rules ("vx,vy,omega" with
 * forward-compat trailing fields, bounds checks, error reporting) are
 * directly unit-testable independent of the WebSocket transport.
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdlib>

#include "BodyVelocity.h"

enum class FrameParseError {
    None,
    EmptyOrTooLong,
    InvalidVx,
    InvalidVy,
    InvalidOmega,
};

struct FrameParseResult {
    BodyVelocity value{};
    FrameParseError error = FrameParseError::None;
    bool ok() const { return error == FrameParseError::None; }
};

inline constexpr size_t kCommandFrameMaxLen = 64;

inline FrameParseResult parseCommandFrame(const uint8_t* payload, size_t length) {
    if (length == 0 || length >= kCommandFrameMaxLen) {
        return {{}, FrameParseError::EmptyOrTooLong};
    }

    char buf[kCommandFrameMaxLen];
    for (size_t i = 0; i < length; ++i) buf[i] = static_cast<char>(payload[i]);
    buf[length] = '\0';

    char* p = buf;
    char* end = nullptr;

    float vx = std::strtof(p, &end);
    if (end == p || *end != ',') return {{}, FrameParseError::InvalidVx};
    p = end + 1;

    float vy = std::strtof(p, &end);
    if (end == p || *end != ',') return {{}, FrameParseError::InvalidVy};
    p = end + 1;

    float omega = std::strtof(p, &end);
    if (end == p) return {{}, FrameParseError::InvalidOmega};
    // Trailing chars (whitespace, newline, extra fields) intentionally ignored.

    return {BodyVelocity{vx, vy, omega}, FrameParseError::None};
}
