/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini, with assistance from Claude (Anthropic)
 *
 * CommandFrameParser — extracted so the parsing rules ("vx,vy,omega" with
 * forward-compat trailing fields, bounds checks, error reporting) are
 * directly unit-testable independent of the WebSocket transport.
 *
 * Wire format (leading-sigil dispatch):
 *   v<vx>,<vy>,<omega>   — velocity frame ('v' optional for backward compat)
 *   c:arm|disarm|kill|clearkill  — control frame
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "BodyVelocity.h"

enum class FrameKind : uint8_t { Velocity, Control };

enum class ControlVerb : uint8_t { Arm, Disarm, Kill, ClearKill, QueryArmState };

enum class FrameParseError {
    None,
    EmptyOrTooLong,
    InvalidVx,
    InvalidVy,
    InvalidOmega,
    InvalidControlVerb,
};

struct FrameParseResult {
    FrameKind kind = FrameKind::Velocity;
    BodyVelocity velocity{};
    ControlVerb control = ControlVerb::Arm;
    FrameParseError error = FrameParseError::None;
    bool ok() const { return error == FrameParseError::None; }
};

inline constexpr size_t kCommandFrameMaxLen = 64;

inline FrameParseResult parseCommandFrame(const uint8_t* payload, size_t length) {
    if (length == 0 || length >= kCommandFrameMaxLen) {
        FrameParseResult r;
        r.error = FrameParseError::EmptyOrTooLong;
        return r;
    }

    char buf[kCommandFrameMaxLen];
    for (size_t i = 0; i < length; ++i) buf[i] = static_cast<char>(payload[i]);
    buf[length] = '\0';

    const char first = buf[0];

    if (first == 'c') {
        FrameParseResult r;
        r.kind = FrameKind::Control;
        if (buf[1] != ':') {
            r.error = FrameParseError::InvalidControlVerb;
            return r;
        }
        const char* verb = buf + 2;
        if (std::strcmp(verb, "arm") == 0) {
            r.control = ControlVerb::Arm;
        } else if (std::strcmp(verb, "disarm") == 0) {
            r.control = ControlVerb::Disarm;
        } else if (std::strcmp(verb, "kill") == 0) {
            r.control = ControlVerb::Kill;
        } else if (std::strcmp(verb, "clearkill") == 0) {
            r.control = ControlVerb::ClearKill;
        } else if (std::strcmp(verb, "armstate?") == 0) {
            r.control = ControlVerb::QueryArmState;
        } else {
            r.error = FrameParseError::InvalidControlVerb;
        }
        return r;
    }

    char* p = buf;
    if (first == 'v') {
        ++p;
    } else if (!(first == '-' || first == '+' || first == '.' ||
                 (first >= '0' && first <= '9'))) {
        FrameParseResult r;
        r.error = FrameParseError::InvalidVx;
        return r;
    }

    char* end = nullptr;

    float vx = std::strtof(p, &end);
    if (end == p || *end != ',') {
        FrameParseResult r;
        r.error = FrameParseError::InvalidVx;
        return r;
    }
    p = end + 1;

    float vy = std::strtof(p, &end);
    if (end == p || *end != ',') {
        FrameParseResult r;
        r.error = FrameParseError::InvalidVy;
        return r;
    }
    p = end + 1;

    float omega = std::strtof(p, &end);
    if (end == p) {
        FrameParseResult r;
        r.error = FrameParseError::InvalidOmega;
        return r;
    }
    // Trailing chars (whitespace, newline, extra fields) intentionally ignored.

    FrameParseResult r;
    r.kind = FrameKind::Velocity;
    r.velocity = BodyVelocity{vx, vy, omega};
    return r;
}
