/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini, with assistance from Claude (Anthropic)
 */

#pragma once

#include <optional>

template <typename T>
class CommandBuffer {
public:
    virtual ~CommandBuffer() = default;

    // Write a new value (producer side, typically Core 0).
    virtual void write(const T& value) = 0;

    // Read the latest value (consumer side, typically Core 1).
    // Returns std::nullopt if no fresh data since last read.
    virtual std::optional<T> read() = 0;
};
