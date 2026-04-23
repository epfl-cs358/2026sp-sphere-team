/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini, with assistance from Claude (Anthropic)
 */

#pragma once

template <typename T>
class InputSource {
public:
    virtual ~InputSource() = default;

    // Returns true if a new command is available.
    virtual bool hasCommand() = 0;

    // Retrieve the latest command.
    virtual T getCommand() = 0;
};
