/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini, with assistance from Claude (Anthropic)
 */

#pragma once

class CommandProducer {
public:
    virtual ~CommandProducer() = default;

    // Bring the producer online: start any FreeRTOS task, open sockets, etc.
    virtual void start() = 0;

    // Bring the producer offline. Must be synchronous: caller can rely on the
    // task having exited and resources being released by the time stop() returns.
    virtual void stop() = 0;

    // True iff the producer's transport is currently connected and capable of
    // delivering fresh commands. Cheap to call from any context.
    virtual bool connected() const = 0;
};
