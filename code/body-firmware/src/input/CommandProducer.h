/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini, with assistance from Claude (Anthropic)
 */

#pragma once

// CommandProducer — the transport-health half of a producer. Lifecycle
// (start/stop) is a separate opt-in interface (util/Lifecycle.h) so a polled
// producer (e.g. a local gamepad) can implement just this.
class CommandProducer {
public:
    virtual ~CommandProducer() = default;

    // True iff the producer's transport is currently connected and capable of
    // delivering fresh commands. Cheap to call from any context.
    virtual bool connected() const = 0;
};
