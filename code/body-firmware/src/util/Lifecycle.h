/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini, with assistance from Claude (Anthropic)
 *
 * Lifecycle — opt-in interface for components that need explicit bring-up
 * and tear-down (FreeRTOS tasks, network sockets, etc.). Kept separate from
 * domain interfaces so polled or stateless components don't have to provide
 * empty start()/stop() impls.
 */

#pragma once

class Lifecycle {
public:
    virtual ~Lifecycle() = default;

    // Bring the component online: start any FreeRTOS task, open sockets, etc.
    virtual void start() = 0;

    // Bring the component offline. Must be synchronous: caller can rely on
    // the task having exited and resources being released by the time
    // stop() returns.
    virtual void stop() = 0;
};
