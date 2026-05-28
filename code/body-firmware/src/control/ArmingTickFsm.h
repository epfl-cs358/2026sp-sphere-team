/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 *
 * ArmingTickFsm — edge-aware dispatch over ArmingState transitions, factored
 * out of controlTask so the control-loop edge logic is directly unit-testable.
 *
 * Semantics:
 *   - Disarmed -> Armed  : onArmedEdge   (seed staleness window, zero replay)
 *   - Armed    -> Disarmed: onDisarmEdge (clear PID windup; B3 fix)
 *   - *        -> Killed : onKilledEdge  (single hard brake)
 *   - same state         : no callback fires.
 *
 * `prev` is updated in-place after dispatch so callers store the FSM state
 * outside the loop body and pass it back in by reference each tick.
 */

#pragma once

#include <functional>

#include "ArmingState.h"

struct ArmingFsmCallbacks {
    std::function<void()> onArmedEdge;
    std::function<void()> onDisarmEdge;
    std::function<void()> onKilledEdge;
};

inline void armingFsmTick(ArmingState::State curr,
                          ArmingState::State& prev,
                          const ArmingFsmCallbacks& cb) {
    if (curr == prev) return;
    if (curr == ArmingState::State::Armed) {
        if (cb.onArmedEdge) cb.onArmedEdge();
    } else if (prev == ArmingState::State::Armed &&
               curr == ArmingState::State::Disarmed) {
        if (cb.onDisarmEdge) cb.onDisarmEdge();
    } else if (curr == ArmingState::State::Killed) {
        if (cb.onKilledEdge) cb.onKilledEdge();
    }
    prev = curr;
}
