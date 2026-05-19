#include "ArmingState.h"

#include <atomic>

#ifdef ARDUINO
#include <Arduino.h>
#include "RemoteSerial.h"
#endif

/// Writers (arm/disarm/kill/clearKill) may run from multiple tasks: the
/// WebSocket dispatcher on Core 0, the RemoteSerial line handler on its own
/// task, and OtaSafeMode::onStart from the Arduino loop() task on Core 1.
/// State is std::atomic with release/acquire ordering. arm/disarm/clearKill
/// use compare_exchange_strong against the expected source state so a kill()
/// landing between the load and store of a concurrent arm() cannot be
/// silently overwritten — kill must always win. kill() itself remains a plain
/// store so it can preempt any pending source state. Control-task readers use
/// lock-free acquire loads.

namespace ArmingState {

namespace {

std::atomic<State> g_state{State::Disarmed};

void arming_log(const char* msg) {
#ifdef ARDUINO
    Serial.println(msg);
    RemoteSerial::println(msg);
#else
    (void)msg;
#endif
}

}  // namespace

void begin() {
    g_state.store(State::Disarmed, std::memory_order_release);
    arming_log("[arming] begin: Disarmed");
}

State get() {
    return g_state.load(std::memory_order_acquire);
}

bool isArmed() {
    return get() == State::Armed;
}

void arm() {
    State expected = State::Disarmed;
    if (g_state.compare_exchange_strong(expected, State::Armed,
                                        std::memory_order_acq_rel,
                                        std::memory_order_acquire)) {
        arming_log("[arming] -> Armed");
    }
    // If expected != Disarmed (Killed or already Armed), this is a no-op.
}

void disarm() {
    State expected = State::Armed;
    if (g_state.compare_exchange_strong(expected, State::Disarmed,
                                        std::memory_order_acq_rel,
                                        std::memory_order_acquire)) {
        arming_log("[arming] -> Disarmed");
    }
    // If expected was Killed or Disarmed, no-op.
}

void kill() {
    g_state.store(State::Killed, std::memory_order_release);
    arming_log("[arming] -> Killed");
}

void clearKill() {
    State expected = State::Killed;
    if (g_state.compare_exchange_strong(expected, State::Disarmed,
                                        std::memory_order_acq_rel,
                                        std::memory_order_acquire)) {
        arming_log("[arming] clearKill -> Disarmed");
    }
}

}  // namespace ArmingState
