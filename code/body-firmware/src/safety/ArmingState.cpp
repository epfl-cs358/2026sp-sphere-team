#include "ArmingState.h"

#include <atomic>

#ifdef ARDUINO
#include <Arduino.h>
#include "RemoteSerial.h"
#endif

/// Single-writer invariant: all transitions are called from Core 0
/// (RemoteSerial/WebSocket dispatchers, OtaSafeMode::onStart). Readers on
/// the control task only use lock-free acquire loads — no compare_exchange.

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
    if (get() == State::Killed) return;
    g_state.store(State::Armed, std::memory_order_release);
    arming_log("[arming] -> Armed");
}

void disarm() {
    if (get() == State::Killed) return;
    g_state.store(State::Disarmed, std::memory_order_release);
    arming_log("[arming] -> Disarmed");
}

void kill() {
    g_state.store(State::Killed, std::memory_order_release);
    arming_log("[arming] -> Killed");
}

void clearKill() {
    if (get() != State::Killed) return;
    g_state.store(State::Disarmed, std::memory_order_release);
    arming_log("[arming] clearKill -> Disarmed");
}

}  // namespace ArmingState
