#include "ArmingState.h"

#include <atomic>
#include <cmath>

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

// Pre-arm gyro-quiet ring buffer. Lives at module scope so the control task
// can push samples every tick without owning an ArmingState handle. Single
// writer (the control task) + single arm-attempt reader; each slot is an
// atomic<float> so the reader never tears mid-update. The head index is also
// atomic to bound writer-vs-reader interleavings.
std::atomic<float>    g_gyroBuf[PREARM_QUIET_WINDOW_SAMPLES] = {};
std::atomic<uint32_t> g_gyroHead{0};
std::atomic<bool>     g_prearmRejected{false};

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
    // Zero the gyro-quiet buffer so a fresh begin() doesn't see stale loud
    // samples from a previous test or boot session.
    for (auto& slot : g_gyroBuf) slot.store(0.0f, std::memory_order_relaxed);
    g_gyroHead.store(0, std::memory_order_release);
    g_prearmRejected.store(false, std::memory_order_release);
    arming_log("[arming] begin: Disarmed");
}

State get() {
    return g_state.load(std::memory_order_acquire);
}

bool isArmed() {
    return get() == State::Armed;
}

void arm() {
    // Pre-arm gyro-quiet gate: refuse if any sample in the last 500 ms had
    // |gyro.z| above the threshold. Checks magnitude only (sign-agnostic).
    // The buffer is populated by the control task every tick — even when
    // Disarmed or Killed — so by the time the user issues an arm verb the
    // window is always warm.
    for (const auto& slot : g_gyroBuf) {
        const float v = slot.load(std::memory_order_acquire);
        if (std::fabs(v) > PREARM_GYRO_QUIET_DPS) {
            g_prearmRejected.store(true, std::memory_order_release);
            arming_log("[arming] arm refused: gyro not quiet");
            return;
        }
    }
    State expected = State::Disarmed;
    if (g_state.compare_exchange_strong(expected, State::Armed,
                                        std::memory_order_acq_rel,
                                        std::memory_order_acquire)) {
        arming_log("[arming] -> Armed");
    }
    // If expected != Disarmed (Killed or already Armed), this is a no-op.
}

void recordGyroZ(float gyroZ) {
    const uint32_t head = g_gyroHead.load(std::memory_order_relaxed);
    g_gyroBuf[head % PREARM_QUIET_WINDOW_SAMPLES].store(
        gyroZ, std::memory_order_release);
    g_gyroHead.store(head + 1, std::memory_order_release);
}

bool consumePrearmRejected() {
    return g_prearmRejected.exchange(false, std::memory_order_acq_rel);
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
