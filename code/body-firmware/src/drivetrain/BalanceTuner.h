/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 */

#pragma once

#include <Arduino.h>

#include <atomic>
#include <functional>

#include "BalanceConfig.h"

// RemoteSerial-driven runtime tuner for `BalanceConfig`.
//
// Owns two `BalanceConfig` buffers and an `std::atomic<const BalanceConfig*>`
// slot. The active buffer is published via `slot()` for the control task to
// read with `memory_order_acquire`; on every `balance set` the spare buffer
// is mutated in place, validated, then published with `memory_order_release`
// and a one control-period sleep before the next `set` is accepted (see
// spec §"Atomic config swap").
//
// Verbs (prefix `balance `): `show`, `show pids`, `set <key> <value>`,
// `reset`, `save`, `status`. Lines without the `balance ` prefix are
// ignored so the main dispatcher can fall through cleanly.
class BalanceTuner {
public:
    using PrintFn = std::function<void(const String&)>;

    BalanceTuner();

    void begin(const BalanceConfig& initial);

    std::atomic<const BalanceConfig*>& slot();

    void handle(const String& line);

    void setPrint(PrintFn fn);

    // Non-emitting variants used by the HTTP JSON API. On failure `err` is
    // populated and the live slot is unchanged; on success the new value is
    // published and the swap-quiescence delay is applied (same as `_set`).
    bool trySet(const String& key, float value, String& err);
    bool saveNvs();
    void resetToDefaults();
    BalanceConfig snapshot() const;

    // Atomically reads and clears the latched event bits accumulated since
    // the last call. Bit positions match `BalanceTelemetry` event_flags
    // (GAIN_CHANGED / CONFIG_SAVED / CONFIG_RESET). Single-consume: the
    // returned value is the prior contents; the field is reset to zero.
    uint32_t consumePending() {
        return _pendingEvents.exchange(0, std::memory_order_acq_rel);
    }

private:
    BalanceConfig _bufA{};
    BalanceConfig _bufB{};
    std::atomic<const BalanceConfig*> _slot{nullptr};
    std::atomic<uint32_t> _pendingEvents{0};
    PrintFn _print;

    // True if `_slot.load()` currently points at `_bufA` (so the spare is `_bufB`).
    bool _liveIsA() const;
    BalanceConfig* _spare();

    void _emit(const String& s);
    void _show(bool pidsOnly);
    void _status();
    void _set(const String& key, float value);
    void _reset();
    void _save();
    void _publish(BalanceConfig* spare);
};
