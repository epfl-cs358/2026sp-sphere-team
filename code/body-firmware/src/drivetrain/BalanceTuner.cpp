/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 */

#include "BalanceTuner.h"

#include <cmath>
#include <cstdio>
#include <cstring>

#include "ArmingState.h"
#include "BalanceConfigStorage.h"
#include "BalanceTelemetry.h"
#include "RobotConstants.h"

#ifdef ARDUINO
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "RemoteSerial.h"
#endif

namespace {

struct Field {
    const char*           name;
    float BalanceConfig::* member;
};

// Field-name → member-pointer table. Used for `balance show`, `balance set
// <key> <value>`, and PIDs-only display. Negative-Kp guard applies only to
// the gain fields (Kp/Ki/Kd, indices 2..7 below); other invariants are
// checked separately in `_set`.
constexpr Field kAllFields[] = {
    {"tiltPerVelocity",   &BalanceConfig::tiltPerVelocity},
    {"maxTiltSetpoint",   &BalanceConfig::maxTiltSetpoint},
    {"pitchKp",           &BalanceConfig::pitchKp},
    {"pitchKi",           &BalanceConfig::pitchKi},
    {"pitchKd",           &BalanceConfig::pitchKd},
    {"rollKp",            &BalanceConfig::rollKp},
    {"rollKi",            &BalanceConfig::rollKi},
    {"rollKd",            &BalanceConfig::rollKd},
    {"pitchDeadband",     &BalanceConfig::pitchDeadband},
    {"rollDeadband",      &BalanceConfig::rollDeadband},
    {"maxOutputVelocity", &BalanceConfig::maxOutputVelocity},
    {"envelopeEnterSin",  &BalanceConfig::envelopeEnterSin},
    {"envelopeExitSin",   &BalanceConfig::envelopeExitSin},
    {"gyroPitchSign",     &BalanceConfig::gyroPitchSign},
    {"gyroRollSign",      &BalanceConfig::gyroRollSign},
};

constexpr const char* kPidFields[] = {
    "pitchKp", "pitchKi", "pitchKd",
    "rollKp",  "rollKi",  "rollKd",
};

bool isGainField(const char* name) {
    for (const char* g : kPidFields) {
        if (std::strcmp(g, name) == 0) return true;
    }
    return false;
}

const Field* findField(const String& key) {
    for (const auto& f : kAllFields) {
        if (key.equals(String(f.name))) return &f;
    }
    return nullptr;
}

String fmtFloat(float v) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%g", static_cast<double>(v));
    return String(buf);
}

const char* armingLabel(ArmingState::State s) {
    switch (s) {
        case ArmingState::State::Armed:    return "armed";
        case ArmingState::State::Disarmed: return "disarmed";
        case ArmingState::State::Killed:   return "killed";
    }
    return "?";
}

}  // namespace

BalanceTuner::BalanceTuner() {
#ifdef ARDUINO
    _print = [](const String& s) { RemoteSerial::println(s); };
#endif
}

void BalanceTuner::begin(const BalanceConfig& initial) {
    _bufA = initial;
    _bufB = initial;
    _slot.store(&_bufA, std::memory_order_release);
}

std::atomic<const BalanceConfig*>& BalanceTuner::slot() {
    return _slot;
}

void BalanceTuner::setPrint(PrintFn fn) {
    _print = std::move(fn);
}

bool BalanceTuner::_liveIsA() const {
    return _slot.load(std::memory_order_acquire) == &_bufA;
}

BalanceConfig* BalanceTuner::_spare() {
    return _liveIsA() ? &_bufB : &_bufA;
}

void BalanceTuner::_emit(const String& s) {
    if (_print) _print(s);
}

void BalanceTuner::handle(const String& line) {
    String l = line;
    l.trim();
    const String prefix("balance ");
    if (!l.startsWith(prefix)) {
        // Foreign verb — let the outer dispatcher handle it.
        return;
    }
    String rest = l.substring(prefix.length());
    rest.trim();

    if (rest.equals(String("show"))) {
        _show(false);
        return;
    }
    if (rest.equals(String("show pids"))) {
        _show(true);
        return;
    }
    if (rest.equals(String("reset"))) {
        _reset();
        return;
    }
    if (rest.equals(String("save"))) {
        _save();
        return;
    }
    if (rest.equals(String("status"))) {
        _status();
        return;
    }
    if (rest.startsWith(String("set "))) {
        String args = rest.substring(4);
        args.trim();
        int sp = args.indexOf(' ');
        if (sp <= 0) {
            _emit(String("error: usage: balance set <key> <value>"));
            return;
        }
        String key   = args.substring(0, sp);
        String valS  = args.substring(sp + 1);
        valS.trim();
        float value  = valS.toFloat();
        _set(key, value);
        return;
    }
    // Echo the unrecognized rest (quoted) so the operator can see if the
    // input had hidden chars / odd whitespace / fragmentation. Without this
    // the error is opaque when the literal-looking command is correct.
    _emit(String("error: unknown balance verb: '") + rest + String("'"));
}

void BalanceTuner::_show(bool pidsOnly) {
    const BalanceConfig* live = _slot.load(std::memory_order_acquire);
    if (!live) return;
    for (const auto& f : kAllFields) {
        if (pidsOnly && !isGainField(f.name)) continue;
        _emit(String(f.name) + String("=") + fmtFloat(live->*(f.member)));
    }
}

void BalanceTuner::_status() {
    // Pitch/roll require IMU state which the tuner does not own. Report
    // placeholders so the operator-visible keys are still present; the
    // wire-in in 2.7/2.8 may extend this once a controller snapshot
    // accessor lands.
    _emit(String("pitch=0"));
    _emit(String("roll=0"));
    _emit(String("armed=") + String(armingLabel(ArmingState::get())));
}

bool BalanceTuner::trySet(const String& key, float value, String& err) {
    const Field* f = findField(key);
    if (!f) {
        err = String("unknown key ") + key;
        return false;
    }

    BalanceConfig* spare = _spare();
    const BalanceConfig* live = _slot.load(std::memory_order_acquire);
    *spare = *live;
    spare->*(f->member) = value;

    if (isGainField(f->name) && value < 0.0f) {
        err = String("gain must be >= 0");
        return false;
    }
    if ((std::strcmp(f->name, "pitchDeadband") == 0 ||
         std::strcmp(f->name, "rollDeadband")  == 0) && value < 0.0f) {
        err = String("deadband must be >= 0");
        return false;
    }
    if (spare->envelopeExitSin >= spare->envelopeEnterSin) {
        err = String("envelopeExitSin must be < envelopeEnterSin");
        return false;
    }
    if (spare->maxTiltSetpoint >= std::asin(spare->envelopeEnterSin)) {
        err = String("maxTiltSetpoint must be < asin(envelopeEnterSin)");
        return false;
    }
    if (spare->maxOutputVelocity < 0.0f) {
        err = String("maxOutputVelocity must be >= 0");
        return false;
    }

    _publish(spare);
    _pendingEvents.fetch_or(kEvent_GAIN_CHANGED, std::memory_order_acq_rel);
    return true;
}

bool BalanceTuner::saveNvs() {
    const BalanceConfig* live = _slot.load(std::memory_order_acquire);
    if (!live) return false;
    BalanceConfigStorage::save(*live);
    _pendingEvents.fetch_or(kEvent_CONFIG_SAVED, std::memory_order_acq_rel);
    return true;
}

void BalanceTuner::resetToDefaults() {
    BalanceConfig* spare = _spare();
    *spare = RobotConstants::balanceConfig();
    _publish(spare);
    _pendingEvents.fetch_or(kEvent_CONFIG_RESET, std::memory_order_acq_rel);
}

BalanceConfig BalanceTuner::snapshot() const {
    const BalanceConfig* live = _slot.load(std::memory_order_acquire);
    if (!live) return BalanceConfig{};
    return *live;
}

void BalanceTuner::_set(const String& key, float value) {
    String err;
    if (trySet(key, value, err)) {
        _emit(String("ok: ") + key + String("=") + fmtFloat(value));
    } else {
        _emit(String("error: ") + err);
    }
}

void BalanceTuner::_reset() {
    resetToDefaults();
    _emit(String("ok: reset to defaults"));
}

void BalanceTuner::_save() {
    if (saveNvs()) {
        _emit(String("ok: saved"));
    }
}

void BalanceTuner::_publish(BalanceConfig* spare) {
    _slot.store(spare, std::memory_order_release);
#ifdef ARDUINO
    // One control period + a tick of slack ensures the controller has
    // observed the new pointer before the previous-live buffer becomes the
    // next spare. RemoteSerial is human-paced so this latency is invisible.
    vTaskDelay(pdMS_TO_TICKS(RobotConstants::CONTROL_PERIOD_MS + 1));
#endif
}
