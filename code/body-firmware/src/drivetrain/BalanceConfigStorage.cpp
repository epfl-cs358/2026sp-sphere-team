/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 */

#include "BalanceConfigStorage.h"

#include <Preferences.h>
#include <cstdint>
#include <cstring>

namespace {
constexpr const char* kNamespace = "balance";
constexpr const char* kKey       = "config";
constexpr size_t      kBlobSize  = sizeof(uint16_t) + sizeof(BalanceConfig);
}  // namespace

bool BalanceConfigStorage::load(BalanceConfig& out) {
    Preferences prefs;
    prefs.begin(kNamespace, true);
    uint8_t buf[kBlobSize];
    size_t len = prefs.getBytes(kKey, buf, sizeof(buf));
    prefs.end();

    if (len != kBlobSize) {
        return false;
    }
    uint16_t version = 0;
    std::memcpy(&version, buf, sizeof(uint16_t));
    if (version != VERSION) {
        return false;
    }
    std::memcpy(&out, buf + sizeof(uint16_t), sizeof(BalanceConfig));
    return true;
}

void BalanceConfigStorage::save(const BalanceConfig& cfg) {
    uint8_t buf[kBlobSize];
    uint16_t version = VERSION;
    std::memcpy(buf, &version, sizeof(uint16_t));
    std::memcpy(buf + sizeof(uint16_t), &cfg, sizeof(BalanceConfig));

    Preferences prefs;
    prefs.begin(kNamespace, false);
    prefs.putBytes(kKey, buf, sizeof(buf));
    prefs.end();
}
