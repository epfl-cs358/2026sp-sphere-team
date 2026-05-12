#pragma once
#include <cstdint>

// Pure decision logic for OtaSafeMode. No Arduino/ESP32 dependencies so
// these functions are exercised in env:native unit tests; the hardware
// integration that consumes them lives in OtaSafeMode.cpp.
namespace OtaPolicy {

// True iff esp_ota_mark_app_valid_cancel_rollback() should be called now.
// The firmware is considered "demonstrably working" once it has been running
// for at least delay_ms past boot — long enough that any setup() crash would
// have already rebooted the chip.
constexpr bool shouldMarkValid(uint32_t now_ms,
                               uint32_t boot_ms,
                               bool     already_marked,
                               uint32_t delay_ms) {
    return !already_marked && (now_ms - boot_ms) >= delay_ms;
}

// True iff loop() should ESP.restart() due to total network loss. The reboot
// fires ONLY when neither STA nor AP is up AND STA has been offline past the
// threshold — the always-on SoftAP is the recovery channel, so its presence
// vetoes the reboot regardless of STA state.
constexpr bool shouldRebootForOffline(bool     ap_up,
                                      bool     sta_connected,
                                      uint32_t now_ms,
                                      uint32_t last_sta_connect_ms,
                                      uint32_t threshold_ms) {
    if (ap_up || sta_connected) return false;
    return (now_ms - last_sta_connect_ms) > threshold_ms;
}

}  // namespace OtaPolicy
