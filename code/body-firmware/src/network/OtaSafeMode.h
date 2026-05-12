#pragma once

// Bulletproof OTA bring-up shared across every main_*.cpp. After the first
// USB flash this is the only path firmware reaches the chip.
//
// Each main_*.cpp must place the OTA_SAFE_MODE_FOR("name") macro at file
// scope. The macro defines a const char* const that OtaSafeMode.cpp
// references in an un-elidable way, so forgetting it produces a link error:
//     undefined reference to `OtaSafeMode::kMainName'
//
// "name" is used as the mDNS hostname (e.g. "name.local") AND as the prefix
// of the always-on recovery SoftAP SSID ("name-recovery").
namespace OtaSafeMode {

extern const char* const kMainName;

// Call once in setup() AFTER Serial.begin() but BEFORE any risky hardware
// init. Brings up WiFi STA + always-on SoftAP recovery channel +
// ArduinoOTA, all keyed off kMainName + the build-time OTA_PASSWORD env var.
void begin();

// Call once per loop() iteration. Services OTA uploads, the mark-valid
// rollback timer, and the AP-aware reboot watchdog.
void tick();

// True iff ArduinoOTA is currently servicing an upload — main code can
// gate dangerous work (PWM, motor commands) without coupling to ArduinoOTA.
bool isUpdating();

}  // namespace OtaSafeMode

// One-liner each main_*.cpp MUST place at file scope.
#define OTA_SAFE_MODE_FOR(NAME) \
    namespace OtaSafeMode { const char* const kMainName = NAME; }
