// Build-time check that the OTA_PASSWORD env var is set and long enough to
// double as the recovery SoftAP's WPA2 PSK. OTA_PASSWORD is injected via
// -DOTA_PASSWORD='"${sysenv.OTA_PASSWORD}"' in platformio.ini, so the same
// env var defines both the firmware-embedded password (ArduinoOTA.setPassword)
// and the upload tool's --auth value, eliminating drift.
//
// Unset env var       → OTA_PASSWORD expands to "" → sizeof == 1 → fails.
// Shorter than 8 chars → fails (WPA2 PSK minimum).
//
// This TU will be absorbed into OtaSafeMode.cpp once that module lands.

static_assert(sizeof(OTA_PASSWORD) >= 9,
    "OTA_PASSWORD env var must be set and at least 8 characters "
    "(WPA2 PSK minimum, since it doubles as the recovery SoftAP password). "
    "Run `export OTA_PASSWORD='your-secret'` before building.");
