/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 *
 * Copy this file to wifi_credentials.h (gitignored) and fill in your
 * network's SSID and password. The real wifi_credentials.h must NOT be
 * committed.
 *
 * OTA_PASSWORD is NOT defined here. It is supplied at build time via the
 * OTA_PASSWORD environment variable, which platformio.ini passes through
 * both as a compiler -D flag (for ArduinoOTA.setPassword + the recovery
 * SoftAP PSK) and as the upload tool's --auth flag, so build-time and
 * upload-time values cannot drift. Must be >=8 chars. Export it before
 * building or flashing:
 *   export OTA_PASSWORD='your-ota-password'
 *   pio run -e robot -t upload          # USB (one-time)
 *   pio run -e robot_ota -t upload      # OTA via mDNS
 * An unset or too-short value produces a clear static_assert build error.
 */

#pragma once

#define WIFI_SSID     "your-ssid"
#define WIFI_PASSWORD "your-password"
