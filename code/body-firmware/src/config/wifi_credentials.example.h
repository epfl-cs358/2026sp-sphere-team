/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 *
 * Copy this file to wifi_credentials.h (gitignored) and fill in your
 * network's SSID and password. The real wifi_credentials.h must NOT be
 * committed.
 *
 * OTA_PASSWORD authenticates over-the-air firmware updates pushed via
 * PlatformIO's espota protocol (env:robot_ota). The same password must be
 * exported before flashing:
 *   export OTA_PASSWORD='your-ota-password'
 *   pio run -e robot_ota -t upload
 */

#pragma once

#define WIFI_SSID     "your-ssid"
#define WIFI_PASSWORD "your-password"
#define OTA_PASSWORD  "your-ota-password"
