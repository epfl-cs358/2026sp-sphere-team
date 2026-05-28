#include "OtaSafeMode.h"

#include "ArmingState.h"

#include <atomic>

#ifdef ARDUINO
#include "OtaPolicy.h"
#include "RemoteSerial.h"
#include "wifi_credentials.h"

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <esp_ota_ops.h>

// Single source of truth. OTA_PASSWORD is defined at build time from the
// OTA_PASSWORD env var (see platformio.ini build_flags), so build-time and
// upload-time values cannot drift. Unset env → "" → sizeof==1 → fails.
// Shorter than 8 chars → fails (WPA2 PSK minimum for the recovery SoftAP).
static_assert(sizeof(OTA_PASSWORD) >= 9,
    "OTA_PASSWORD env var must be set and at least 8 characters "
    "(WPA2 PSK minimum, since it doubles as the recovery SoftAP password). "
    "Run `export OTA_PASSWORD='your-secret'` before building.");
#endif  // ARDUINO

namespace OtaSafeMode {
namespace {

std::atomic<bool>  g_updating{false};

static void onOtaStart() {
    g_updating.store(true, std::memory_order_release);
    ArmingState::disarm();
#ifdef ARDUINO
    // RemoteSerial fans out to both Serial and WebSerial — bench operators
    // watching WebSerial during an OTA push see this line.
    RemoteSerial::println("[ota] update starting");
#endif
}

#ifdef ARDUINO

// Anchor kMainName so a main missing OTA_SAFE_MODE_FOR(...) fails to link
// regardless of LTO / --gc-sections elimination. Without this, if no caller
// of begin()/tick() exists yet (mid-refactor or before commit 5 lands the
// wiring) the linker can drop the whole TU and the undefined-reference
// enforcement disappears with it.
//
// __attribute__((constructor)) puts this in .init_array, which the linker
// always keeps because the C++ runtime walks it at startup. The asm
// volatile prevents the compiler from elidating the read of kMainName.
__attribute__((constructor)) static void anchor_kMainName() {
    asm volatile("" : : "r"(kMainName));
}

constexpr uint32_t STA_CONNECT_TIMEOUT_MS = 30000;
constexpr uint32_t OFFLINE_REBOOT_MS      = 30000;
constexpr uint32_t MARK_VALID_DELAY_MS    = 30000;

std::atomic<uint32_t>  g_last_sta_connected_ms{0};
uint32_t           g_boot_ms = 0;
bool               g_marked  = false;
char               g_ap_ssid[48] = {0};

void onWifiEvent(WiFiEvent_t e) {
    switch (e) {
        case ARDUINO_EVENT_WIFI_STA_CONNECTED:
            WiFi.setSleep(false);
            RemoteSerial::println("[wifi] STA associated (PS=NONE)");
            break;
        case ARDUINO_EVENT_WIFI_STA_GOT_IP:
            g_last_sta_connected_ms.store(millis(), std::memory_order_release);
            RemoteSerial::printf("[wifi] STA IP %s\n",
                                 WiFi.localIP().toString().c_str());
            break;
        case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
            RemoteSerial::println("[wifi] STA disconnected");
            break;
        default:
            break;
    }
}

bool tryConnectSta(uint32_t timeout_ms) {
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("[wifi] STA connecting");
    const uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED) {
        if (millis() - start > timeout_ms) {
            Serial.println(" timeout (AP fallback active)");
            return false;
        }
        delay(250);
        Serial.print('.');
    }
    Serial.printf("\n[wifi] STA IP %s\n", WiFi.localIP().toString().c_str());
    // WIFI_PS_NONE: latency-sensitive teleop; reset by kernel on reassociate, also re-applied in onWifiEvent.
    WiFi.setSleep(false);
    g_last_sta_connected_ms.store(millis(), std::memory_order_release);
    return true;
}

void startApAlways() {
    snprintf(g_ap_ssid, sizeof(g_ap_ssid), "%s-recovery", kMainName);
    WiFi.softAPConfig(IPAddress(192, 168, 4, 1),
                      IPAddress(192, 168, 4, 1),
                      IPAddress(255, 255, 255, 0));
    WiFi.softAP(g_ap_ssid, OTA_PASSWORD);
    Serial.printf("[wifi] AP up: SSID=%s IP=%s\n",
                  g_ap_ssid, WiFi.softAPIP().toString().c_str());
}

#endif  // ARDUINO

}  // namespace

#ifdef ARDUINO

void begin() {
    // Force a load of kMainName so the link cannot drop the symbol. Any main
    // missing OTA_SAFE_MODE_FOR(...) fails to link with a clear message.
    Serial.printf("[ota] OtaSafeMode for '%s'\n", kMainName);

    g_boot_ms = millis();
    g_last_sta_connected_ms.store(millis(), std::memory_order_release);

    WiFi.mode(WIFI_AP_STA);
    WiFi.persistent(false);
    WiFi.setAutoReconnect(true);
    WiFi.onEvent(onWifiEvent);

    startApAlways();                       // permanent recovery channel
    tryConnectSta(STA_CONNECT_TIMEOUT_MS); // best-effort; AP covers failure

    // Hostname also keys mDNS discovery and re-anchors kMainName from a
    // second live API call as defense against LTO eliding the printf above.
    ArduinoOTA.setHostname(kMainName);
    ArduinoOTA.setPassword(OTA_PASSWORD);
    ArduinoOTA.onStart(onOtaStart);
    ArduinoOTA.onEnd([]() {
        g_updating.store(false, std::memory_order_release);
        RemoteSerial::println("[ota] update complete; rebooting");
    });
    ArduinoOTA.onError([](ota_error_t e) {
        g_updating.store(false, std::memory_order_release);
        RemoteSerial::printf("[ota] error %u\n", e);
    });
    ArduinoOTA.begin();

    Serial.printf("[ota] rollback supported by bootloader: %s\n",
                  esp_ota_check_rollback_is_possible() ? "yes" : "no");
    Serial.printf("[ota] free heap after init: %u bytes\n",
                  static_cast<unsigned>(ESP.getFreeHeap()));
}

void tick() {
    ArduinoOTA.handle();

    const uint32_t now = millis();

    if (OtaPolicy::shouldMarkValid(now, g_boot_ms, g_marked,
                                   MARK_VALID_DELAY_MS)) {
        if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK) {
            RemoteSerial::println("[ota] firmware marked valid; rollback cancelled.");
        }
        g_marked = true;
    }

    if (WiFi.status() == WL_CONNECTED) {
        g_last_sta_connected_ms.store(now, std::memory_order_release);
    }

    const bool ap_up = (WiFi.getMode() & WIFI_MODE_AP) != 0;
    if (OtaPolicy::shouldRebootForOffline(
            ap_up,
            WiFi.status() == WL_CONNECTED,
            now,
            g_last_sta_connected_ms.load(std::memory_order_acquire),
            OFFLINE_REBOOT_MS)) {
        // Dual-write: ESP.restart() may not flush AsyncTCP, so we shove the
        // FATAL line to both transports and accept that the WebSerial client
        // may or may not receive it before reset.
        RemoteSerial::println("FATAL: no network at all — restarting.");
        ESP.restart();
    }
}

#endif  // ARDUINO

bool isUpdating() {
    return g_updating.load(std::memory_order_acquire);
}

#ifdef BB8_TEST_HOOKS
namespace detail {
void onOtaStartForTest() { onOtaStart(); }
}  // namespace detail
#endif

}  // namespace OtaSafeMode
