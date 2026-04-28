/*
 * BB-8 Head Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini, with assistance from Claude (Anthropic)
 */

#pragma once

#include <WiFi.h>
#include <ESPmDNS.h>
#include "WifiProvisioner.h"
#include "wifi_credentials.h"

class CaptivePortalWifi : public WifiProvisioner {
public:
    bool begin() override {
        WiFi.disconnect(true, true);
        delay(100);
        WiFi.setHostname("sphere-head");
        WiFi.begin("Alessandro", "00000000");

        Serial.printf("Connecting to %s", WIFI_SSID);
        unsigned long start = millis();
        while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
            delay(250);
            Serial.print(".");
        }
        Serial.println();

        if (WiFi.status() != WL_CONNECTED) {
            Serial.printf("WiFi failed, status=%d\n", WiFi.status());
            return false;
        }

        WiFi.setAutoReconnect(true);

        if (!MDNS.begin("sphere-head")) {
            Serial.println("mDNS setup failed");
        } else {
            Serial.println("mDNS: sphere-head.local");
        }

        _ip = WiFi.localIP().toString();
        return true;
    }

    bool isConnected() override {
        return WiFi.status() == WL_CONNECTED;
    }

    int getRSSI() override {
        return WiFi.RSSI();
    }

    const char* getIP() override {
        return _ip.c_str();
    }

private:
    String _ip;
};
