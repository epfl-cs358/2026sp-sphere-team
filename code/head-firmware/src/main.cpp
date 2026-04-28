/*
 * BB-8 Head Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini, with assistance from Claude (Anthropic)
 */

#include <Arduino.h>

#include "pins.h"
#include "StreamConfig.h"
#include "OV2640Camera.h"
#include "CaptivePortalWifi.h"
#include "AsyncStreamServer.h"
#include "OnboardLED.h"

static StreamConfig config;
static OV2640Camera camera;
static CaptivePortalWifi wifi;
static OnboardLED led(PIN_LED);

static unsigned long startTime = 0;
static bool cameraReady = false;
static uint32_t frameCount = 0;

static int getRSSI() { return wifi.getRSSI(); }
static unsigned long getUptime() { return millis() - startTime; }

static AsyncStreamServer server(config, getRSSI, getUptime);

void setup() {
    Serial.begin(115200);
    delay(2000); // wait for USB CDC serial
    startTime = millis();

    Serial.println("\n=== BB-8 Head Firmware ===");

    led.begin();
    led.setState(LEDState::Connecting);
    led.update(millis());
    Serial.println("Starting WiFi provisioning...");
    if (!wifi.begin()) {
        Serial.println("WiFi connection failed");
        led.setState(LEDState::Error);
        led.update(millis());
        return;
    }
    Serial.printf("IP: %s\n", wifi.getIP());
    led.setState(LEDState::Ready);
    led.update(millis());

    // Start server before camera so /status is always reachable
    server.begin(80);
    Serial.println("Server started on port 80");

    if (!camera.begin(config)) {
        Serial.println("Camera init FAILED");
    } else {
        cameraReady = true;
        Serial.println("Camera ready");
    }

    server.onConfigChange([](const StreamConfig& newConfig) {
        if (cameraReady) camera.applyConfig(newConfig);
    });

    Serial.println("Head firmware ready");
}

void loop() {
    led.update(millis());

    if (!wifi.isConnected()) {
        led.setState(LEDState::Error);
        return;
    }

    if (!cameraReady || server.connectedClients() == 0) {
        led.setState(LEDState::Ready);
        return;
    }

    if (!server.frameRequested()) return;

    led.setState(LEDState::Streaming);

    size_t len = 0;
    const uint8_t* frame = camera.capture(len);
    if (frame && len > 0) {
        server.broadcastFrame(frame, len);
        frameCount++;
        if (frameCount % 100 == 0) {
            Serial.printf("Frame %u, %zu bytes, %u clients\n",
                frameCount, len, server.connectedClients());
        }
    }
    camera.release();
}
