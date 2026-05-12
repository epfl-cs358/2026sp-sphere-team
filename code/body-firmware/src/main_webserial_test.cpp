#include <Arduino.h>
#include "OtaSafeMode.h"
#include "RemoteSerial.h"

OTA_SAFE_MODE_FOR("bb8-robot");

void setup() {
    Serial.begin(115200);
    delay(200);
    OtaSafeMode::begin();
    RemoteSerial::begin();

    RemoteSerial::onMessage([](const String& raw) {
        String msg = raw;
        msg.trim();
        if (msg == "ping") {
            RemoteSerial::println("pong");
        } else {
            RemoteSerial::printf("got: %s\n", msg.c_str());
        }
    });

    RemoteSerial::println("ready");
}

void loop() {
    OtaSafeMode::tick();
    RemoteSerial::tick();
    vTaskDelay(pdMS_TO_TICKS(100));
}
