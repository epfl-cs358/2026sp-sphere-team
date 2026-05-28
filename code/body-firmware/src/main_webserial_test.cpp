#include <Arduino.h>
#include "BringUp.h"

OTA_SAFE_MODE_FOR("bb8-robot");

void setup() {
    BringUp::begin();

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
    BringUp::tick();
    vTaskDelay(pdMS_TO_TICKS(100));
}
