#include "RemoteSerial.h"

#include <ESPAsyncWebServer.h>
#include <WebSerial.h>

#include <cstdarg>
#include <cstdio>

namespace {

// Port 81 — port 80 is owned by WebSocketCommandProducer in main_robot.cpp.
AsyncWebServer g_server(81);

std::function<void(const String&)> g_user_cb;

bool g_started = false;

}  // namespace

namespace RemoteSerial {

void begin() {
    if (g_started) {
        return;
    }
    g_started = true;

    WebSerial.begin(&g_server);
    WebSerial.onMessage([](uint8_t* data, size_t len) {
        if (!g_user_cb) {
            return;
        }
        String msg;
        msg.reserve(len);
        for (size_t i = 0; i < len; ++i) {
            msg += static_cast<char>(data[i]);
        }
        g_user_cb(msg);
    });
    g_server.begin();

    Serial.println("[remote-serial] mounted at /webserial on port 81");
}

void tick() {
    // No-op: WebSerial 2.1.x on ESP32 is event-driven via AsyncTCP.
}

void print(const String& s) {
    Serial.print(s);
    WebSerial.print(s);
}

void println(const String& s) {
    Serial.println(s);
    WebSerial.println(s);
}

void printf(const char* fmt, ...) {
    char buf[256];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n < 0) {
        return;
    }
    if (static_cast<size_t>(n) >= sizeof(buf)) {
        // Truncated: replace last 3 bytes before NUL with "..." so the
        // reader sees the cut explicitly instead of a silent slice.
        buf[sizeof(buf) - 4] = '.';
        buf[sizeof(buf) - 3] = '.';
        buf[sizeof(buf) - 2] = '.';
        buf[sizeof(buf) - 1] = '\0';
    }
    Serial.print(buf);
    WebSerial.print(buf);
}

void onMessage(std::function<void(const String&)> cb) {
    g_user_cb = std::move(cb);
}

}  // namespace RemoteSerial
