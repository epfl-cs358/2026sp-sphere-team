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
        // Strip trailing C-string null terminators and line endings — some
        // WebSerial clients (including the official frontend on certain
        // browsers) append a 0x00 to text payloads. Arduino String::trim()
        // doesn't remove 0x00, so without this `equals("save")` against
        // a `"save\0"` message returns false (length mismatch) and the
        // dispatcher misroutes the verb.
        while (len > 0) {
            uint8_t b = data[len - 1];
            if (b == 0x00 || b == '\r' || b == '\n') {
                --len;
                continue;
            }
            break;
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
    // WebSerial 2.x batches writes in a two-tier print buffer and only
    // flushes either on buffer-fill or when `loop()` observes the flush
    // timer has elapsed (WSL_PRINT_FLUSH_TIME_US / WSL_GLOBAL_FLUSH_TIME_MS).
    // Without periodic pumping, a single println sits in the buffer until
    // the next write triggers `loop()` inside `write()` — which is why
    // operators saw a one-message lag (response visible only on the NEXT
    // command). Calling loop() here at the main-loop cadence drains the
    // buffer at roughly the LOOP_TICK_MS rate.
    WebSerial.loop();
}

// Pre-begin() calls (e.g. wifi event callbacks firing during the initial STA
// associate inside OtaSafeMode::begin()) fall back to Serial-only — calling
// WebSerial.print() before WebSerial.begin() is undefined-behaviour territory.
void print(const String& s) {
    Serial.print(s);
    if (g_started) WebSerial.print(s);
}

void println(const String& s) {
    Serial.println(s);
    if (g_started) WebSerial.println(s);
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
    if (g_started) WebSerial.print(buf);
}

void onMessage(std::function<void(const String&)> cb) {
    g_user_cb = std::move(cb);
}

AsyncWebServer& server() {
    return g_server;
}

}  // namespace RemoteSerial
