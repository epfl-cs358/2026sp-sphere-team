#pragma once

#include <Arduino.h>

#include <functional>

class AsyncWebServer;

namespace RemoteSerial {

// Mount the WebSerial UI on an internal AsyncWebServer at
// http://<hostname>.local:81/webserial. Port 81 (not 80) so we don't
// collide with WebSocketCommandProducer in main_robot.cpp. Idempotent.
void begin();

// No-op on the current WebSerial 2.1.x release for ESP32 (transport is
// event-driven via AsyncTCP). Kept symmetric with OtaSafeMode::tick so
// loops can call it unconditionally if WebSerial's buffering mode ever
// needs servicing.
void tick();

void print(const String& s);
void println(const String& s);
void printf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

// Expose the internal AsyncWebServer (port 81) so callers can attach extra
// REST routes alongside WebSerial. Safe to call before or after begin();
// ESPAsyncWebServer iterates its handler list per request, so post-begin
// route registration takes effect on the next inbound request.
AsyncWebServer& server();

// Register a callback invoked when a message arrives from the browser.
// The argument is the raw line as sent (no trim). Replaces any previous
// callback. Call from setup() before begin(); reassigning concurrently
// with inbound browser traffic on the AsyncTCP task is not safe.
void onMessage(std::function<void(const String&)> cb);

}  // namespace RemoteSerial
