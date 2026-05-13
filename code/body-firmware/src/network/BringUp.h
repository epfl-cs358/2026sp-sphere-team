#pragma once

// Shared bring-up for every main_*.cpp that wants OTA + WebSerial console.
// Collapses the boilerplate that used to live at the top of each main:
//
//     Serial.begin(115200);
//     OtaSafeMode::begin();
//     RemoteSerial::begin();
//
// and the matching pair of ::tick() calls in loop(), into a single
// BringUp::begin() / BringUp::tick(). Re-exports the OTA_SAFE_MODE_FOR
// macro (still required at file scope, see OtaSafeMode.h) and the
// OtaSafeMode::isUpdating() check so consumers only need this one header.

#include "OtaSafeMode.h"
#include "RemoteSerial.h"

namespace BringUp {

// Order: Serial.begin → USB-CDC settle delay → OtaSafeMode::begin →
// RemoteSerial::begin. The settle delay matters because OtaSafeMode::begin
// prints WiFi state to Serial immediately; without it the first ~100ms of
// logs are lost on USB-CDC.
void begin(unsigned long baud = 115200);

// Calls OtaSafeMode::tick() then RemoteSerial::tick(). Cheap; call once
// per loop() iteration (and inside any blocking wait loop in setup).
void tick();

}  // namespace BringUp
