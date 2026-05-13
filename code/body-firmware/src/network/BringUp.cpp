#include "BringUp.h"

#include <Arduino.h>

namespace BringUp {

void begin(unsigned long baud) {
    Serial.begin(baud);
    delay(200);
    OtaSafeMode::begin();
    RemoteSerial::begin();
}

void tick() {
    OtaSafeMode::tick();
    RemoteSerial::tick();
}

}  // namespace BringUp
