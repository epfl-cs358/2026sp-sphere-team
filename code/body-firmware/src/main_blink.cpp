#include <Arduino.h>

const uint8_t pins[] = {14, 27, 16, 17, 18, 19};
const int NUM_PINS = sizeof(pins) / sizeof(pins[0]);

void setup() {
    Serial.begin(115200);
    for (int i = 0; i < NUM_PINS; i++) {
        pinMode(pins[i], OUTPUT);
    }
}

void loop() {
    for (int i = 0; i < NUM_PINS; i += 2) {
        delay(1500);
        analogWrite(pins[i], 128);
        Serial.printf("Motor %d (pin %d): ON\n", i / 2, pins[i]);

        delay(1500);
        analogWrite(pins[i], 0);
        Serial.printf("Motor %d (pin %d): OFF\n", i / 2, pins[i]);
    }
}
