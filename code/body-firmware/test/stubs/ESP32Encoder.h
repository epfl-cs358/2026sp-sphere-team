#pragma once
#include <cstdint>

class ESP32Encoder {
public:
    ESP32Encoder() = default;
    void attachFullQuad(int, int) {}
    int64_t getCount() { return _count; }
    void setCount(int64_t v) { _count = v; }
    void clearCount() { _count = 0; }

    // test helper
    int64_t _count = 0;
};
