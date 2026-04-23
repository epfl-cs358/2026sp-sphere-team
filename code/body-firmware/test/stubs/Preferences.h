#pragma once
#include <cstdint>
#include <cstddef>
#include <fff.h>

DECLARE_FAKE_VALUE_FUNC(bool, prefs_begin, const char*, bool);
DECLARE_FAKE_VALUE_FUNC(size_t, prefs_getBytes, const char*, void*, size_t);
DECLARE_FAKE_VALUE_FUNC(size_t, prefs_putBytes, const char*, const void*, size_t);
DECLARE_FAKE_VOID_FUNC(prefs_end);

class Preferences {
public:
    bool begin(const char* name, bool readOnly = false) { return prefs_begin(name, readOnly); }
    size_t getBytes(const char* key, void* buf, size_t maxLen) { return prefs_getBytes(key, buf, maxLen); }
    size_t putBytes(const char* key, const void* value, size_t len) { return prefs_putBytes(key, value, len); }
    void end() { prefs_end(); }
};

#define RESET_PREFERENCES_FAKES() do { \
    RESET_FAKE(prefs_begin); \
    RESET_FAKE(prefs_getBytes); \
    RESET_FAKE(prefs_putBytes); \
    RESET_FAKE(prefs_end); \
    FFF_RESET_HISTORY(); \
} while(0)
