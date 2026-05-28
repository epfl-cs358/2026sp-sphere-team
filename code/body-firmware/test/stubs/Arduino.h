#pragma once
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>

using byte = uint8_t;

// Minimal Arduino String shim for native tests. Wraps std::string with
// just enough of the Arduino API surface for command-parsing code.
class String : public std::string {
public:
    String() = default;
    String(const char* s) : std::string(s ? s : "") {}
    String(const std::string& s) : std::string(s) {}
    String(char c) : std::string(1, c) {}

    int    indexOf(char c) const            { auto p = find(c); return p == npos ? -1 : static_cast<int>(p); }
    int    indexOf(char c, int from) const  { auto p = find(c, static_cast<size_t>(from)); return p == npos ? -1 : static_cast<int>(p); }
    String substring(int from) const        { return String(std::string(this->substr(static_cast<size_t>(from)))); }
    String substring(int from, int to) const { return String(std::string(this->substr(static_cast<size_t>(from), static_cast<size_t>(to - from)))); }
    bool   startsWith(const String& s) const { return rfind(s, 0) == 0; }
    bool   equals(const String& s) const    { return static_cast<const std::string&>(*this) == static_cast<const std::string&>(s); }
    void   trim() {
        size_t a = 0;
        while (a < size() && std::isspace(static_cast<unsigned char>((*this)[a]))) ++a;
        size_t b = size();
        while (b > a && std::isspace(static_cast<unsigned char>((*this)[b - 1]))) --b;
        *this = String(std::string(this->substr(a, b - a)));
    }
    float  toFloat() const                  { return std::strtof(c_str(), nullptr); }
};

#define OUTPUT 0x01
#define HIGH   0x01
#define LOW    0x00

inline unsigned long _micros_value = 0;
inline unsigned long micros() { return _micros_value; }
inline void _set_micros(unsigned long v) { _micros_value = v; }
inline void delay(unsigned long) {}

#ifdef ARDUINO_GPIO_FAKES
#include <fff.h>
DECLARE_FAKE_VOID_FUNC(pinMode, uint8_t, uint8_t);
DECLARE_FAKE_VOID_FUNC(digitalWrite, uint8_t, uint8_t);
DECLARE_FAKE_VOID_FUNC(analogWrite, uint8_t, uint8_t);

#define RESET_ARDUINO_FAKES() do { \
    RESET_FAKE(pinMode); \
    RESET_FAKE(digitalWrite); \
    RESET_FAKE(analogWrite); \
    FFF_RESET_HISTORY(); \
} while(0)
#else
inline void pinMode(uint8_t, uint8_t) {}
inline void digitalWrite(uint8_t, uint8_t) {}
inline void analogWrite(uint8_t, uint8_t) {}
#endif
