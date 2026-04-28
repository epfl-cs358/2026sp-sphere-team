/*
 * BB-8 Head Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini, with assistance from Claude (Anthropic)
 */

#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include "StreamConfig.h"

inline const char* resolutionToString(Resolution r) {
    switch (r) {
        case Resolution::VGA:  return "VGA";
        case Resolution::SVGA: return "SVGA";
        case Resolution::XGA:  return "XGA";
        case Resolution::HD:   return "HD";
        case Resolution::UXGA: return "UXGA";
    }
    return "VGA";
}

inline bool stringToResolution(const char* str, size_t len, Resolution& out) {
    if (len == 3 && strncmp(str, "VGA", 3) == 0)  { out = Resolution::VGA;  return true; }
    if (len == 4 && strncmp(str, "SVGA", 4) == 0) { out = Resolution::SVGA; return true; }
    if (len == 3 && strncmp(str, "XGA", 3) == 0)  { out = Resolution::XGA;  return true; }
    if (len == 2 && strncmp(str, "HD", 2) == 0)   { out = Resolution::HD;   return true; }
    if (len == 4 && strncmp(str, "UXGA", 4) == 0) { out = Resolution::UXGA; return true; }
    return false;
}

inline void buildStatusJson(char* buf, size_t bufLen, const StreamConfig& cfg,
                            int rssi, unsigned long uptimeMs, uint8_t clients) {
    snprintf(buf, bufLen,
        R"({"resolution":"%s","fps":%u,"quality":%u,"rssi":%d,"uptime":%lu,"clients":%u})",
        resolutionToString(cfg.resolution),
        cfg.fps, cfg.quality, rssi, uptimeMs, clients);
}

inline bool parseConfigJson(const char* json, size_t len, StreamConfig& cfg) {
    if (len == 0 || json[0] != '{') return false;

    // Parse "resolution":"VALUE"
    const char* rKey = strstr(json, "\"resolution\":\"");
    if (rKey) {
        const char* rVal = rKey + 14; // skip "resolution":"
        const char* rEnd = strchr(rVal, '"');
        if (rEnd) {
            Resolution r;
            if (stringToResolution(rVal, rEnd - rVal, r)) {
                cfg.resolution = r;
            }
        }
    }

    // Parse "fps":NUMBER
    const char* fKey = strstr(json, "\"fps\":");
    if (fKey) {
        const char* fVal = fKey + 6;
        int v = atoi(fVal);
        cfg.fps = StreamConfig::clampFps(static_cast<uint8_t>(v));
    }

    // Parse "quality":NUMBER
    const char* qKey = strstr(json, "\"quality\":");
    if (qKey) {
        const char* qVal = qKey + 10;
        int v = atoi(qVal);
        cfg.quality = StreamConfig::clampQuality(static_cast<uint8_t>(v));
    }

    return true;
}
