/*
 * BB-8 Head Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini, with assistance from Claude (Anthropic)
 */

#pragma once

#include <cstdint>

enum class Resolution : uint8_t { VGA, SVGA, XGA, HD, UXGA };

struct StreamConfig {
    Resolution resolution = Resolution::VGA;
    uint8_t fps = 20;
    uint8_t quality = 80;

    static constexpr uint8_t MIN_FPS = 1;
    static constexpr uint8_t MAX_FPS = 30;
    static constexpr uint8_t MIN_QUALITY = 10;
    static constexpr uint8_t MAX_QUALITY = 100;

    static uint8_t clampFps(uint8_t v) {
        if (v < MIN_FPS) return MIN_FPS;
        if (v > MAX_FPS) return MAX_FPS;
        return v;
    }

    static uint8_t clampQuality(uint8_t v) {
        if (v < MIN_QUALITY) return MIN_QUALITY;
        if (v > MAX_QUALITY) return MAX_QUALITY;
        return v;
    }

    static uint16_t width(Resolution r) {
        switch (r) {
            case Resolution::VGA:  return 640;
            case Resolution::SVGA: return 800;
            case Resolution::XGA:  return 1024;
            case Resolution::HD:   return 1280;
            case Resolution::UXGA: return 1600;
        }
        return 640;
    }

    static uint16_t height(Resolution r) {
        switch (r) {
            case Resolution::VGA:  return 480;
            case Resolution::SVGA: return 600;
            case Resolution::XGA:  return 768;
            case Resolution::HD:   return 720;
            case Resolution::UXGA: return 1200;
        }
        return 480;
    }
};
