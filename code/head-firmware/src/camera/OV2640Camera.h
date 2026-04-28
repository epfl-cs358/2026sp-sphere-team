/*
 * BB-8 Head Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini, with assistance from Claude (Anthropic)
 */

#pragma once

#include <esp_camera.h>
#include "Camera.h"
#include "StreamConfig.h"

class OV2640Camera : public Camera {
public:
    bool begin(const StreamConfig& config) override {
        camera_config_t camCfg = {};

        // XIAO ESP32S3 Sense pin mapping
        camCfg.pin_pwdn = -1;
        camCfg.pin_reset = -1;
        camCfg.pin_xclk = 10;
        camCfg.pin_sccb_sda = 40;
        camCfg.pin_sccb_scl = 39;
        camCfg.pin_d7 = 48;
        camCfg.pin_d6 = 11;
        camCfg.pin_d5 = 12;
        camCfg.pin_d4 = 14;
        camCfg.pin_d3 = 16;
        camCfg.pin_d2 = 18;
        camCfg.pin_d1 = 17;
        camCfg.pin_d0 = 15;
        camCfg.pin_vsync = 38;
        camCfg.pin_href = 47;
        camCfg.pin_pclk = 13;

        camCfg.xclk_freq_hz = 20000000;
        camCfg.ledc_timer = LEDC_TIMER_0;
        camCfg.ledc_channel = LEDC_CHANNEL_0;
        camCfg.pixel_format = PIXFORMAT_JPEG;
        camCfg.frame_size = toFrameSize(config.resolution);
        camCfg.jpeg_quality = config.quality;
        camCfg.fb_count = 2;
        camCfg.fb_location = CAMERA_FB_IN_PSRAM;
        camCfg.grab_mode = CAMERA_GRAB_LATEST;

        esp_err_t err = esp_camera_init(&camCfg);
        if (err != ESP_OK) return false;

        _initialized = true;
        return true;
    }

    bool applyConfig(const StreamConfig& config) override {
        if (!_initialized) return false;

        sensor_t* sensor = esp_camera_sensor_get();
        if (!sensor) return false;

        sensor->set_framesize(sensor, toFrameSize(config.resolution));
        sensor->set_quality(sensor, config.quality);
        return true;
    }

    const uint8_t* capture(size_t& outLen) override {
        if (_fb) esp_camera_fb_return(_fb);
        _fb = esp_camera_fb_get();
        if (!_fb) {
            outLen = 0;
            return nullptr;
        }
        outLen = _fb->len;
        return _fb->buf;
    }

    void release() override {
        if (_fb) {
            esp_camera_fb_return(_fb);
            _fb = nullptr;
        }
    }

private:
    bool _initialized = false;
    camera_fb_t* _fb = nullptr;

    static framesize_t toFrameSize(Resolution r) {
        switch (r) {
            case Resolution::VGA:  return FRAMESIZE_VGA;
            case Resolution::SVGA: return FRAMESIZE_SVGA;
            case Resolution::XGA:  return FRAMESIZE_XGA;
            case Resolution::HD:   return FRAMESIZE_HD;
            case Resolution::UXGA: return FRAMESIZE_UXGA;
        }
        return FRAMESIZE_VGA;
    }
};
