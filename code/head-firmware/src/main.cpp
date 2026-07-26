#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ESP32Servo.h>
#include "esp_camera.h"

#include "wifi_credentials.h"
#include "pins.h"
#include "ServoController.h"

// ── Camera pin map: Seeed XIAO ESP32-S3 Sense (OV2640) ───────────────────────
static camera_config_t camConfig() {
    camera_config_t cfg;
    cfg.pin_pwdn     = -1;
    cfg.pin_reset    = -1;
    cfg.pin_xclk     = 10;
    cfg.pin_sccb_sda = 40;
    cfg.pin_sccb_scl = 39;
    cfg.pin_d7 = 48; cfg.pin_d6 = 11; cfg.pin_d5 = 12; cfg.pin_d4 = 14;
    cfg.pin_d3 = 16; cfg.pin_d2 = 18; cfg.pin_d1 = 17; cfg.pin_d0 = 15;
    cfg.pin_vsync = 38;
    cfg.pin_href  = 47;
    cfg.pin_pclk  = 13;
    cfg.xclk_freq_hz = 20000000;
    // Use LEDC_TIMER_1 / LEDC_CHANNEL_2 so the servo (which ESP32Servo assigns
    // to channels 0-1) does not collide with the camera's XCLK generator.
    cfg.ledc_timer   = LEDC_TIMER_1;
    cfg.ledc_channel = LEDC_CHANNEL_2;
    cfg.pixel_format = PIXFORMAT_JPEG;
    cfg.frame_size   = FRAMESIZE_VGA;
    cfg.jpeg_quality = 12;
    cfg.fb_count     = 2;
    cfg.grab_mode    = CAMERA_GRAB_LATEST;
    return cfg;
}

// ── Real servo adapter: bridges IServo → ESP32Servo SDK ──────────────────────
class ESP32ServoAdapter : public IServo {
public:
    void attach(int pin) override { _servo.attach(pin); }
    void write(int angle) override { _servo.write(angle); }
private:
    Servo _servo;
};

// ── Globals ───────────────────────────────────────────────────────────────────
static ESP32ServoAdapter servoAdapter;
static ServoController   servoCtrl(servoAdapter, SERVO_PIN);

static WebServer         httpServer(80);
static WebSocketsServer  wsServer(81);

// Active streaming client; written by handleStream(), read by streamTask().
static WiFiClient streamClient;
static volatile bool streaming = false;

// ── Stream task: runs on Core 0, sends MJPEG frames without blocking loop() ──
static void streamTask(void*) {
    for (;;) {
        if (!streaming || !streamClient.connected()) {
            streaming = false;
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) { vTaskDelay(1); continue; }
        streamClient.printf("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                            fb->len);
        streamClient.write(fb->buf, fb->len);
        streamClient.println();
        esp_camera_fb_return(fb);
    }
}

// ── MJPEG stream handler: sends headers and hands off to streamTask ───────────
static void handleStream() {
    streamClient = httpServer.client();
    streamClient.println("HTTP/1.1 200 OK");
    streamClient.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
    streamClient.println("Access-Control-Allow-Origin: *");
    streamClient.println();
    streaming = true;
    // Returns immediately; streamTask drives the frame loop on Core 0.
}

// ── WebSocket event handler ───────────────────────────────────────────────────
static void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t) {
    if (type == WStype_TEXT) {
        servoCtrl.handleCommand(reinterpret_cast<const char*>(payload));
    }
}

// ── setup / loop ─────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);

    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print('.'); }
    Serial.printf("\nIP: %s\n", WiFi.localIP().toString().c_str());

    camera_config_t cfg = camConfig();
    if (esp_camera_init(&cfg) != ESP_OK) {
        Serial.println("Camera init failed");
        return;
    }

    servoCtrl.begin();

    xTaskCreatePinnedToCore(streamTask, "stream", 4096, nullptr, 1, nullptr, 0);

    httpServer.on("/stream", handleStream);
    httpServer.begin();
    Serial.println("MJPEG stream: http://" + WiFi.localIP().toString() + "/stream");

    wsServer.begin();
    wsServer.onEvent(onWsEvent);
    Serial.println("WebSocket server: ws://" + WiFi.localIP().toString() + ":81");
}

void loop() {
    wsServer.loop();
    httpServer.handleClient();
}
