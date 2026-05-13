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
    cfg.ledc_timer   = LEDC_TIMER_0;
    cfg.ledc_channel = LEDC_CHANNEL_0;
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

// ── MJPEG stream handler ──────────────────────────────────────────────────────
static void handleStream() {
    WiFiClient client = httpServer.client();

    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: multipart/x-mixed-replace; boundary=frame");
    client.println("Access-Control-Allow-Origin: *");
    client.println();

    while (client.connected()) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) continue;

        client.printf("--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                      fb->len);
        client.write(fb->buf, fb->len);
        client.println();
        esp_camera_fb_return(fb);
    }
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
