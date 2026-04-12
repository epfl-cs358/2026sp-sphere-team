# Body Firmware — Library Reference

## Required

| Library | PlatformIO Name | Purpose |
|---------|----------------|---------|
| Adafruit BNO055 | `adafruit/Adafruit BNO055` | IMU driver — orientation, accel, gyro, magnetometer fusion, calibration |
| Arduino-BTS7960 | `luisllamasbinaburo/Arduino-BTS7960` | H-bridge motor driver control (PWM + direction) |
| ESP32Encoder | `madhephaestus/ESP32Encoder` | Hardware quadrature encoder reading for motor RPM feedback |
| ESPAsyncWebServer | `esp32async/ESPAsyncWebServer` | Async HTTP + WebSocket server for receiving drive commands |
| ArduinoJson | `bblanchon/ArduinoJson` | JSON parsing/serialization for WebSocket messages |

## Optional / Evaluate Later

| Library | PlatformIO Name | Purpose |
|---------|----------------|---------|
| PID | `br3ttb/PID` | Established PID controller — may roll our own instead |
| QuickPID | `dlloydev/QuickPID` | Faster PID alternative with anti-windup |
| ESP32_MCPWM | `littlemanbuilds/ESP32_MCPWM` | ESP32 hardware MCPWM peripheral — alternative to BTS7960 lib |

## Built-in (no dependency needed)

| Library | Purpose |
|---------|---------|
| FreeRTOS | Task scheduling, dual-core pinning, semaphores — included in ESP32 Arduino |
| WiFi | ESP32 WiFi — included in ESP32 Arduino |

## No library available

| Feature | Notes |
|---------|-------|
| 3-wheel omni kinematics | Custom implementation needed — decompose (vx, vy, omega) into 3 wheel speeds at 120° |
