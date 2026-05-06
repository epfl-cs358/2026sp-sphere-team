#include <Arduino.h>
#include "pins.h"
#include "L298NDriver.h"
#include "FIT0186Motor.h"
#include "OmniDrivetrain.h"
#include "PID.h"

static constexpr unsigned long LOOP_INTERVAL_MS = 10;

static DrivetrainConfig config = {
    .wheelRadius = 0.03f,   // placeholder — update with measured value
    .robotRadius = 0.08f,   // placeholder — update with measured value
    .tiltAngle = 0.0f,      // placeholder — update with measured value
    .maxRPM = 251.0f
};

static L298NDriver driver0(MOTOR0_PINS);
static L298NDriver driver1(MOTOR1_PINS);
static L298NDriver driver2(MOTOR2_PINS);

static FIT0186Motor<5> motor0(driver0, MOTOR0_ENCODER);
static FIT0186Motor<5> motor1(driver1, MOTOR1_ENCODER);
static FIT0186Motor<5> motor2(driver2, MOTOR2_ENCODER);

static PID pid0(1.0f, 0.1f, 0.01f, -1.0f, 1.0f);
static PID pid1(1.0f, 0.1f, 0.01f, -1.0f, 1.0f);
static PID pid2(1.0f, 0.1f, 0.01f, -1.0f, 1.0f);

static OmniDrivetrain drivetrain(motor0, motor1, motor2, config, pid0, pid1, pid2);

static unsigned long lastLoopTime = 0;

void setup() {
    Serial.begin(115200);
    motor0.begin();
    motor1.begin();
    motor2.begin();
    lastLoopTime = millis();
    Serial.println("Drivetrain test ready. Send: <vx> <vy> <omega_deg>");
    Serial.println("Send 'stop' to brake all.");
}

void loop() {
    unsigned long now = millis();
    if (now - lastLoopTime >= LOOP_INTERVAL_MS) {
        drivetrain.update(static_cast<float>(now - lastLoopTime) / 1000.0f);
        lastLoopTime = now;
    }

    if (!Serial.available()) return;

    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line == "stop") {
        drivetrain.stop();
        Serial.println("All motors braked.");
        return;
    }

    int firstSpace = line.indexOf(' ');
    int secondSpace = line.indexOf(' ', firstSpace + 1);

    if (firstSpace < 0 || secondSpace < 0) {
        Serial.println("ERR: format is <vx> <vy> <omega_deg>");
        return;
    }

    float vx = line.substring(0, firstSpace).toFloat();
    float vy = line.substring(firstSpace + 1, secondSpace).toFloat();
    float omegaDeg = line.substring(secondSpace + 1).toFloat();
    float omega = omegaDeg * static_cast<float>(M_PI) / 180.0f;

    BodyVelocity vel = {vx, vy, omega};
    drivetrain.drive(vel);
    Serial.printf("Drive: vx=%.2f vy=%.2f omega=%.1f deg/s\n", vx, vy, omegaDeg);
    Serial.printf("RPM: m0=%.1f m1=%.1f m2=%.1f\n",
                  motor0.getFilteredRPM(), motor1.getFilteredRPM(), motor2.getFilteredRPM());
}
