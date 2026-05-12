#include <Arduino.h>
#include "pins.h"
#include "L298NDriver.h"
#include "FIT0186Motor.h"
#include "OmniDrivetrain.h"
#include "RobotConstants.h"
#include "PID.h"

#include "OtaSafeMode.h"
OTA_SAFE_MODE_FOR("bb8-drivetrain-test");

static constexpr unsigned long LOOP_INTERVAL_MS = 10;
static constexpr unsigned long PRINT_INTERVAL_MS = 200;

static DrivetrainConfig config = RobotConstants::drivetrainConfig();

static L298NDriver driver0(MOTOR0_PINS);
static L298NDriver driver1(MOTOR1_PINS);
static L298NDriver driver2(MOTOR2_PINS);

static FIT0186Motor<5> motor0(driver0, MOTOR0_ENCODER);
static FIT0186Motor<5> motor1(driver1, MOTOR1_ENCODER);
static FIT0186Motor<5> motor2(driver2, MOTOR2_ENCODER);

static PID pid0(0.005f, 0.002f, 0.0005f, -1.0f, 1.0f, 5.0f);
static PID pid1(0.005f, 0.002f, 0.0005f, -1.0f, 1.0f, 5.0f);
static PID pid2(0.005f, 0.002f, 0.0005f, -1.0f, 1.0f, 5.0f);

static OmniDrivetrain drivetrain(motor0, motor1, motor2, config, pid0, pid1, pid2);

static unsigned long lastLoopTime = 0;
static unsigned long lastPrintTime = 0;
static bool showRPM = false;
static bool driving = false;

void setup() {
    Serial.begin(115200);
    motor0.begin();
    motor1.begin();
    motor2.begin();
    lastLoopTime = millis();
    Serial.println("Drivetrain test ready.");
    Serial.println("Commands:");
    Serial.println("  <vx> <vy> <omega_deg>  — drive at body velocity");
    Serial.println("  stop                   — brake all motors");
    Serial.println("  rpm                    — toggle RPM display");
}

void loop() {
    unsigned long now = millis();
    if (now - lastLoopTime >= LOOP_INTERVAL_MS) {
        drivetrain.update(static_cast<float>(now - lastLoopTime) / 1000.0f);
        lastLoopTime = now;
    }

    if (showRPM && (now - lastPrintTime >= PRINT_INTERVAL_MS)) {
        lastPrintTime = now;
        auto target = drivetrain.getTargetRPMs();
        Serial.printf("T: %7.1f %7.1f %7.1f | A: %7.1f %7.1f %7.1f\n",
                      target[0], target[1], target[2],
                      motor0.getFilteredRPM(), motor1.getFilteredRPM(), motor2.getFilteredRPM());
    }

    if (!Serial.available()) return;

    String line = Serial.readStringUntil('\n');
    line.trim();

    if (line == "stop") {
        drivetrain.stop();
        driving = false;
        Serial.println("All motors braked.");
        return;
    }

    if (line == "rpm") {
        showRPM = !showRPM;
        Serial.printf("RPM display %s\n", showRPM ? "ON" : "OFF");
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
    driving = true;
    Serial.printf("Drive: vx=%.2f vy=%.2f omega=%.1f deg/s\n", vx, vy, omegaDeg);
}
