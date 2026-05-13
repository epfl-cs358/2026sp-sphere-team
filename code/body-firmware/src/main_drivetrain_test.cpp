#include <Arduino.h>
#include "pins.h"
#include "L298NDriver.h"
#include "FIT0186Motor.h"
#include "OmniDrivetrain.h"
#include "RobotConstants.h"
#include "PID.h"

#include "BringUp.h"
OTA_SAFE_MODE_FOR("bb8-robot");

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

static void handleCommand(const String& raw) {
    String line = raw;
    line.trim();
    if (line.length() == 0) return;

    if (line == "stop") {
        drivetrain.stop();
        driving = false;
        RemoteSerial::println("All motors braked.");
        return;
    }

    if (line == "rpm") {
        showRPM = !showRPM;
        RemoteSerial::printf("RPM display %s\n", showRPM ? "ON" : "OFF");
        return;
    }

    int firstSpace = line.indexOf(' ');
    int secondSpace = line.indexOf(' ', firstSpace + 1);

    if (firstSpace < 0 || secondSpace < 0) {
        RemoteSerial::println("ERR: format is <vx> <vy> <omega_deg>");
        return;
    }

    float vx = line.substring(0, firstSpace).toFloat();
    float vy = line.substring(firstSpace + 1, secondSpace).toFloat();
    float omegaDeg = line.substring(secondSpace + 1).toFloat();
    float omega = omegaDeg * static_cast<float>(M_PI) / 180.0f;

    BodyVelocity vel = {vx, vy, omega};
    drivetrain.drive(vel);
    driving = true;
    RemoteSerial::printf("Drive: vx=%.2f vy=%.2f omega=%.1f deg/s\n", vx, vy, omegaDeg);
}

void setup() {
    BringUp::begin();

    // Bring each motor up AND force its PWM channels to zero before the loop
    // runs. begin() only configures pinMode; setSpeed(0.0f) guarantees both
    // fwd/rev pins are written to 0 so no stale LEDC state from before the
    // soft-reset can leak into the wheels.
    motor0.begin(); motor0.setSpeed(0.0f);
    motor1.begin(); motor1.setSpeed(0.0f);
    motor2.begin(); motor2.setSpeed(0.0f);

    RemoteSerial::onMessage(handleCommand);

    lastLoopTime = millis();
    RemoteSerial::println("Drivetrain test ready.");
    RemoteSerial::println("Commands:");
    RemoteSerial::println("  <vx> <vy> <omega_deg>  — drive at body velocity");
    RemoteSerial::println("  stop                   — brake all motors");
    RemoteSerial::println("  rpm                    — toggle RPM display");
}

void loop() {
    BringUp::tick();

    // Hard-zero PWM while flash is being overwritten so the wheels can't keep
    // chewing on a stale target through the upload window.
    if (OtaSafeMode::isUpdating()) {
        motor0.setSpeed(0.0f);
        motor1.setSpeed(0.0f);
        motor2.setSpeed(0.0f);
        driving = false;
        return;
    }

    unsigned long now = millis();
    if (now - lastLoopTime >= LOOP_INTERVAL_MS) {
        drivetrain.update(static_cast<float>(now - lastLoopTime) / 1000.0f);
        lastLoopTime = now;
    }

    if (showRPM && (now - lastPrintTime >= PRINT_INTERVAL_MS)) {
        lastPrintTime = now;
        auto target = drivetrain.getTargetRPMs();
        RemoteSerial::printf("T: %7.1f %7.1f %7.1f | A: %7.1f %7.1f %7.1f\n",
                             target[0], target[1], target[2],
                             motor0.getFilteredRPM(), motor1.getFilteredRPM(), motor2.getFilteredRPM());
    }

    if (Serial.available()) {
        String line = Serial.readStringUntil('\n');
        handleCommand(line);
    }
}
