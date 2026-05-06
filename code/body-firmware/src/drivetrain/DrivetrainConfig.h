#pragma once

struct DrivetrainConfig {
    float wheelRadius;   // meters
    float robotRadius;   // center to wheel contact point, meters
    float tiltAngle;     // wheel tilt from vertical, radians
    float maxRPM;        // motor no-load RPM (e.g. 251 for FIT0186)
};
