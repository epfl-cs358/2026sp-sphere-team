#pragma once

struct BodyVelocity {
    float vx = 0;     // forward speed, m/s (positive = forward)
    float vy = 0;     // strafe speed, m/s (positive = right)
    float omega = 0;  // rotation rate, rad/s (positive = CW from above)
};
