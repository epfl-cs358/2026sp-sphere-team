#pragma once

#include <cstdint>

namespace ArmingState {

enum class State : uint8_t { Disarmed, Armed, Killed };

// Pre-arm gyro-quiet gate: arm() refuses if |gyro.z| exceeded
// PREARM_GYRO_QUIET_DPS at any sample in the trailing 500 ms window.
// 1.5 deg/s expressed in rad/s.
static constexpr float    PREARM_GYRO_QUIET_DPS      = 0.02618f;
static constexpr uint32_t PREARM_QUIET_WINDOW_SAMPLES = 50;  // 500 ms @ 100 Hz

void  begin();
State get();
bool  isArmed();

void arm();
void disarm();
void kill();
void clearKill();

// Control task feeds |gyro.z| every tick (Armed, Disarmed, and Killed alike)
// so the trailing-window buffer is always warm by the time the user attempts
// to arm. Argument is taken as a raw rate; magnitude is what's checked.
void recordGyroZ(float gyroZ);

// One-shot rejection flag: set inside arm() when the gyro-quiet gate refuses
// the transition. The control task consumes this each tick and OR's the
// corresponding event bit into outgoing telemetry. Returns true exactly once
// per rejection, then auto-clears.
bool consumePrearmRejected();

}  // namespace ArmingState
