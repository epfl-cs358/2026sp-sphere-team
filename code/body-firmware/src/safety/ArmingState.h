#pragma once

#include <cstdint>

namespace ArmingState {

enum class State : uint8_t { Disarmed, Armed, Killed };

void  begin();
State get();
bool  isArmed();

void arm();
void disarm();
void kill();
void clearKill();

}  // namespace ArmingState
