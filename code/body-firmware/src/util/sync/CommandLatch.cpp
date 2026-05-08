/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini, with assistance from Claude (Anthropic)
 */

#include "CommandLatch.h"
#include "BodyVelocity.h"

// Explicit instantiation for BodyVelocity so the linker has a body
// in builds that compile this translation unit (ESP32 production).
template class CommandLatch<BodyVelocity>;
