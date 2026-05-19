/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 */

#pragma once

#include <cstdint>

#include "BalanceConfig.h"

// NVS-backed persistence for BalanceConfig. Mirrors the BNO055 calibration
// pattern (Preferences begin/getBytes/end, begin/putBytes/end). Blob layout
// is [uint16_t VERSION][BalanceConfig payload]; size-or-version mismatches
// leave the caller's struct untouched and return false so boot code can
// fall back to RobotConstants::balanceConfig().
namespace BalanceConfigStorage {

constexpr uint16_t VERSION = 2;

bool load(BalanceConfig& out);
void save(const BalanceConfig& cfg);

}  // namespace BalanceConfigStorage
