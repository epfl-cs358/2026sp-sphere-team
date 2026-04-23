/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini, with assistance from Claude (Anthropic)
 */

#pragma once

// TODO: Design the PID interface.
//
// Needed in two places:
//   1. Per-wheel RPM control inside Drivetrain::update()
//   2. Heading stabilisation inside DrivetrainController::update()
//
// Open questions:
//   - Same class for both, or separate specialisations?
//   - Should it carry its own timing, or receive dt externally?
//   - Anti-windup strategy?
//   - Output clamping?
