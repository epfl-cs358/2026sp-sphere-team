#pragma once

// Parses a "tilt:<angle>" WebSocket message.
// Returns the integer angle on success, or -1 if the message is malformed.
// Clamping to [0, 180] is the caller's responsibility.
int parseServoCommand(const char* msg);
