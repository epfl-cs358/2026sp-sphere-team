/*
 * BB-8 Head Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini, with assistance from Claude (Anthropic)
 */

#pragma once

class WifiProvisioner {
public:
    virtual ~WifiProvisioner() = default;
    virtual bool begin() = 0;
    virtual bool isConnected() = 0;
    virtual int getRSSI() = 0;
    virtual const char* getIP() = 0;
};
