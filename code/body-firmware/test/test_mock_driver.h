#pragma once

#include "Driver.h"

class MockDriver : public Driver {
public:
    void begin() override { beginCalled = true; }
    void setOutput(float value) override { lastOutput = value; setOutputCallCount++; }
    void brake() override { brakeCalled = true; }

    bool beginCalled = false;
    float lastOutput = 0.0f;
    int setOutputCallCount = 0;
    bool brakeCalled = false;
};
