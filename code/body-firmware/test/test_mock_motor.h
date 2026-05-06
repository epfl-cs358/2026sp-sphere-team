#pragma once

#include "Motor.h"

class MockMotor : public Motor {
public:
    void begin() override { beginCalled = true; }

    void update() override { updateCallCount++; }

    void setSpeed(float speed) override {
        lastSpeed = speed;
        setSpeedCallCount++;
    }

    void brake() override { brakeCalled = true; }

    float getRPM() override { return rpmToReturn; }

    float getFilteredRPM() override { return filteredRpmToReturn; }

    bool beginCalled = false;
    int updateCallCount = 0;
    float lastSpeed = 0.0f;
    int setSpeedCallCount = 0;
    bool brakeCalled = false;
    float rpmToReturn = 0.0f;
    float filteredRpmToReturn = 0.0f;

    void resetMock() {
        beginCalled = false;
        updateCallCount = 0;
        lastSpeed = 0.0f;
        setSpeedCallCount = 0;
        brakeCalled = false;
        rpmToReturn = 0.0f;
        filteredRpmToReturn = 0.0f;
    }
};
