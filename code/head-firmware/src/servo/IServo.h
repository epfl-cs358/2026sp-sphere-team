#pragma once

class IServo {
public:
    virtual void attach(int pin) = 0;
    virtual void write(int angle) = 0;
    virtual ~IServo() = default;
};
