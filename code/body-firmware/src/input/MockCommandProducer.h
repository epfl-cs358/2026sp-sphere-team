/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini, with assistance from Claude (Anthropic)
 */

#pragma once

#include "CommandProducer.h"
#include "CommandLatch.h"

template <typename T>
class MockCommandProducer : public CommandProducer {
public:
    explicit MockCommandProducer(CommandLatch<T>& latch) : _latch(latch) {}

    void start() override { _started = true; }
    void stop() override { _started = false; }
    bool connected() const override { return _connected; }

    void inject(const T& cmd) {
        if (_started) _latch.write(cmd);
    }
    void setConnected(bool c) { _connected = c; }
    bool isStarted() const { return _started; }

private:
    CommandLatch<T>& _latch;
    bool _started = false;
    bool _connected = true;
};
