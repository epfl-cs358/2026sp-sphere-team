/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini, with assistance from Claude (Anthropic)
 */

#pragma once

#include <array>
#include <cstddef>

template <size_t N>
class MovingAverage {
    static_assert(N > 0, "Window size must be at least 1");

public:
    void push(float value) {
        _buffer[_index] = value;
        _index = (_index + 1) % N;
        if (_count < N) ++_count;
    }

    float average() const {
        if (_count == 0) return 0.0f;
        float sum = 0.0f;
        for (size_t i = 0; i < _count; ++i) {
            sum += _buffer[i];
        }
        return sum / static_cast<float>(_count);
    }

    void reset() {
        _buffer = {};
        _index = 0;
        _count = 0;
    }

private:
    std::array<float, N> _buffer{};
    size_t _index = 0;
    size_t _count = 0;
};
