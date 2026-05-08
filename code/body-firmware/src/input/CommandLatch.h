/*
 * BB-8 Body Firmware — MIT 2026 SP, Team Sphere
 * Alessandro Lombardini, with assistance from Claude (Anthropic)
 */

#pragma once

#include <optional>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

template <typename T>
class CommandLatch {
public:
    CommandLatch() : _queue(xQueueCreate(1, sizeof(T))) {}

    ~CommandLatch() {
        if (_queue) vQueueDelete(_queue);
    }

    CommandLatch(const CommandLatch&) = delete;
    CommandLatch& operator=(const CommandLatch&) = delete;

    void write(const T& value) {
        xQueueOverwrite(_queue, &value);
    }

    std::optional<T> read() {
        T out;
        if (xQueueReceive(_queue, &out, 0) == pdTRUE) {
            return out;
        }
        return std::nullopt;
    }

private:
    QueueHandle_t _queue;
};
