#pragma once

#include <freertos/FreeRTOS.h>
#include <cassert>
#include <cstring>
#include <cstdlib>

// Internal stub state held behind the opaque QueueHandle_t.
struct StubQueue {
    UBaseType_t length;
    UBaseType_t itemSize;
    bool hasItem;
    void* slot;  // single-slot only (length must be 1)
};

typedef StubQueue* QueueHandle_t;

inline QueueHandle_t xQueueCreate(UBaseType_t length, UBaseType_t itemSize) {
    // The stub only models length-1 overwrite-style queues. Catch any future
    // misuse at test time rather than letting it pass here and break on device.
    assert(length == 1);
    auto* q = new StubQueue;
    q->length = length;
    q->itemSize = itemSize;
    q->hasItem = false;
    q->slot = std::malloc(itemSize);
    return q;
}

inline BaseType_t xQueueOverwrite(QueueHandle_t q, const void* item) {
    // Per FreeRTOS docs, only valid for length-1 queues.
    std::memcpy(q->slot, item, q->itemSize);
    q->hasItem = true;
    return pdTRUE;
}

inline BaseType_t xQueueReceive(QueueHandle_t q, void* out, TickType_t ticksToWait) {
    // Stub is single-threaded; blocking semantics are not modelled. Any
    // ticksToWait > 0 would silently return immediately and look like a
    // success — flag it loudly instead.
    assert(ticksToWait == 0);
    if (!q->hasItem) return pdFALSE;
    std::memcpy(out, q->slot, q->itemSize);
    q->hasItem = false;
    return pdTRUE;
}

inline void vQueueDelete(QueueHandle_t q) {
    if (!q) return;
    std::free(q->slot);
    delete q;
}
