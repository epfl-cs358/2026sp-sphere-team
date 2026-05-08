#pragma once

#include <cstdint>

typedef int BaseType_t;
typedef unsigned int UBaseType_t;
typedef uint32_t TickType_t;

#define pdTRUE  ((BaseType_t)1)
#define pdFALSE ((BaseType_t)0)
#define pdPASS  pdTRUE
#define pdFAIL  pdFALSE

#define portMAX_DELAY ((TickType_t)0xFFFFFFFFUL)

// In the stub, ticks == ms (configTICK_RATE_HZ assumed 1000)
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
