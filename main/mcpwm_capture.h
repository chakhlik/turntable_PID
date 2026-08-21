#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef struct {
    uint32_t period_ticks;  // Период в тиках 80 МГц (1 тик = 12.5 нс)
    uint16_t pulse_index;
    bool is_zero_mark;
} pulse_data_t;

extern QueueHandle_t xPulseQueue;

void gptimer_capture_init(int gpio_num);