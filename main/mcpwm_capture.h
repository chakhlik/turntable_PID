#pragma once
#include "driver/gpio.h"
#include <stdint.h>
#include <stdbool.h>

// Структура данных для одного измерения
// Вынесена в заголовочный файл, так как используется и в mcpwm_capture.c, и в udp_telemetry.c
typedef struct {
    uint32_t period_us;      // Измеренный период в микросекундах
    uint8_t  pulse_index;    // Индекс импульса (0..143)
    bool     is_zero_mark;   // Флаг: это был широкий зазор (нулевая метка)
} pulse_data_t;

void mcpwm_capture_init(gpio_num_t gpio_num);