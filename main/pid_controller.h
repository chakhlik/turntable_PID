#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

typedef enum {
    PID_MODE_OFF,
    PID_MODE_ON,               // Обычный PID без LUT
    PID_MODE_LUT_CALIBRATION,  // Режим сбора данных для LUT
    PID_MODE_LUT_ACTIVE        // Режим работы PID с компенсацией по LUT
} pid_mode_t;

void pid_init(void);
void pid_task(void *pvParameters);
void pid_start(void);
void pid_stop(void);
bool pid_is_running(void);

void pid_set_mode(pid_mode_t mode);
void pid_set_target_speed(uint8_t speed);

void pid_set_ki(float ki);
float pid_get_ki(void);

// === Новые функции для LUT ===
void lut_clear_ram(void);
void lut_load_from_nvs(void);
void lut_calculate_and_save(void);

// Структура данных для телеметрии (после усреднения)
typedef struct {
    uint32_t packet_num;
    uint32_t period;
    uint16_t pulse_index;
    uint8_t  is_zero_mark;
} telemetry_data_t;

// Очередь для передачи данных в задачу UDP
extern QueueHandle_t xTelemetryQueue;