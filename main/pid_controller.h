#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    PID_MODE_OFF,
    PID_MODE_ON,
    PID_MODE_LUT_CALIBRATION
} pid_mode_t;

void pid_init(void);
void pid_task(void *pvParameters);
void pid_start(void);
void pid_stop(void);
bool pid_is_running(void);

void pid_set_mode(pid_mode_t mode);
void pid_set_target_speed(uint8_t speed);

// LUT функции
void lut_calibration_task(void *pvParameters);

void pid_set_ki(float ki);
float pid_get_ki(void);