#pragma once

#include <stdint.h>

// Инициализация LED (GPIO 2)
void led_init(void);

// Запуск задачи мигания
void led_task_start(void);

// Установка параметров мигания
// period_ms: период одного полного цикла (вкл+выкл) в миллисекундах
// duty_percent: скважность в процентах (0-100)
// pulse_count: количество импульсов (0 = бесконечно)
void led_set_blink(uint32_t period_ms, uint8_t duty_percent, uint16_t pulse_count);

// Быстрые пресеты
void led_blink_fast(void);      // Быстрое мигание (200мс, 50%, бесконечно)
void led_blink_once(void);      // Одно мигание (500мс, 50%, 1 раз)
void led_blink_twice(void);     // Два мигания (500мс, 50%, 2 раза)
void led_on(void);              // Постоянно горит (100%)
void led_off(void);             // Выключен (0%)