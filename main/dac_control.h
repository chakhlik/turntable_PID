#pragma once

#include <stdint.h>

// Инициализация I2C и ЦАП
void dac_init(void);

// Установка значения ЦАП (0..4095)
void dac_set_value(uint16_t value);