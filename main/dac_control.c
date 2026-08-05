#include "dac_control.h"
#include "driver/i2c.h"
#include "esp_log.h"

static const char *TAG = "DAC";

#define I2C_MASTER_NUM      I2C_NUM_0
#define I2C_MASTER_SDA_IO   GPIO_NUM_21
#define I2C_MASTER_SCL_IO   GPIO_NUM_22
#define I2C_MASTER_FREQ_HZ  400000  // 400 kHz (Fast Mode)

// Адрес MCP4725 (0x60 если пин A0 подключен к GND)
#define MCP4725_ADDR        0x60

void dac_init(void)
{
    ESP_LOGI(TAG, "Initializing I2C Master for MCP4725...");

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C config failed: %s", esp_err_to_name(err));
        return;
    }

    err = i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "I2C initialized successfully.");

    // При старте сразу выставляем нейтральное значение (середина шкалы)
    dac_set_value(2048);
}

void dac_set_value(uint16_t value)
{
    // Ограничиваем диапазон 12 бит
    if (value > 4095) value = 4095;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
    // Старт условия
    i2c_master_start(cmd);
    
    // Адрес устройства + бит записи
    i2c_master_write_byte(cmd, (MCP4725_ADDR << 1) | I2C_MASTER_WRITE, true);
    
    // MCP4725 Fast Write Mode: отправляем сразу 2 байта данных
    // Старшие 4 бита (D11..D8)
    i2c_master_write_byte(cmd, (value >> 8) & 0x0F, true);
    // Младшие 8 бит (D7..D0)
    i2c_master_write_byte(cmd, value & 0xFF, true);
    
    // Стоп условие
    i2c_master_stop(cmd);

    // Отправка команды с таймаутом 100 мс
    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "DAC write failed: %s", esp_err_to_name(ret));
    }
}