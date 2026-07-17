#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "wifi_ap.h"
#include "mcpwm_capture.h"
#include "udp_telemetry.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "=== Turntable PID - Iteration 1 ===");
    
    // 1. Инициализация NVS (обязательно для Wi-Fi)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. Запуск Wi-Fi в режиме точки доступа (AP Mode)
    // Это создаст сеть "TurntablePID" с паролем "12345678"
    // IP-адрес ESP32 будет 192.168.4.1
    wifi_ap_init();
    
    // Ждем подключения к Wi-Fi (или просто даем время на старт)
    vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_LOGI(TAG, "Wi-Fi AP started. SSID: TurntablePID, Password: 12345678");
    ESP_LOGI(TAG, "ESP32 IP: 192.168.4.1");

    // 3. Инициализация MCPWM Capture
    // Пин датчика скорости (замените на ваш реальный пин!)
    mcpwm_capture_init(GPIO_NUM_18);
    
    // 4. Запуск задачи отправки телеметрии по UDP
    // Привязываем к Core 0 (RT-ядро)
    xTaskCreatePinnedToCore(
        udp_telemetry_task,
        "udp_telemetry",
        4096,
        NULL,
        5,
        NULL,
        0  // Core 0
    );
    
    ESP_LOGI(TAG, "System started. Waiting for pulses...");
    
    // Основной цикл ничего не делает, вся работа в задачах
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}