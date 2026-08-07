#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "wifi_sta.h"
#include "mqtt_tools.h"
#include "fsm.h"
#include "pid_controller.h"
#include "dac_control.h"
#include "mcpwm_capture.h"
#include "udp_telemetry.h"

static const char *TAG = "MAIN";

void app_main(void)
{
    ESP_LOGI(TAG, "=== Turntable PID - FSM + LUT ===");
    
    // 1. Инициализация NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. WiFi STA
    wifi_sta_init();
    
    // Ждем подключения WiFi
    int retry = 0;
    while (!wifi_sta_is_connected() && retry < 20) {
        vTaskDelay(pdMS_TO_TICKS(500));
        retry++;
    }
    
    if (!wifi_sta_is_connected()) {
        ESP_LOGE(TAG, "Failed to connect to WiFi");
        return;
    }
    
    ESP_LOGI(TAG, "WiFi connected");

    
    
    // 3. DAC
    dac_init();
    
    // 4. PID
    pid_init();
    
    // 5. Capture
    mcpwm_capture_init(GPIO_NUM_19);
    
    // 6. FSM
    fsm_init();

    // 7. MQTT
    my_mqtt_client_init();
    
    // Задачи
    xTaskCreatePinnedToCore(
        udp_telemetry_task,
        "udp_telemetry",
        4096,
        NULL,
        5,
        NULL,
        0
    );
    
    xTaskCreatePinnedToCore(
        pid_task,
        "pid_task",
        4096,
        NULL,
        15,
        NULL,
        0
    );
    
    xTaskCreatePinnedToCore(
        fsm_task,
        "fsm_task",
        4096,
        NULL,
        10,
        NULL,
        1
    );
    
    xTaskCreatePinnedToCore(
        lut_calibration_task,
        "lut_cal",
        4096,
        NULL,
        8,
        NULL,
        1
    );
    
    ESP_LOGI(TAG, "System started");
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}