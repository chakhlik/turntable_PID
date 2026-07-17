#include "mcpwm_capture.h"
#include "driver/mcpwm_cap.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

static const char *TAG = "MCPWM";

// Очередь для передачи данных из ISR в задачу UDP
QueueHandle_t xPulseQueue = NULL;

// Переменные ISR
static volatile uint32_t last_capture_val = 0;
static volatile uint8_t  pulse_index = 0;

// Порог для определения нулевой метки (широкого зазора)
#define ZERO_MARK_THRESHOLD_US  18000

// Обработчик прерывания MCPWM Capture
static bool IRAM_ATTR mcpwm_capture_isr_handler(mcpwm_cap_channel_handle_t cap_channel,
                                                 const mcpwm_capture_event_data_t *edata,
                                                 void *user_data)
{
    uint32_t current_capture = edata->cap_value;
    
    // Вычисляем период
    uint32_t period = current_capture - last_capture_val;
    last_capture_val = current_capture;
    
    // Защита от переполнения при первом запуске
    if (period > 100000) {
        period = 0; 
    }
    
    pulse_data_t data;
    data.period_us = period;
    data.is_zero_mark = false;
    
    if (period > ZERO_MARK_THRESHOLD_US) {
        pulse_index = 0;
        data.is_zero_mark = true;
    } else {
        pulse_index++;
        if (pulse_index >= 144) {
            pulse_index = 0;
        }
    }
    
    data.pulse_index = pulse_index;
    
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(xPulseQueue, &data, &xHigherPriorityTaskWoken);
    
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
    
    return false;
}

void mcpwm_capture_init(gpio_num_t gpio_num)
{
    ESP_LOGI(TAG, "Initializing MCPWM Capture on GPIO %d", gpio_num);
    
    xPulseQueue = xQueueCreate(64, sizeof(pulse_data_t));
    if (xPulseQueue == NULL) {
        ESP_LOGE(TAG, "Failed to create queue");
        return;
    }
    
    // 1. Создаем таймер захвата (Capture Timer)
    mcpwm_cap_timer_handle_t cap_timer = NULL;
    mcpwm_capture_timer_config_t cap_timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,  // 1 МГц -> 1 мкс разрешение
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&cap_timer_config, &cap_timer));
    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(cap_timer));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(cap_timer));
    
    // 2. Создаем канал захвата (Capture Channel)
    mcpwm_cap_channel_handle_t cap_channel = NULL;
    mcpwm_capture_channel_config_t cap_channel_config = {
        .gpio_num = gpio_num,
        .prescale = 1,
        // В ESP-IDF v5.x настройки фронтов находятся внутри структуры flags
        .flags = {
            .pos_edge = true,   // Реагируем на нарастающий фронт
            .neg_edge = false,  // Не реагируем на спадающий
        },
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &cap_channel_config, &cap_channel));
    
    // 3. Регистрируем ISR через колбэки
    mcpwm_capture_event_callbacks_t cbs = {
        .on_cap = mcpwm_capture_isr_handler,
    };
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(cap_channel, &cbs, NULL));
    
    // 4. Включаем канал
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_channel));
    
    ESP_LOGI(TAG, "MCPWM Capture initialized successfully. Resolution: 1 us");
}