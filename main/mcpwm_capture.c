#include "mcpwm_capture.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

extern QueueHandle_t xUdpQueue; // Объявлена в udp_telemetry.c
static const char *TAG = "CAPTURE";

// Очередь для передачи данных из ISR в задачу UDP
QueueHandle_t xPulseQueue = NULL;

// Переменные ISR
static volatile int64_t last_capture_us = 0;
static volatile uint16_t pulse_index = 0;

// Порог для определения нулевой метки (широкого зазора)
#define ZERO_MARK_THRESHOLD_US  9500 // 18000 for 144 wheel

// GPIO пин датчика
static gpio_num_t capture_gpio = GPIO_NUM_19;

// Обработчик прерывания GPIO
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    // Получаем текущее время в микросекундах
    int64_t current_time_us = esp_timer_get_time();
    
    // Вычисляем период
    int64_t period_us = current_time_us - last_capture_us;
    last_capture_us = current_time_us;
    
    // Защита от переполнения при первом запуске
    if (period_us > 100000 || period_us < 0) {
        period_us = 0; 
    }
    
    pulse_data_t data;
    data.period_us = (uint32_t)period_us;
    data.is_zero_mark = false;
    
    if (period_us > ZERO_MARK_THRESHOLD_US) {
        pulse_index = 0;
        data.is_zero_mark = true;
    } else {
        pulse_index++;
        if (pulse_index >= 288) {  // <-- Было 144
            pulse_index = 0;
        }
    }
    
    data.pulse_index = pulse_index;
    
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(xPulseQueue, &data, &xHigherPriorityTaskWoken);
    // Отправляем копию данных в очередь для UDP телеметрии
    if (xUdpQueue != NULL) {
        xQueueSendFromISR(xUdpQueue, &data, &xHigherPriorityTaskWoken);
    }
    
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

void mcpwm_capture_init(gpio_num_t gpio_num)
{
    ESP_LOGI(TAG, "Initializing GPIO Capture on GPIO %d", gpio_num);
    
    capture_gpio = gpio_num;
    
    // Создаем очередь
    xPulseQueue = xQueueCreate(64, sizeof(pulse_data_t));
    if (xPulseQueue == NULL) {
        ESP_LOGE(TAG, "Failed to create queue");
        return;
    }
    
    // Настраиваем GPIO как вход
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE,  // Прерывание по нарастающему фронту
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << gpio_num),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    
    // Устанавливаем ISR сервис
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    
    // Регистрируем обработчик прерывания
    ESP_ERROR_CHECK(gpio_isr_handler_add(gpio_num, gpio_isr_handler, NULL));
    
    ESP_LOGI(TAG, "GPIO Capture initialized successfully. Resolution: 1 us (SYSTIMER)");
}