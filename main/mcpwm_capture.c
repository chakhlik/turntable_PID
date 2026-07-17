#include "mcpwm_capture.h"
#include "driver/mcpwm_cap.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

static const char *TAG = "MCPWM";

// Очередь для передачи данных из ISR в задачу UDP
// Размер 64 элемента - с запасом, чтобы никогда не переполнялась
QueueHandle_t xPulseQueue = NULL;

// Структура данных для одного измерения
typedef struct {
    uint32_t period_us;      // Измеренный период в микросекундах
    uint8_t  pulse_index;    // Индекс импульса (0..143)
    bool     is_zero_mark;   // Флаг: это был широкий зазор (нулевая метка)
} pulse_data_t;

// Переменные ISR
static volatile uint32_t last_capture_val = 0;
static volatile uint8_t  pulse_index = 0;

// Порог для определения нулевой метки (широкого зазора)
// Нормальный период ~12500 мкс, двойной ~25000 мкс
// Порог = 18000 мкс (с запасом)
#define ZERO_MARK_THRESHOLD_US  18000

// Обработчик прерывания MCPWM Capture
static bool IRAM_ATTR mcpwm_capture_isr_handler(mcpwm_cap_handle_t cap,
                                                 const mcpwm_capture_event_data_t *edata,
                                                 void *user_data)
{
    uint32_t current_capture = edata->cap_value;
    
    // Вычисляем период (в тиках таймера, которые равны микросекундам при prescaler=80)
    uint32_t period = current_capture - last_capture_val;
    last_capture_val = current_capture;
    
    // Защита от переполнения при первом запуске
    if (period > 100000) {
        period = 0; // Игнорируем первое измерение
    }
    
    // Формируем структуру данных
    pulse_data_t data;
    data.period_us = period;
    data.is_zero_mark = false;
    
    // Проверяем, является ли это нулевой меткой (широкий зазор)
    if (period > ZERO_MARK_THRESHOLD_US) {
        pulse_index = 0;  // Сбрасываем индекс
        data.is_zero_mark = true;
        // Период для нулевой метки делим пополам для ПИД (но в итерации 1 просто передаем как есть)
    } else {
        pulse_index++;
        if (pulse_index >= 144) {
            pulse_index = 0; // Защита от выхода за границы
        }
    }
    
    data.pulse_index = pulse_index;
    
    // Отправляем данные в очередь (не блокируясь!)
    // Если очередь полна, данные отбрасываются (для итерации 1 это допустимо)
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
    
    // Создаем очередь
    xPulseQueue = xQueueCreate(64, sizeof(pulse_data_t));
    if (xPulseQueue == NULL) {
        ESP_LOGE(TAG, "Failed to create queue");
        return;
    }
    
    // 1. Создаем группу MCPWM
    mcpwm_group_handle_t group = NULL;
    mcpwm_group_config_t group_config = {
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_group(&group_config, &group));
    
    // 2. Настраиваем таймер MCPWM
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000,  // 1 МГц -> 1 мкс разрешение
        .period_ticks = 65535,     // Максимум для 16 бит
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(group, &timer_config, &timer));
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));
    
    // 3. Настраиваем канал захвата (Capture)
    mcpwm_capture_handle_t cap = NULL;
    mcpwm_capture_config_t cap_config = {
        .gpio_num = gpio_num,
        .prescale = 1,
        .capture_edge = MCPWM_CAP_EDGE_POS,  // Реагируем на нарастающий фронт
        .filter_clk_src = MCPWM_CAPTURE_FILTER_CLK_SRC_DEFAULT,
        .filter_resolution_hz = 0,  // Аппаратный фильтр отключен (используем внешний триггер Шмитта)
    };
    ESP_ERROR_CHECK(mcpwm_new_capture(group, &cap_config, &cap));
    
    // 4. Регистрируем ISR
    ESP_ERROR_CHECK(mcpwm_capture_set_event_callback(cap, mcpwm_capture_isr_handler, NULL));
    ESP_ERROR_CHECK(mcpwm_capture_enable(cap, MCPWM_CAP_EDGE_POS));
    
    ESP_LOGI(TAG, "MCPWM Capture initialized. Resolution: 1 us");
}