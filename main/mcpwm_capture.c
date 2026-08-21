#include "mcpwm_capture.h"
#include "driver/gptimer.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/queue.h"

static const char *TAG = "CAPTURE";

static gptimer_handle_t gptimer = NULL;
static uint64_t last_capture_ticks = 0;
QueueHandle_t xPulseQueue = NULL;

// Порог для определения нулевой метки в тиках (10000 мкс * 80 тиков/мкс = 800 000 тиков)
#define ZERO_MARK_THRESHOLD_TICKS 800000 

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint64_t current_ticks;
    // Читаем текущее значение 64-битного таймера. 
    // Эта функция безопасна для вызова из ISR, так как это простое чтение регистра.
    gptimer_get_raw_count(gptimer, &current_ticks);
    
    uint32_t period_ticks = (uint32_t)(current_ticks - last_capture_ticks);
    last_capture_ticks = current_ticks;

    pulse_data_t data;
    data.period_ticks = period_ticks;
    data.is_zero_mark = false;

    if (period_ticks > ZERO_MARK_THRESHOLD_TICKS) {
        data.pulse_index = 0;
        data.is_zero_mark = true;
    } else {
        data.pulse_index = 0; // Индекс будет корректно пересчитан в pid_task
    }

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(xPulseQueue, &data, &xHigherPriorityTaskWoken);
    
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

void gptimer_capture_init(int gpio_num)
{
    ESP_LOGI(TAG, "Initializing GPTimer Capture on GPIO %d", gpio_num);

    xPulseQueue = xQueueCreate(32, sizeof(pulse_data_t));

    // Настраиваем 64-битный таймер с частотой 80 МГц (1 тик = 12.5 нс)
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT, // APB 80 МГц
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 80 * 1000 * 1000,  // 80 МГц
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    // Запускаем таймер (он будет свободно считать, переполнение через ~3600 лет)
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_start(gptimer));

    // Настраиваем GPIO на вход с прерыванием по нарастающему фронту
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_POSEDGE, // Используйте GPIO_INTR_ANYEDGE, если у вас другой тип сигнала
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << gpio_num),
        .pull_down_en = 0,
        .pull_up_en = 1,
    };
    gpio_config(&io_conf);

    // Устанавливаем обработчик прерывания с высоким приоритетом и в IRAM
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL3);
    gpio_isr_handler_add(gpio_num, gpio_isr_handler, NULL);

    ESP_LOGI(TAG, "GPTimer Capture initialized. Resolution: 12.5 ns (80 MHz)");
}