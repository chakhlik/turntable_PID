#include "led_indicator.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

static const char *TAG = "LED";

#define LED_GPIO GPIO_NUM_2

// Параметры мигания (volatile для атомарного доступа из разных задач)
static volatile uint32_t blink_period_ms = 0;
static volatile uint8_t blink_duty_percent = 0;
static volatile uint16_t blink_pulse_count = 0;
static volatile uint16_t blink_pulse_current = 0;
static volatile int64_t blink_start_time_us = 0;
static volatile bool led_state = false;

void led_init(void)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << LED_GPIO),
        .pull_down_en = 0,
        .pull_up_en = 0,
    };
    gpio_config(&io_conf);
    
    gpio_set_level(LED_GPIO, 0);
    ESP_LOGI(TAG, "LED initialized on GPIO %d", LED_GPIO);
}

static void led_set_level(bool level)
{
    gpio_set_level(LED_GPIO, level ? 1 : 0);
    led_state = level;
}

void led_set_blink(uint32_t period_ms, uint8_t duty_percent, uint16_t pulse_count)
{
    blink_period_ms = period_ms;
    blink_duty_percent = duty_percent;
    blink_pulse_count = pulse_count;
    blink_pulse_current = 0;
    blink_start_time_us = esp_timer_get_time() / 1000;
    
    //ESP_LOGI(TAG, "LED blink set: period=%lu ms, duty=%u%%, pulses=%u", 
    //         period_ms, duty_percent, pulse_count);
}

void led_blink_fast(void)
{
    led_set_blink(200, 50, 0);  // 200мс, 50%, бесконечно
}

void led_blink_once(void)
{
    led_set_blink(500, 50, 1);  // 500мс, 50%, 1 раз
}

void led_blink_twice(void)
{
    led_set_blink(500, 50, 2);  // 500мс, 50%, 2 раза
}

void led_on(void)
{
    led_set_blink(0, 100, 0);   // Постоянно горит
    led_set_level(true);
}

void led_off(void)
{
    led_set_blink(0, 0, 0);     // Выключен
    led_set_level(false);
}

static void led_task(void *pvParameters)
{
    ESP_LOGI(TAG, "LED task started");
    
    while (1) {
        uint32_t period = blink_period_ms;
        uint8_t duty = blink_duty_percent;
        uint16_t count = blink_pulse_count;
        uint16_t current = blink_pulse_current;
        int64_t start_time = blink_start_time_us;
        
        if (period > 0 && duty > 0) {
            // Режим мигания
            int64_t current_time_us = esp_timer_get_time() / 1000;
            uint32_t elapsed_ms = (uint32_t)(current_time_us - start_time);
            
            // Вычисляем время включения и выключения
            uint32_t on_time_ms = (period * duty) / 100;
            //uint32_t off_time_ms = period - on_time_ms;
            
            if (current < count || count == 0) {
                // Еще нужно мигать
                if (elapsed_ms < on_time_ms) {
                    // Время включения
                    if (!led_state) {
                        led_set_level(true);
                    }
                } else if (elapsed_ms < period) {
                    // Время выключения
                    if (led_state) {
                        led_set_level(false);
                        blink_pulse_current = current + 1;
                    }
                } else {
                    // Цикл завершен, начинаем новый
                    blink_start_time_us = current_time_us;
                    blink_pulse_current = current + 1;
                }
            } else {
                // Все импульсы отмигали, выключаем
                if (led_state) {
                    led_set_level(false);
                }
            }
        }
        // Если period == 0 или duty == 0, ничего не делаем (состояние установлено функциями led_on/led_off)
        
        vTaskDelay(pdMS_TO_TICKS(10));  // Проверка каждые 10мс
    }
}

void led_task_start(void)
{
    xTaskCreatePinnedToCore(
        led_task,
        "led_task",
        2048,
        NULL,
        3,
        NULL,
        0
    );
}