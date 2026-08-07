#include "pid_controller.h"
#include "dac_control.h"
#include "mcpwm_capture.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char *TAG = "PID";

#define TARGET_PERIOD_33  6250  // мкс для 33.333 RPM (288 импульсов)
#define TARGET_PERIOD_45  4630  // мкс для 45 RPM (288 импульсов)

static pid_mode_t current_mode = PID_MODE_OFF;
static uint32_t target_period = TARGET_PERIOD_33;
static bool pid_running = false;

// PID параметры
static float Kp = 0.0f;
static float Ki = 0.006f;
static int32_t integral_term = 0;
static uint16_t dac_value = 2048;

#define INTEGRAL_LIMIT  100000
#define DAC_MIN         1024
#define DAC_MAX         3072

extern QueueHandle_t xPulseQueue;

void pid_init(void)
{
    ESP_LOGI(TAG, "PID controller initialized");
    dac_set_value(2048);
}

void pid_start(void)
{
    pid_running = true;
    integral_term = 0;
    dac_value = 2048;
    ESP_LOGI(TAG, "PID started");
}

void pid_stop(void)
{
    pid_running = false;
    dac_set_value(2048);
    ESP_LOGI(TAG, "PID stopped");
}

bool pid_is_running(void)
{
    return pid_running;
}

void pid_set_mode(pid_mode_t mode)
{
    current_mode = mode;
    const char* mode_str[] = {"OFF", "ON", "LUT_CALIBRATION"};
    ESP_LOGI(TAG, "PID mode set to: %s", mode_str[mode]);
    switch (current_mode) {
        case PID_MODE_OFF:
            pid_stop();
            break;
        case PID_MODE_ON:
            pid_start();
            break;
        case PID_MODE_LUT_CALIBRATION:
            pid_stop();
            break;
    }

}

void pid_set_target_speed(uint8_t speed)
{
    if (speed == 33) {
        target_period = TARGET_PERIOD_33;
    } else if (speed == 45) {
        target_period = TARGET_PERIOD_45;
    }
    ESP_LOGI(TAG, "Target speed: %d RPM (period: %lu us)", speed, target_period);
}

void pid_task(void *pvParameters)
{
    pulse_data_t pulse_data;
    uint32_t period_sum = 0;
    uint8_t pulse_count = 0;
    float filtered_period = 6250.0f;
    const float FILTER_ALPHA = 0.3f;
    
    ESP_LOGI(TAG, "PID task started with even-count averaging");
    
    while (1) {
        if (xQueueReceive(xPulseQueue, &pulse_data, pdMS_TO_TICKS(100)) == pdTRUE) {
            
            // Накапливаем период
            period_sum += pulse_data.period_us;
            
            // Обновляем счетчик
            if (pulse_data.is_zero_mark) {
                pulse_count = 2;  // Нулевой импульс сразу дает 2
            } else {
                pulse_count += 1; // Обычный импульс добавляет 1
            }
            
            // Передача и расчет происходят строго когда накоплено 2 "нормальных" импульса
            if (pulse_count == 2) {
                // Вычисляем средний период
                uint32_t avg_period = period_sum / 2;
                
                // Сбрасываем накопитель для следующего цикла
                period_sum = 0;
                pulse_count = 0;
                
                // Применяем экспоненциальное сглаживание
                filtered_period = FILTER_ALPHA * avg_period + 
                                 (1.0f - FILTER_ALPHA) * filtered_period;
                
                // Рассчитываем PID
                if (pid_running && current_mode != PID_MODE_OFF) {
                    int32_t error = (int32_t)target_period - (int32_t)filtered_period;
                    
                    integral_term -= (int32_t)(error * Ki * 1000);
                    
                    if (integral_term > INTEGRAL_LIMIT) integral_term = INTEGRAL_LIMIT;
                    else if (integral_term < -INTEGRAL_LIMIT) integral_term = -INTEGRAL_LIMIT;
                    
                    int32_t pid_output = 2048 + (integral_term / 1000);
                    
                    if (pid_output < DAC_MIN) pid_output = DAC_MIN;
                    else if (pid_output > DAC_MAX) pid_output = DAC_MAX;
                    
                    dac_value = (uint16_t)pid_output;
                    dac_set_value(dac_value);
                    
                    static uint32_t counter = 0;
                    if (counter++ % 100 == 0) {
                        ESP_LOGI(TAG, "Avg: %lu, Filtered: %.1f, Error: %ld, DAC: %u",
                                 avg_period, filtered_period, error, dac_value);
                    }
                }
            }
        }
    }
}

void lut_calibration_task(void *pvParameters)
{
    ESP_LOGI(TAG, "LUT calibration task started");
    
    // Здесь будет реализация калибровки LUT
    // Сбор данных за несколько оборотов и вычисление таблицы коррекции
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void pid_set_ki(float ki)
{
    Ki = ki;
    ESP_LOGI(TAG, "Ki changed to: %.6f", ki);
}

float pid_get_ki(void)
{
    return Ki;
}