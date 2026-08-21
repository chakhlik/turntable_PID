#include "pid_controller.h"
#include "dac_control.h"
#include "mcpwm_capture.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <string.h>

static const char *TAG = "PID";

// === КОНСТАНТЫ В ТИКАХ (1 тик = 12.5 нс при 80 МГц) ===
#define TICKS_PER_US        80
#define TARGET_PERIOD_33_TICKS  (6250 * TICKS_PER_US)   // 500000 тиков
#define TARGET_PERIOD_45_TICKS  (4630 * TICKS_PER_US)   // 370400 тиков

// Порог нулевой метки: 10000 мкс × 80 = 800000 тиков
#define ZERO_MARK_THRESHOLD_TICKS  800000

#define LUT_SIZE          288
#define MIN_CALIB_REVS    10

static pid_mode_t current_mode = PID_MODE_OFF;
static uint32_t target_period_ticks = TARGET_PERIOD_33_TICKS;
static bool pid_running = false;

static float Kp = 0.0f;
static float Ki = 0.006f;
static int32_t integral_term = 0;
static uint16_t dac_value = 2048;

// Пределы интегратора в тиках (было 100000 мкс, теперь × 80)
#define INTEGRAL_LIMIT_TICKS  8000000
#define DAC_MIN         1024
#define DAC_MAX         3072

extern QueueHandle_t xPulseQueue;
QueueHandle_t xTelemetryQueue = NULL;

// === Переменные для LUT (в тиках) ===
static int16_t lut_correction[LUT_SIZE] = {0};           // Поправки в тиках
static int32_t lut_sum[LUT_SIZE] = {0};                  // Основной накопитель
static int32_t temp_lut_sum[LUT_SIZE] = {0};             // Временный буфер
static uint16_t lut_rev_count = 0;

static uint16_t current_pulse_index = 0;
static uint16_t pulses_since_zero = 0;

void pid_init(void)
{
    ESP_LOGI(TAG, "PID controller initialized (ticks mode)");
    ESP_LOGI(TAG, "Target period: %lu ticks (%lu us)", 
             TARGET_PERIOD_33_TICKS, TARGET_PERIOD_33_TICKS / TICKS_PER_US);
    dac_set_value(2048);
    nvs_flash_init();
    
    xTelemetryQueue = xQueueCreate(64, sizeof(telemetry_data_t));
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
    const char* mode_str[] = {"OFF", "ON", "LUT_CALIBRATION", "LUT_ACTIVE"};
    ESP_LOGI(TAG, "PID mode set to: %s", mode_str[mode]);
    
    if (mode == PID_MODE_OFF) {
        pid_stop();
    } else if (mode == PID_MODE_ON || mode == PID_MODE_LUT_ACTIVE) {
        pid_start();
    } else if (mode == PID_MODE_LUT_CALIBRATION) {
        pid_start();
        lut_clear_ram();
    }
}

void pid_set_target_speed(uint8_t speed)
{
    if (speed == 33) {
        target_period_ticks = TARGET_PERIOD_33_TICKS;
    } else if (speed == 45) {
        target_period_ticks = TARGET_PERIOD_45_TICKS;
    }
    ESP_LOGI(TAG, "Target speed: %d RPM (period: %lu ticks = %lu us)", 
             speed, target_period_ticks, target_period_ticks / TICKS_PER_US);
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

// === Функции управления LUT ===

void lut_clear_ram(void)
{
    memset(lut_sum, 0, sizeof(lut_sum));
    memset(temp_lut_sum, 0, sizeof(temp_lut_sum));
    memset(lut_correction, 0, sizeof(lut_correction));
    lut_rev_count = 0;
    current_pulse_index = 0;
    pulses_since_zero = 0;
    ESP_LOGI(TAG, "LUT RAM cleared");
}

void lut_load_from_nvs(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("ttable_lut", NVS_READONLY, &nvs);
    if (err == ESP_OK) {
        size_t required_size = sizeof(lut_correction);
        err = nvs_get_blob(nvs, "lut_corr", lut_correction, &required_size);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "LUT loaded from NVS (%d bytes)", required_size);
        } else {
            ESP_LOGW(TAG, "No LUT data in NVS");
        }
        nvs_close(nvs);
    }
}

void lut_calculate_and_save(void)
{
    if (lut_rev_count < MIN_CALIB_REVS) {
        ESP_LOGW(TAG, "Not enough revolutions: %d (min %d)", lut_rev_count, MIN_CALIB_REVS);
        return;
    }

    ESP_LOGI(TAG, "Calculating LUT from %d revolutions...", lut_rev_count);
    
    for (int i = 0; i < LUT_SIZE; i++) {
        int32_t avg_period_ticks = lut_sum[i] / lut_rev_count;
        
        // Для нулевого индекса вычитаем 2×target (т.к. нулевая метка = 2 импульса)
        if (i == 0) {
            lut_correction[i] = (int16_t)(avg_period_ticks - 2 * target_period_ticks);
        } else {
            lut_correction[i] = (int16_t)(avg_period_ticks - target_period_ticks);
        }
    }

    nvs_handle_t nvs;
    esp_err_t err = nvs_open("ttable_lut", NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        err = nvs_set_blob(nvs, "lut_corr", lut_correction, sizeof(lut_correction));
        if (err == ESP_OK) {
            nvs_commit(nvs);
            ESP_LOGI(TAG, "LUT saved to NVS");
        }
        nvs_close(nvs);
    }

    memset(lut_sum, 0, sizeof(lut_sum));
    lut_rev_count = 0;
}

void pid_task(void *pvParameters)
{
    pulse_data_t pulse_data;
    uint32_t period_sum_ticks = 0;  // Сумма в тиках
    uint8_t pulse_count = 0;
    float filtered_period_ticks = (float)TARGET_PERIOD_33_TICKS;
    const float FILTER_ALPHA = 0.3f;
    
    static uint32_t packet_num = 0;
    static uint32_t log_counter = 0;
    
    ESP_LOGI(TAG, "PID task started (ticks mode)");
    
    while (1) {
        if (xQueueReceive(xPulseQueue, &pulse_data, pdMS_TO_TICKS(100)) == pdTRUE) {
            
            // 1. Обработка нулевой метки
            if (pulse_data.is_zero_mark) {
                bool last_revolution_valid = (pulses_since_zero == 287);
                
                if (!last_revolution_valid) {
                    ESP_LOGW(TAG, "Invalid revolution! Pulses: %d", pulses_since_zero);
                } else {
                    if (current_mode == PID_MODE_LUT_CALIBRATION) {
                        for (int i = 0; i < LUT_SIZE; i++) {
                            lut_sum[i] += temp_lut_sum[i];
                        }
                        lut_rev_count++;
                        
                        if (lut_rev_count % 10 == 0) {
                            ESP_LOGI(TAG, "LUT: %d valid revolutions", lut_rev_count);
                        }
                    }
                }
                
                pulses_since_zero = 0;
                current_pulse_index = 0;
                memset(temp_lut_sum, 0, sizeof(temp_lut_sum));
                
            } else {
                current_pulse_index = (current_pulse_index + 1) % LUT_SIZE;
                pulses_since_zero++;
            }

            // 2. Накопление для LUT (в тиках)
            if (current_mode == PID_MODE_LUT_CALIBRATION) {
                temp_lut_sum[current_pulse_index] += pulse_data.period_ticks;
            }

            // 3. Применение LUT (в тиках)
            int32_t corrected_period_ticks = pulse_data.period_ticks;
            if (current_mode == PID_MODE_LUT_ACTIVE) {
                corrected_period_ticks -= lut_correction[current_pulse_index];
            }

            // 4. Усреднение по 2 импульса
            period_sum_ticks += corrected_period_ticks;
            
            if (pulse_data.is_zero_mark) {
                pulse_count = 2;
            } else {
                pulse_count += 1;
            }
            
            if (pulse_count == 2) {
                uint32_t avg_period_ticks = period_sum_ticks / 2;
                period_sum_ticks = 0;
                pulse_count = 0;
                
                // Экспоненциальное сглаживание (в тиках, float)
                filtered_period_ticks = FILTER_ALPHA * avg_period_ticks + 
                                       (1.0f - FILTER_ALPHA) * filtered_period_ticks;
                
                // Телеметрия (отправляем тики, plotter переведет в мкс)
                telemetry_data_t t_data;
                t_data.packet_num = packet_num++;
                t_data.period = avg_period_ticks;  // В тиках!
                t_data.pulse_index = current_pulse_index;
                t_data.is_zero_mark = pulse_data.is_zero_mark ? 1 : 0;
                xQueueSend(xTelemetryQueue, &t_data, 0);
                
                // Расчет ПИД
                if (pid_running && current_mode != PID_MODE_OFF && current_mode != PID_MODE_LUT_CALIBRATION) {
                    int32_t error_ticks = (int32_t)target_period_ticks - (int32_t)filtered_period_ticks;
                    
                    integral_term -= (int32_t)(error_ticks * Ki * 1000);
                    
                    if (integral_term > INTEGRAL_LIMIT_TICKS) integral_term = INTEGRAL_LIMIT_TICKS;
                    else if (integral_term < -INTEGRAL_LIMIT_TICKS) integral_term = -INTEGRAL_LIMIT_TICKS;
                    
                    int32_t pid_output = 2048 + (integral_term / 1000);
                    
                    if (pid_output < DAC_MIN) pid_output = DAC_MIN;
                    else if (pid_output > DAC_MAX) pid_output = DAC_MAX;
                    
                    dac_value = (uint16_t)pid_output;
                    dac_set_value(dac_value);
                    
                    // Логирование каждые 100 итераций (перевод в мкс для читаемости)
                    if (log_counter++ % 100 == 0) {
                        ESP_LOGI(TAG, "Idx: %3d, Raw: %lu ticks (%.0f us), Corr: %ld ticks, Avg: %lu ticks, Err: %ld ticks, DAC: %u",
                                 current_pulse_index, 
                                 pulse_data.period_ticks,
                                 pulse_data.period_ticks / 80.0f,
                                 (long)lut_correction[current_pulse_index], 
                                 avg_period_ticks, 
                                 error_ticks, 
                                 dac_value);
                    }
                }
            }
        }
    }
}