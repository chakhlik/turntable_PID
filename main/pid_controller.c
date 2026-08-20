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

#define TARGET_PERIOD_33  6250  // мкс
#define TARGET_PERIOD_45  4630  // мкс
#define LUT_SIZE          287   // Количество позиций за оборот
#define MIN_CALIB_REVS    10    // Минимальное число оборотов для калибровки

static pid_mode_t current_mode = PID_MODE_OFF;
static uint32_t target_period = TARGET_PERIOD_33;
static bool pid_running = false;

static float Kp = 0.0f;
static float Ki = 0.006f;
static int32_t integral_term = 0;
static uint16_t dac_value = 2048;

#define INTEGRAL_LIMIT  100000
#define DAC_MIN         1024
#define DAC_MAX         3072

extern QueueHandle_t xPulseQueue;
// Глобальная очередь для телеметрии
QueueHandle_t xTelemetryQueue = NULL;

// === Переменные для LUT ===
static int16_t lut_correction[LUT_SIZE] = {0};           // Итоговая таблица поправок
static int32_t lut_sum[LUT_SIZE] = {0};                  // Основной накопитель (только валидные обороты)
static int32_t temp_lut_sum[LUT_SIZE] = {0};             // Временный буфер текущего оборота
static uint16_t lut_rev_count = 0;                       // Счетчик валидных оборотов

static uint16_t current_pulse_index = 0;                 // Текущий индекс импульса (0..287)
static uint16_t pulses_since_zero = 0;                   // Счетчик импульсов с последней нулевой метки

void pid_init(void)
{
    ESP_LOGI(TAG, "PID controller initialized");
    dac_set_value(2048);
    nvs_flash_init();

    // Создаем очередь для телеметрии (глубина 64, достаточно для сглаживания пиков Wi-Fi)
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
        target_period = TARGET_PERIOD_33;
    } else if (speed == 45) {
        target_period = TARGET_PERIOD_45;
    }
    ESP_LOGI(TAG, "Target speed: %d RPM (period: %lu us)", speed, target_period);
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
            ESP_LOGI(TAG, "LUT loaded from NVS successfully (%d bytes)", required_size);
        } else {
            ESP_LOGW(TAG, "No LUT data found in NVS");
        }
        nvs_close(nvs);
    } else {
        ESP_LOGW(TAG, "Failed to open NVS namespace: %s", esp_err_to_name(err));
    }
}

void lut_calculate_and_save(void)
{
    if (lut_rev_count < MIN_CALIB_REVS) {
        ESP_LOGW(TAG, "Not enough VALID revolutions for calibration: %d (min %d)", lut_rev_count, MIN_CALIB_REVS);
        return;
    }

    ESP_LOGI(TAG, "Calculating LUT from %d valid revolutions...", lut_rev_count);
    
    for (int i = 0; i < LUT_SIZE; i++) {
        int32_t avg_period = lut_sum[i] / lut_rev_count;
        if (i == 0) {
            // Нулевой индекс: сырой период ~12500, нужно вычесть 2×target
            lut_correction[i] = (int16_t)(avg_period - 2 * target_period);
        } else {
            // Обычные индексы: сырой период ~6250, вычитаем target
            lut_correction[i] = (int16_t)(avg_period - target_period);
        }
    }

    nvs_handle_t nvs;
    esp_err_t err = nvs_open("ttable_lut", NVS_READWRITE, &nvs);
    if (err == ESP_OK) {
        err = nvs_set_blob(nvs, "lut_corr", lut_correction, sizeof(lut_correction));
        if (err == ESP_OK) {
            err = nvs_commit(nvs);
            if (err == ESP_OK) {
                ESP_LOGI(TAG, "LUT calculated and saved to NVS successfully!");
            }
        }
        nvs_close(nvs);
    }

    memset(lut_sum, 0, sizeof(lut_sum));
    lut_rev_count = 0;
}

void pid_task(void *pvParameters)
{
    pulse_data_t pulse_data;
    uint32_t period_sum = 0;
    uint8_t pulse_count = 0;
    float filtered_period = 6250.0f;
    const float FILTER_ALPHA = 0.7f;
    
    ESP_LOGI(TAG, "PID task started with corrected LUT logic");

    uint32_t packet_num = 0;
    
    while (1) {
        if (xQueueReceive(xPulseQueue, &pulse_data, pdMS_TO_TICKS(100)) == pdTRUE) {
            
            // =========================================================
            // 1. Обработка нулевой метки - ПРОВЕРКА ЗАВЕРШЕННОГО ОБОРОТА
            // =========================================================
            if (pulse_data.is_zero_mark) {
                // Проверяем валидность ЗАВЕРШЕННОГО оборота
                bool last_revolution_valid = (pulses_since_zero == 286);
                
                if (!last_revolution_valid) {
                    ESP_LOGW(TAG, "Invalid revolution detected! Pulses: %d (expected 286). Discarded.", pulses_since_zero);
                } else {
                    // Оборот валиден - копируем временный буфер в основной накопитель
                    if (current_mode == PID_MODE_LUT_CALIBRATION) {
                        for (int i = 0; i < LUT_SIZE; i++) {
                            lut_sum[i] += temp_lut_sum[i];
                        }
                        lut_rev_count++;
                        
                        if (lut_rev_count % 10 == 0) {
                            ESP_LOGI(TAG, "LUT calibration: %d valid revolutions accumulated", lut_rev_count);
                        }
                    }
                }
                
                // Сбрасываем счетчики для нового оборота
                pulses_since_zero = 0;
                current_pulse_index = 0;
                memset(temp_lut_sum, 0, sizeof(temp_lut_sum));
                
            } else {
                current_pulse_index = (current_pulse_index + 1) % LUT_SIZE;
                pulses_since_zero++;
            }

            // =========================================================
            // 2. Накопление данных во ВРЕМЕННЫЙ буфер (всегда, без проверки флага)
            // =========================================================
            if (current_mode == PID_MODE_LUT_CALIBRATION) {
                temp_lut_sum[current_pulse_index] += pulse_data.period_us;
            }

            // =========================================================
            // 3. ПРИМЕНЕНИЕ LUT ДО УСРЕДНЕНИЯ
            // =========================================================
            int32_t corrected_period = pulse_data.period_us;
            if (current_mode == PID_MODE_LUT_ACTIVE) {
                corrected_period = pulse_data.period_us - lut_correction[current_pulse_index];
            }

            // =========================================================
            // 4. Усреднение по 2 импульса и расчет ПИД
            // =========================================================
            period_sum += corrected_period;
            
            if (pulse_data.is_zero_mark) {
                pulse_count = 2;
            } else {
                pulse_count += 1;
            }
            
            if (pulse_count == 2) {
                uint32_t avg_period = period_sum / 2;
                period_sum = 0;
                pulse_count = 0;
                
                filtered_period = FILTER_ALPHA * avg_period + (1.0f - FILTER_ALPHA) * filtered_period;

                // === ОТПРАВКА В ОЧЕРЕДЬ ТЕЛЕМЕТРИИ ===
                telemetry_data_t t_data;
                t_data.packet_num = packet_num++;
                t_data.period = avg_period;
                t_data.pulse_index = current_pulse_index;
                t_data.is_zero_mark = pulse_data.is_zero_mark ? 1 : 0;
                
                // xQueueSend с timeout=0: если очередь переполнена, пакет просто отбрасывается,
                // что предотвращает блокировку задачи PID. Для телеметрии это допустимо.
                xQueueSend(xTelemetryQueue, &t_data, 0);
                // ======================================
                
                if (pid_running && current_mode != PID_MODE_OFF && current_mode != PID_MODE_LUT_CALIBRATION) {
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
                        ESP_LOGI(TAG, "Idx: %3d, Raw: %lu, Corr: %ld, Avg: %lu, Err: %ld, DAC: %u",
                                 current_pulse_index, 
                                 pulse_data.period_us, 
                                 (long)lut_correction[current_pulse_index], 
                                 avg_period, 
                                 error, 
                                 dac_value);
                    }
                }
            }
        }
    }
}