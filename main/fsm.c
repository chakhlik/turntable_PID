#include "fsm.h"
#include "pid_controller.h"
#include "dac_control.h"
#include "mqtt_tools.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "FSM";

static fsm_state_t current_state = FSM_STATE_IDLE_33;
static bool tonearm_outer_closed = false;
static bool tonearm_inner_closed = false;

static const char* state_to_string(fsm_state_t state)
{
    switch (state) {
        case FSM_STATE_IDLE_33: return "idle_33";
        case FSM_STATE_IDLE_45: return "idle_45";
        case FSM_STATE_ON_33: return "on_33";
        case FSM_STATE_ON_45: return "on_45";
        case FSM_STATE_ROTATION_33: return "rotation_33";
        case FSM_STATE_ROTATION_45: return "rotation_45";
        default: return "unknown";
    }
}

static void set_state(fsm_state_t new_state)
{
    current_state = new_state;
    ESP_LOGI(TAG, "State changed to: %s", state_to_string(new_state));
    
    // Публикуем состояние в MQTT
    char msg[64];
    snprintf(msg, sizeof(msg), "{\"state\":\"%s\"}", state_to_string(new_state));
    my_mqtt_client_publish("turntable/status", msg);
}

void fsm_init(void)
{
    set_state(FSM_STATE_IDLE_33);
    dac_set_value(2048);
}

fsm_state_t fsm_get_state(void)
{
    return current_state;
}

void fsm_event_play(void)
{
    ESP_LOGI(TAG, "Event: Play");
    
    switch (current_state) {
        case FSM_STATE_IDLE_33:
        case FSM_STATE_IDLE_45:
            if (!tonearm_outer_closed && !tonearm_inner_closed) {
                if (current_state == FSM_STATE_IDLE_33) {
                    set_state(FSM_STATE_ROTATION_33);
                } else {
                    set_state(FSM_STATE_ROTATION_45);
                }
                pid_start();
            } else {
                if (current_state == FSM_STATE_IDLE_33) {
                    set_state(FSM_STATE_ON_33);
                } else {
                    set_state(FSM_STATE_ON_45);
                }
            }
            break;
            
        case FSM_STATE_ON_33:
        case FSM_STATE_ON_45:
        case FSM_STATE_ROTATION_33:
        case FSM_STATE_ROTATION_45:
            set_state(current_state == FSM_STATE_ON_33 || current_state == FSM_STATE_ROTATION_33 
                     ? FSM_STATE_IDLE_33 : FSM_STATE_IDLE_45);
            pid_stop();
            dac_set_value(2048);
            break;
    }
}

void fsm_event_stop(void)
{
    fsm_event_play(); // Stop работает как toggle
}

void fsm_event_speed_33(void)
{
    ESP_LOGI(TAG, "Event: Speed 33");
    
    switch (current_state) {
        case FSM_STATE_IDLE_33:
        case FSM_STATE_ON_33:
        case FSM_STATE_ROTATION_33:
            // Уже 33, ничего не делаем
            break;
            
        case FSM_STATE_IDLE_45:
            set_state(FSM_STATE_IDLE_33);
            break;
            
        case FSM_STATE_ON_45:
            set_state(FSM_STATE_ON_33);
            pid_set_target_speed(33);
            break;
            
        case FSM_STATE_ROTATION_45:
            set_state(FSM_STATE_ROTATION_33);
            pid_set_target_speed(33);
            break;
    }
}

void fsm_event_speed_45(void)
{
    ESP_LOGI(TAG, "Event: Speed 45");
    
    switch (current_state) {
        case FSM_STATE_IDLE_45:
        case FSM_STATE_ON_45:
        case FSM_STATE_ROTATION_45:
            // Уже 45, ничего не делаем
            break;
            
        case FSM_STATE_IDLE_33:
            set_state(FSM_STATE_IDLE_45);
            break;
            
        case FSM_STATE_ON_33:
            set_state(FSM_STATE_ON_45);
            pid_set_target_speed(45);
            break;
            
        case FSM_STATE_ROTATION_33:
            set_state(FSM_STATE_ROTATION_45);
            pid_set_target_speed(45);
            break;
    }
}

void fsm_task(void *pvParameters)
{
    ESP_LOGI(TAG, "FSM task started");
    
    while (1) {
        // Проверка перехода из ON в ROTATION
        if (!tonearm_inner_closed && !tonearm_outer_closed) {
            if (current_state == FSM_STATE_ON_33 || current_state == FSM_STATE_ON_45) {
                set_state(current_state == FSM_STATE_ON_33 ? FSM_STATE_ROTATION_33 : FSM_STATE_ROTATION_45);
                pid_start();
            }
        } else {
            if (current_state == FSM_STATE_ROTATION_33 || current_state == FSM_STATE_ROTATION_45) {
                set_state(current_state == FSM_STATE_ROTATION_33 ? FSM_STATE_ON_33 : FSM_STATE_ON_45);
                pid_stop();
                dac_set_value(2048);
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}