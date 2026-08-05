#include "mqtt_tools.h"
#include "mqtt_client.h"
#include "esp_log.h"
#include "fsm.h"
#include "pid_controller.h"

static const char *TAG = "MQTT";

#define MQTT_BROKER_URI    "mqtt://192.168.1.136"
#define MQTT_USERNAME      "eksh"
#define MQTT_PASSWORD      "bvzrjirb"

static esp_mqtt_client_handle_t client = NULL;

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, 
                               int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    
    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT connected");
            // Подписываемся на топики управления
            esp_mqtt_client_subscribe(client, "turntable/play", 1);
            esp_mqtt_client_subscribe(client, "turntable/speed", 1);
            esp_mqtt_client_subscribe(client, "turntable/pid_mode", 1);
            esp_mqtt_client_subscribe(client, "turntable/pid_ki", 1);
            break;
            
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT disconnected");
            break;
            
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "TOPIC=%.*s", event->topic_len, event->topic);
            ESP_LOGI(TAG, "DATA=%.*s", event->data_len, event->data);
            
            // Обработка команд
            if (strncmp(event->topic, "turntable/play", event->topic_len) == 0) {
                if (strncmp(event->data, "ON", event->data_len) == 0) {
                    fsm_event_play();
                } else if (strncmp(event->data, "OFF", event->data_len) == 0) {
                    fsm_event_stop();
                }
            } else if (strncmp(event->topic, "turntable/speed", event->topic_len) == 0) {
                if (strncmp(event->data, "33", event->data_len) == 0) {
                    fsm_event_speed_33();
                } else if (strncmp(event->data, "45", event->data_len) == 0) {
                    fsm_event_speed_45();
                }
            } else if (strncmp(event->topic, "turntable/pid_mode", event->topic_len) == 0) {
                if (strncmp(event->data, "ON", event->data_len) == 0) {
                    pid_set_mode(PID_MODE_ON);
                } else if (strncmp(event->data, "OFF", event->data_len) == 0) {
                    pid_set_mode(PID_MODE_OFF);
                } else if (strncmp(event->data, "LUT", event->data_len) == 0) {
                    pid_set_mode(PID_MODE_LUT_CALIBRATION);
                }
            } else if (strncmp(event->topic, "turntable/pid_ki", event->topic_len) == 0) {
                // Парсим значение Ki из данных
                char data_buf[32] = {0};
                memcpy(data_buf, event->data, event->data_len < 31 ? event->data_len : 31);
                float ki = atof(data_buf);
                pid_set_ki(ki);
            }
            break;
            
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
}

void my_mqtt_client_init(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI,
        .credentials.username = MQTT_USERNAME,
        .credentials.authentication.password = MQTT_PASSWORD,
    };

    client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
    
    ESP_LOGI(TAG, "MQTT client initialized");
}

void my_mqtt_client_publish(const char* topic, const char* data)
{
    if (client != NULL) {
        esp_mqtt_client_publish(client, topic, data, strlen(data), 1, 0);
    }
}