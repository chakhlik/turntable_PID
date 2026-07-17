#include "udp_telemetry.h"
#include "mcpwm_capture.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

static const char *TAG = "UDP";

// IP-адрес ПК, на котором запущен Python-скрипт
// Замените на реальный IP вашего ПК в сети 192.168.4.x
#define DEST_IP_ADDR    "192.168.4.100"
#define DEST_PORT       5005

extern QueueHandle_t xPulseQueue;

void udp_telemetry_task(void *pvParameters)
{
    ESP_LOGI(TAG, "UDP Telemetry task started");
    
    // Создаем UDP-сокет
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Failed to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    
    // Настраиваем адрес получателя
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(DEST_IP_ADDR);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(DEST_PORT);
    
    ESP_LOGI(TAG, "Sending data to %s:%d", DEST_IP_ADDR, DEST_PORT);
    
    pulse_data_t data;
    uint32_t packet_count = 0;
    
    while (1) {
        // Ждем данные из очереди (блокируемся на 100 мс, если данных нет)
        if (xQueueReceive(xPulseQueue, &data, pdMS_TO_TICKS(100)) == pdTRUE) {
            // Формируем CSV-строку
            // Формат: packet_num,period_us,pulse_index,is_zero_mark
            char udp_buf[128];
            int len = snprintf(udp_buf, sizeof(udp_buf), "%lu,%lu,%u,%d\n",
                               packet_count++,
                               data.period_us,
                               data.pulse_index,
                               data.is_zero_mark ? 1 : 0);
            
            // Отправляем по UDP
            int err = sendto(sock, udp_buf, len, 0, 
                           (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0) {
                ESP_LOGE(TAG, "Send failed: errno %d", errno);
            }
        }
    }
    
    close(sock);
    vTaskDelete(NULL);
}