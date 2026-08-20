#include "udp_telemetry.h"
#include "pid_controller.h" // Для telemetry_data_t и xTelemetryQueue
//#include "mcpwm_capture.h"   
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "lwip/sockets.h"
#include "lwip/netdb.h"

static const char *TAG = "UDP";
//QueueHandle_t xUdpQueue = NULL;

#define DEST_IP_ADDR    "192.168.1.78"
#define DEST_PORT       5005

// Очередь объявлена в mcpwm_capture.c, делаем её видимой
// extern QueueHandle_t xPulseQueue; заменяем на отдельную очередь

void udp_telemetry_task(void *pvParameters)
{
    ESP_LOGI(TAG, "UDP Telemetry task started");
    
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Failed to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    // !!!!Изменен способ телеметрии!!!!
    // Создаем очередь специально для UDP (размер 64)
    //if (xUdpQueue == NULL) {
    //    xUdpQueue = xQueueCreate(64, sizeof(pulse_data_t));
    //}
    
    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(DEST_IP_ADDR);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(DEST_PORT);
    
    ESP_LOGI(TAG, "Sending data to %s:%d", DEST_IP_ADDR, DEST_PORT);
    
    //pulse_data_t data;  // <-- Теперь тип определен через mcpwm_capture.h
    //uint32_t packet_count = 0;

    telemetry_data_t t_data;
    
    while (1) {
        if (xQueueReceive(xTelemetryQueue, &t_data, pdMS_TO_TICKS(100)) == pdTRUE) {
            char udp_buf[128];
            int len = snprintf(udp_buf, sizeof(udp_buf), "%lu,%lu,%u,%d\n",
                               (unsigned long)t_data.packet_num,
                               (unsigned long)t_data.period, 
                               t_data.pulse_index,
                               t_data.is_zero_mark);
            
            int err = sendto(sock, udp_buf, len, 0, 
                           (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            // закоментировано для экономии ресурсов
            //if (err < 0) {
            //    ESP_LOGE(TAG, "Send failed: errno %d", errno);
            //}
        }
    }
    
    close(sock);
    vTaskDelete(NULL);
}