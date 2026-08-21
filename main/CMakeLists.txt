idf_component_register(SRCS "mcpwm_capture.c" "pid_controller.c" "main.c" 
                              "wifi_sta.c" 
                              "mqtt_tools.c"
                              "fsm.c" 
                              
                              "lut_calibration.c"
                              "dac_control.c" 
                              
                              "udp_telemetry.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_wifi esp_event esp_netif mqtt driver nvs_flash freertos)