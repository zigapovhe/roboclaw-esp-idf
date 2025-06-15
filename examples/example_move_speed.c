#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "roboclaw.h"
#include "roboclaw_uart.h"

static const char *TAG = "ROBOCLAW_MOVE_EXAMPLE";

#define ROBOCLAW_ADDRESS 0x80

void app_main(void) {
    ESP_LOGI(TAG, "RoboClaw Motor Movement Example");
    
    // Initialize UART communication at 38400 baud
    begin(38400);
    
    // Give RoboClaw time to initialize
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Test motor movement
    while (1) {
        ESP_LOGI(TAG, "Testing motor forward movement...");
        if (ForwardM1(ROBOCLAW_ADDRESS, 64)) {
            ESP_LOGI(TAG, "Motor 1 forward command sent successfully");
        } else {
            ESP_LOGE(TAG, "Failed to send motor 1 forward command");
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));
        
        // Stop motor
        if (ForwardM1(ROBOCLAW_ADDRESS, 0)) {
            ESP_LOGI(TAG, "Motor 1 stopped");
        } else {
            ESP_LOGE(TAG, "Failed to stop motor 1");
        }
        
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}