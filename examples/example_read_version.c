#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "roboclaw.h"
#include "roboclaw_uart.h"

static const char *TAG = "ROBOCLAW_VERSION_EXAMPLE";

#define ROBOCLAW_ADDRESS 0x80

void app_main(void) {
    ESP_LOGI(TAG, "RoboClaw Version Reading Example");
    
    // Initialize UART communication at 38400 baud
    begin(38400);
    
    // Give RoboClaw time to initialize
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Read version continuously
    while (1) {
        char version[64];
        
        ESP_LOGI(TAG, "Reading RoboClaw version...");
        
        if (ReadVersion(ROBOCLAW_ADDRESS, version)) {
            ESP_LOGI(TAG, "SUCCESS - RoboClaw Version: '%s'", version);
        } else {
            ESP_LOGE(TAG, "FAILED to read version");
        }
        
        // Test reading battery voltage
        bool valid;
        uint16_t battery_voltage = ReadMainBatteryVoltage(ROBOCLAW_ADDRESS, &valid);
        if (valid) {
            ESP_LOGI(TAG, "Main Battery Voltage: %d (raw)", battery_voltage);
        } else {
            ESP_LOGE(TAG, "Failed to read battery voltage");
        }
        
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}