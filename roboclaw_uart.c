#include "roboclaw_uart.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "ROBOCLAW_UART";
static uart_port_t uart_num = UART_NUM_1;

#define UART_BUFFER_SIZE 1024
#define TXD_PIN 17
#define RXD_PIN 16

// Initialize UART
void begin(uint32_t baud_rate) {
    const uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    ESP_ERROR_CHECK(uart_driver_install(uart_num, UART_BUFFER_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    ESP_LOGI(TAG, "UART initialized for RoboClaw communication at %lu baud", (unsigned long)baud_rate);
}

// Flush the UART buffer
void flush(void) {
    uart_flush(uart_num);
}

// Read multiple bytes (main communication function)
int read_bytes(uint8_t *buffer, size_t length, uint32_t timeout_ms) {
    return uart_read_bytes(uart_num, buffer, length, pdMS_TO_TICKS(timeout_ms));
}

// Write multiple bytes (main communication function)
int write_bytes(const uint8_t *data, size_t length) {
    return uart_write_bytes(uart_num, data, length);
}