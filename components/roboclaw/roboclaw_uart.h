#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/uart.h"

// Initialize UART communication
void begin(uint32_t baud_rate);

// Flush the UART buffer
void flush(void);

// Read multiple bytes (main communication function)
int read_bytes(uint8_t *buffer, size_t length, uint32_t timeout_ms);

// Write multiple bytes (main communication function)
int write_bytes(const uint8_t *data, size_t length);