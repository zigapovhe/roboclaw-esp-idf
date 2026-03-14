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

// Enable thread-safe UART access (call once after begin() if using multiple tasks)
void roboclaw_enable_thread_safety(void);

// Internal lock/unlock (used automatically by roboclaw.c functions)
void uart_lock(void);
void uart_unlock(void);
