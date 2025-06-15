#pragma once

#include <stdint.h>

// Working CRC calculation function
uint16_t roboclaw_crc16(uint8_t *packet, int nBytes);