#include "roboclaw_crc.h"

// Working CRC calculation function
uint16_t roboclaw_crc16(uint8_t *packet, int nBytes) {
    uint16_t crc = 0;
    for (int byte = 0; byte < nBytes; byte++) {
        crc = crc ^ ((uint16_t)packet[byte] << 8);
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = crc << 1;
            }
        }
    }
    return crc;
}