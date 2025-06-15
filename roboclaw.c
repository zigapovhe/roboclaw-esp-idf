#include "roboclaw.h"
#include "roboclaw_crc.h"
#include "roboclaw_uart.h"
#include <stdarg.h>
#include <string.h>

// Helper function to send a simple command (address + command + CRC)
static bool send_simple_command(uint8_t address, uint8_t command) {
    uint8_t cmd[] = {address, command};
    uint16_t crc = roboclaw_crc16(cmd, 2);
    
    uint8_t full_cmd[] = {
        address,
        command,
        (uint8_t)(crc >> 8),
        (uint8_t)(crc & 0xFF)
    };
    
    flush();
    int sent = write_bytes(full_cmd, 4);
    if (sent != 4) {
        return false;
    }
    
    // Wait for 0xFF response
    uint8_t response;
    int len = read_bytes(&response, 1, 100); // 100ms timeout
    return (len == 1 && response == 0xFF);
}

// Helper function to send a command with one parameter
static bool send_command_with_param(uint8_t address, uint8_t command, uint8_t param) {
    uint8_t cmd[] = {address, command, param};
    uint16_t crc = roboclaw_crc16(cmd, 3);
    
    uint8_t full_cmd[] = {
        address,
        command,
        param,
        (uint8_t)(crc >> 8),
        (uint8_t)(crc & 0xFF)
    };
    
    flush();
    int sent = write_bytes(full_cmd, 5);
    if (sent != 5) {
        return false;
    }
    
    // Wait for 0xFF response
    uint8_t response;
    int len = read_bytes(&response, 1, 100); // 100ms timeout
    return (len == 1 && response == 0xFF);
}

// Helper function to send a command with uint32_t parameter
static bool send_command_with_dword(uint8_t address, uint8_t command, uint32_t param) {
    uint8_t cmd[] = {
        address, 
        command,
        (uint8_t)(param >> 24),
        (uint8_t)(param >> 16),
        (uint8_t)(param >> 8),
        (uint8_t)(param)
    };
    uint16_t crc = roboclaw_crc16(cmd, 6);
    
    uint8_t full_cmd[] = {
        address,
        command,
        (uint8_t)(param >> 24),
        (uint8_t)(param >> 16),
        (uint8_t)(param >> 8),
        (uint8_t)(param),
        (uint8_t)(crc >> 8),
        (uint8_t)(crc & 0xFF)
    };
    
    flush();
    int sent = write_bytes(full_cmd, 8);
    if (sent != 8) {
        return false;
    }
    
    // Wait for 0xFF response
    uint8_t response;
    int len = read_bytes(&response, 1, 100); // 100ms timeout
    return (len == 1 && response == 0xFF);
}

// Helper function to read 4 bytes + status (encoder/speed functions)
static uint32_t read_4bytes_with_status(uint8_t address, uint8_t command, uint8_t *status, bool *valid) {
    if (valid) *valid = false;
    if (status) *status = 0;
    
    uint8_t cmd[] = {address, command};
    uint16_t crc = roboclaw_crc16(cmd, 2);
    
    uint8_t full_cmd[] = {
        address,
        command,
        (uint8_t)(crc >> 8),
        (uint8_t)(crc & 0xFF)
    };
    
    flush();
    int sent = write_bytes(full_cmd, 4);
    if (sent != 4) {
        return 0;
    }
    
    // Wait for response (4 bytes data + 1 byte status + 2 bytes CRC = 7 bytes total)
    uint8_t response[8];
    int len = read_bytes(response, sizeof(response), 1000);
    
    if (len < 7) {
        return 0;
    }
    
    // Extract 4-byte value (big endian) and status
    uint32_t value = (response[0] << 24) | (response[1] << 16) | (response[2] << 8) | response[3];
    if (status) *status = response[4];
    if (valid) *valid = true;
    
    return value;
}

// Motor control functions (updated to use working approach)
bool ForwardM1(uint8_t address, uint8_t speed) {
    return send_command_with_param(address, M1FORWARD, speed);
}

bool BackwardM1(uint8_t address, uint8_t speed) {
    return send_command_with_param(address, M1BACKWARD, speed);
}

bool ForwardM2(uint8_t address, uint8_t speed) {
    return send_command_with_param(address, M2FORWARD, speed);
}

bool BackwardM2(uint8_t address, uint8_t speed) {
    return send_command_with_param(address, M2BACKWARD, speed);
}

bool SpeedM1(uint8_t address, uint32_t speed) {
    return send_command_with_dword(address, M1SPEED, speed);
}

bool SpeedM2(uint8_t address, uint32_t speed) {
    return send_command_with_dword(address, M2SPEED, speed);
}

// Encoder functions (updated to use working approach)
uint32_t ReadEncM1(uint8_t address, uint8_t *status, bool *valid) {
    return read_4bytes_with_status(address, GETM1ENC, status, valid);
}

uint32_t ReadEncM2(uint8_t address, uint8_t *status, bool *valid) {
    return read_4bytes_with_status(address, GETM2ENC, status, valid);
}

bool ResetEncoders(uint8_t address) {
    return send_simple_command(address, RESETENC);
}

// Version reading (using the working approach)
bool ReadVersion(uint8_t address, char *version) {
    // Prepare command like the working code
    uint8_t cmd[] = {address, GETVERSION};
    uint16_t crc = roboclaw_crc16(cmd, 2);
    
    uint8_t full_cmd[] = {
        address,
        GETVERSION,
        (uint8_t)(crc >> 8),
        (uint8_t)(crc & 0xFF)
    };
    
    // Clear any existing data in UART buffer
    flush();
    
    // Send command
    int sent = write_bytes(full_cmd, 4);
    if (sent != 4) {
        return false;
    }
    
    // Wait for response
    uint8_t response[64];
    int len = read_bytes(response, sizeof(response), 1000); // 1 second timeout
    
    if (len < 4) {
        return false;
    }
    
    // Extract version string (skip last 2 CRC bytes, stop at null terminator or newline)
    int version_len = 0;
    for (int i = 0; i < (len - 2) && version_len < 47; i++) { // max 47 chars + null terminator
        if (response[i] == 0x00 || response[i] == 0x0A) {
            break;
        }
        if (response[i] >= 32 && response[i] <= 126) { // Printable ASCII
            version[version_len++] = response[i];
        }
    }
    version[version_len] = '\0';
    
    return true;
}

// Battery voltage functions (using the working approach)
uint16_t ReadMainBatteryVoltage(uint8_t address, bool *valid) {
    if (valid) *valid = false;
    
    // Prepare command like the working code
    uint8_t cmd[] = {address, GETMBATT};
    uint16_t crc = roboclaw_crc16(cmd, 2);
    
    uint8_t full_cmd[] = {
        address,
        GETMBATT,
        (uint8_t)(crc >> 8),
        (uint8_t)(crc & 0xFF)
    };
    
    // Clear any existing data in UART buffer
    flush();
    
    // Send command
    int sent = write_bytes(full_cmd, 4);
    if (sent != 4) {
        return 0;
    }
    
    // Wait for response (2 bytes data + 2 bytes CRC = 4 bytes total)
    uint8_t response[8];
    int len = read_bytes(response, sizeof(response), 1000); // 1 second timeout
    
    if (len < 4) {
        return 0;
    }
    
    // Extract 2-byte value (big endian)
    uint16_t value = (response[0] << 8) | response[1];
    
    if (valid) *valid = true;
    return value;
}

uint16_t ReadLogicBatteryVoltage(uint8_t address, bool *valid) {
    if (valid) *valid = false;
    
    // Prepare command like the working code
    uint8_t cmd[] = {address, GETLBATT};
    uint16_t crc = roboclaw_crc16(cmd, 2);
    
    uint8_t full_cmd[] = {
        address,
        GETLBATT,
        (uint8_t)(crc >> 8),
        (uint8_t)(crc & 0xFF)
    };
    
    // Clear any existing data in UART buffer
    flush();
    
    // Send command
    int sent = write_bytes(full_cmd, 4);
    if (sent != 4) {
        return 0;
    }
    
    // Wait for response (2 bytes data + 2 bytes CRC = 4 bytes total)
    uint8_t response[8];
    int len = read_bytes(response, sizeof(response), 1000); // 1 second timeout
    
    if (len < 4) {
        return 0;
    }
    
    // Extract 2-byte value (big endian)
    uint16_t value = (response[0] << 8) | response[1];
    
    if (valid) *valid = true;
    return value;
}

// Speed reading functions (updated to use working approach)
uint32_t ReadSpeedM1(uint8_t address, uint8_t *status, bool *valid) {
    return read_4bytes_with_status(address, GETM1SPEED, status, valid);
}

uint32_t ReadSpeedM2(uint8_t address, uint8_t *status, bool *valid) {
    return read_4bytes_with_status(address, GETM2SPEED, status, valid);
}