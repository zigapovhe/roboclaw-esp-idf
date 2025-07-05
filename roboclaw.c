#include "roboclaw.h"
#include "roboclaw_crc.h"
#include "roboclaw_uart.h"
#include <stdarg.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <inttypes.h>

static const char *TAG = "ROBOCLAW";

// Define packet timeout (must be > 10ms to clear buffer)
#define ROBOCLAW_PACKET_TIMEOUT_MS 15

// Response timeout should be 10ms as per documentation
#define ROBOCLAW_RESPONSE_TIMEOUT_MS 10

// Helper function to send a simple command (address + command + CRC)
static bool send_simple_command(uint8_t address, uint8_t command) {
    uint8_t packet[4];
    packet[0] = address;
    packet[1] = command;
    
    // Calculate CRC on address + command
    uint16_t crc = roboclaw_crc16(packet, 2);
    
    // Add CRC to packet
    packet[2] = (uint8_t)(crc >> 8);
    packet[3] = (uint8_t)(crc & 0xFF);
    
    // Clear UART buffer
    flush();
    
    // Send entire packet at once
    int sent = write_bytes(packet, 4);
    if (sent != 4) {
        ESP_LOGE(TAG, "Failed to send command: sent %d/4 bytes", sent);
        return false;
    }
    
    // Wait for 0xFF response with 10ms timeout
    uint8_t response;
    int len = read_bytes(&response, 1, ROBOCLAW_RESPONSE_TIMEOUT_MS);
    
    if (len == 1 && response == 0xFF) {
        return true;
    }
    
    ESP_LOGE(TAG, "Invalid response: len=%d, data=0x%02X", len, len > 0 ? response : 0);
    return false;
}

// Helper function to send a command with one byte parameter
static bool send_command_with_byte(uint8_t address, uint8_t command, uint8_t value) {
    const int MAX_RETRIES = 3;
    
    ESP_LOGI(TAG, "send_command_with_byte: addr=0x%02X, cmd=0x%02X, value=%d", 
             address, command, value);
    
    for (int retry = 0; retry < MAX_RETRIES; retry++) {
        if (retry > 0) {
            ESP_LOGI(TAG, "Retry %d/%d", retry + 1, MAX_RETRIES);
            // Wait more than 10ms to ensure packet timeout clears any partial data
            vTaskDelay(pdMS_TO_TICKS(15)); 
        }
        
        // Build complete packet first
        uint8_t packet[5];
        packet[0] = address;
        packet[1] = command;
        packet[2] = value;
        
        // Calculate CRC on address + command + value
        uint16_t crc = roboclaw_crc16(packet, 3);
        
        // Add CRC to packet
        packet[3] = (uint8_t)(crc >> 8);
        packet[4] = (uint8_t)(crc & 0xFF);
        
        ESP_LOGI(TAG, "Sending packet: [0x%02X 0x%02X 0x%02X 0x%02X 0x%02X]", 
                 packet[0], packet[1], packet[2], packet[3], packet[4]);
        
        // Clear UART buffer before sending
        flush();
        
        // CRITICAL: Send entire packet at once to avoid 10ms timeout between bytes
        int sent = write_bytes(packet, 5);
        ESP_LOGI(TAG, "Bytes sent: %d/5", sent);
        
        if (sent != 5) {
            ESP_LOGE(TAG, "Failed to send all bytes");
            // Wait > 10ms before retry to clear packet buffer
            vTaskDelay(pdMS_TO_TICKS(15));
            continue;
        }
        
        // Wait for 0xFF acknowledgment with 10ms timeout
        // This also ensures packet buffer clears if no response
        uint8_t response = 0;
        int len = read_bytes(&response, 1, 10); // 10ms timeout as per docs
        
        ESP_LOGI(TAG, "Response: len=%d, data=0x%02X", len, len > 0 ? response : 0);
        
        if (len == 1 && response == 0xFF) {
            ESP_LOGI(TAG, "Command successful!");
            return true;
        }
        
        // No response or wrong response - packet buffer will auto-clear after 10ms
        if (len > 0) {
            ESP_LOGW(TAG, "Unexpected response: 0x%02X", response);
        } else {
            ESP_LOGW(TAG, "No response received (packet may be invalid)");
        }
    }
    
    ESP_LOGE(TAG, "All retries failed!");
    return false;
}

// Helper function to send a command with uint32_t parameter
static bool send_command_with_dword(uint8_t address, uint8_t command, uint32_t value) {
    // Build complete packet first
    uint8_t packet[8];
    packet[0] = address;
    packet[1] = command;
    packet[2] = (uint8_t)(value >> 24);
    packet[3] = (uint8_t)(value >> 16);
    packet[4] = (uint8_t)(value >> 8);
    packet[5] = (uint8_t)(value);
    
    // Calculate CRC on address + command + 4 bytes of value
    uint16_t crc = roboclaw_crc16(packet, 6);
    
    // Add CRC to packet
    packet[6] = (uint8_t)(crc >> 8);
    packet[7] = (uint8_t)(crc & 0xFF);
    
    // Clear UART buffer
    flush();
    
    // Send entire packet at once
    int sent = write_bytes(packet, 8);
    if (sent != 8) {
        return false;
    }
    
    // Wait for 0xFF response with 10ms timeout
    uint8_t response;
    int len = read_bytes(&response, 1, ROBOCLAW_RESPONSE_TIMEOUT_MS);
    return (len == 1 && response == 0xFF);
}

// Motor control functions
bool ForwardM1(uint8_t address, uint8_t speed) {
    ESP_LOGI(TAG, "ForwardM1: address=0x%02X, speed=%d", address, speed);
    return send_command_with_byte(address, M1FORWARD, speed);
}

bool BackwardM1(uint8_t address, uint8_t speed) {
    ESP_LOGI(TAG, "BackwardM1: address=0x%02X, speed=%d", address, speed);
    return send_command_with_byte(address, M1BACKWARD, speed);
}

bool ForwardM2(uint8_t address, uint8_t speed) {
    ESP_LOGI(TAG, "ForwardM2: address=0x%02X, speed=%d", address, speed);
    return send_command_with_byte(address, M2FORWARD, speed);
}

bool BackwardM2(uint8_t address, uint8_t speed) {
    ESP_LOGI(TAG, "BackwardM2: address=0x%02X, speed=%d", address, speed);
    return send_command_with_byte(address, M2BACKWARD, speed);
}

bool SpeedM1(uint8_t address, uint32_t speed) {
    ESP_LOGI(TAG, "SpeedM1: address=0x%02X, speed=%" PRIu32, address, speed);
    return send_command_with_dword(address, M1SPEED, speed);
}

bool SpeedM2(uint8_t address, uint32_t speed) {
    ESP_LOGI(TAG, "SpeedM2: address=0x%02X, speed=%" PRIu32, address, speed);
    return send_command_with_dword(address, M2SPEED, speed);
}

// Helper function to read data with CRC validation
static bool read_data_with_crc(uint8_t address, uint8_t command, uint8_t *buffer, 
                               int expected_data_bytes, int timeout_ms) {
    // Build command packet
    uint8_t cmd_packet[4];
    cmd_packet[0] = address;
    cmd_packet[1] = command;
    
    // Calculate CRC for command
    uint16_t crc = roboclaw_crc16(cmd_packet, 2);
    
    // Add CRC to packet
    cmd_packet[2] = (uint8_t)(crc >> 8);
    cmd_packet[3] = (uint8_t)(crc & 0xFF);
    
    // Clear UART buffer
    flush();
    
    // Send command packet at once
    int sent = write_bytes(cmd_packet, 4);
    if (sent != 4) {
        ESP_LOGE(TAG, "Failed to send read command");
        return false;
    }
    
    // Read response (data + 2 bytes CRC)
    uint8_t response[expected_data_bytes + 2];
    int len = read_bytes(response, expected_data_bytes + 2, timeout_ms);
    
    if (len < expected_data_bytes + 2) {
        ESP_LOGE(TAG, "Insufficient response: got %d bytes, expected %d", 
                 len, expected_data_bytes + 2);
        return false;
    }
    
    // Verify CRC - include address and command in CRC calculation
    uint8_t crc_data[2 + expected_data_bytes];
    crc_data[0] = address;
    crc_data[1] = command;
    memcpy(crc_data + 2, response, expected_data_bytes);
    
    uint16_t calculated_crc = roboclaw_crc16(crc_data, 2 + expected_data_bytes);
    uint16_t received_crc = (response[expected_data_bytes] << 8) | response[expected_data_bytes + 1];
    
    if (calculated_crc != received_crc) {
        ESP_LOGE(TAG, "CRC mismatch: calculated=0x%04X, received=0x%04X", 
                 calculated_crc, received_crc);
        return false;
    }
    
    // Copy data bytes
    memcpy(buffer, response, expected_data_bytes);
    
    return true;
}

// Encoder reading functions
uint32_t ReadEncM1(uint8_t address, uint8_t *status, bool *valid) {
    if (valid) *valid = false;
    if (status) *status = 0;
    
    uint8_t buffer[5]; // 4 bytes encoder + 1 byte status
    if (!read_data_with_crc(address, GETM1ENC, buffer, 5, 1000)) {
        return 0;
    }
    
    // Extract encoder value (big endian)
    uint32_t value = ((uint32_t)buffer[0] << 24) | 
                     ((uint32_t)buffer[1] << 16) | 
                     ((uint32_t)buffer[2] << 8) | 
                     buffer[3];
    
    if (status) *status = buffer[4];
    if (valid) *valid = true;
    
    return value;
}

uint32_t ReadEncM2(uint8_t address, uint8_t *status, bool *valid) {
    if (valid) *valid = false;
    if (status) *status = 0;
    
    uint8_t buffer[5]; // 4 bytes encoder + 1 byte status
    if (!read_data_with_crc(address, GETM2ENC, buffer, 5, 1000)) {
        return 0;
    }
    
    // Extract encoder value (big endian)
    uint32_t value = ((uint32_t)buffer[0] << 24) | 
                     ((uint32_t)buffer[1] << 16) | 
                     ((uint32_t)buffer[2] << 8) | 
                     buffer[3];
    
    if (status) *status = buffer[4];
    if (valid) *valid = true;
    
    return value;
}

bool ResetEncoders(uint8_t address) {
    return send_simple_command(address, RESETENC);
}

// Version reading
bool ReadVersion(uint8_t address, char *version) {
    if (!version) return false;
    
    // Build command packet
    uint8_t cmd_packet[4];
    cmd_packet[0] = address;
    cmd_packet[1] = GETVERSION;
    
    // Calculate CRC
    uint16_t crc = roboclaw_crc16(cmd_packet, 2);
    
    // Add CRC
    cmd_packet[2] = (uint8_t)(crc >> 8);
    cmd_packet[3] = (uint8_t)(crc & 0xFF);
    
    // Clear buffer
    flush();
    
    // Send command
    int sent = write_bytes(cmd_packet, 4);
    if (sent != 4) {
        return false;
    }
    
    
    // Read response
    uint8_t response[64];
    int len = read_bytes(response, sizeof(response), 1000);
    
    if (len < 4) {
        return false;
    }
    
    // Extract version string (skip last 2 CRC bytes)
    int version_len = 0;
    for (int i = 0; i < (len - 2) && version_len < 47; i++) {
        if (response[i] == 0x00 || response[i] == 0x0A) {
            break;
        }
        if (response[i] >= 32 && response[i] <= 126) {
            version[version_len++] = response[i];
        }
    }
    version[version_len] = '\0';
    
    return true;
}

// Battery voltage functions
uint16_t ReadMainBatteryVoltage(uint8_t address, bool *valid) {
    if (valid) *valid = false;
    
    uint8_t buffer[2];
    if (!read_data_with_crc(address, GETMBATT, buffer, 2, 1000)) {
        return 0;
    }
    
    // Extract voltage (big endian)
    uint16_t value = ((uint16_t)buffer[0] << 8) | buffer[1];
    
    if (valid) *valid = true;
    return value;
}

uint16_t ReadLogicBatteryVoltage(uint8_t address, bool *valid) {
    if (valid) *valid = false;
    
    uint8_t buffer[2];
    if (!read_data_with_crc(address, GETLBATT, buffer, 2, 1000)) {
        return 0;
    }
    
    // Extract voltage (big endian)
    uint16_t value = ((uint16_t)buffer[0] << 8) | buffer[1];
    
    if (valid) *valid = true;
    return value;
}

// Speed reading functions
uint32_t ReadSpeedM1(uint8_t address, uint8_t *status, bool *valid) {
    if (valid) *valid = false;
    if (status) *status = 0;
    
    uint8_t buffer[5]; // 4 bytes speed + 1 byte status
    if (!read_data_with_crc(address, GETM1SPEED, buffer, 5, 1000)) {
        return 0;
    }
    
    // Extract speed value (big endian)
    uint32_t value = ((uint32_t)buffer[0] << 24) | 
                     ((uint32_t)buffer[1] << 16) | 
                     ((uint32_t)buffer[2] << 8) | 
                     buffer[3];
    
    if (status) *status = buffer[4];
    if (valid) *valid = true;
    
    return value;
}

uint32_t ReadSpeedM2(uint8_t address, uint8_t *status, bool *valid) {
    if (valid) *valid = false;
    if (status) *status = 0;
    
    uint8_t buffer[5]; // 4 bytes speed + 1 byte status
    if (!read_data_with_crc(address, GETM2SPEED, buffer, 5, 1000)) {
        return 0;
    }
    
    // Extract speed value (big endian)
    uint32_t value = ((uint32_t)buffer[0] << 24) | 
                     ((uint32_t)buffer[1] << 16) | 
                     ((uint32_t)buffer[2] << 8) | 
                     buffer[3];
    
    if (status) *status = buffer[4];
    if (valid) *valid = true;
    
    return value;
}

// Current reading functions
bool ReadCurrents(uint8_t address, int32_t *motor1_current, int32_t *motor2_current) {
    if (!motor1_current || !motor2_current) return false;
    
    uint8_t buffer[8]; // 4 bytes M1 + 4 bytes M2
    if (!read_data_with_crc(address, GETCURRENTS, buffer, 8, 1000)) {
        return false;
    }
    
    // Extract current values (big endian, signed)
    *motor1_current = (int32_t)(((uint32_t)buffer[0] << 24) | 
                                ((uint32_t)buffer[1] << 16) | 
                                ((uint32_t)buffer[2] << 8) | 
                                buffer[3]);
    
    *motor2_current = (int32_t)(((uint32_t)buffer[4] << 24) | 
                                ((uint32_t)buffer[5] << 16) | 
                                ((uint32_t)buffer[6] << 8) | 
                                buffer[7]);
    
    return true;
}

// Temperature reading functions
uint16_t ReadTemp(uint8_t address, bool *valid) {
    if (valid) *valid = false;
    
    uint8_t buffer[2];
    if (!read_data_with_crc(address, GETTEMP, buffer, 2, 1000)) {
        return 0;
    }
    
    // Extract temperature (big endian)
    uint16_t value = ((uint16_t)buffer[0] << 8) | buffer[1];
    
    if (valid) *valid = true;
    return value;
}

uint16_t ReadTemp2(uint8_t address, bool *valid) {
    if (valid) *valid = false;
    
    uint8_t buffer[2];
    if (!read_data_with_crc(address, GETTEMP2, buffer, 2, 1000)) {
        return 0;
    }
    
    // Extract temperature (big endian)
    uint16_t value = ((uint16_t)buffer[0] << 8) | buffer[1];
    
    if (valid) *valid = true;
    return value;
}

// Error reading function
bool ReadError(uint8_t address, uint8_t *error) {
    if (!error) return false;
    
    uint8_t buffer[1];
    if (!read_data_with_crc(address, GETERROR, buffer, 1, 1000)) {
        return false;
    }
    
    *error = buffer[0];
    return true;
}

// Duty cycle control functions (direct PWM control)
bool DutyM1(uint8_t address, int16_t duty) {
    // Duty is -32767 to 32767
    uint8_t packet[6];
    packet[0] = address;
    packet[1] = M1DUTY;
    packet[2] = (uint8_t)(duty >> 8);
    packet[3] = (uint8_t)(duty & 0xFF);
    
    uint16_t crc = roboclaw_crc16(packet, 4);
    packet[4] = (uint8_t)(crc >> 8);
    packet[5] = (uint8_t)(crc & 0xFF);
    
    flush();
    int sent = write_bytes(packet, 6);
    if (sent != 6) return false;
    
    uint8_t response;
    int len = read_bytes(&response, 1, ROBOCLAW_RESPONSE_TIMEOUT_MS);
    return (len == 1 && response == 0xFF);
}

bool DutyM2(uint8_t address, int16_t duty) {
    // Duty is -32767 to 32767
    uint8_t packet[6];
    packet[0] = address;
    packet[1] = M2DUTY;
    packet[2] = (uint8_t)(duty >> 8);
    packet[3] = (uint8_t)(duty & 0xFF);
    
    uint16_t crc = roboclaw_crc16(packet, 4);
    packet[4] = (uint8_t)(crc >> 8);
    packet[5] = (uint8_t)(crc & 0xFF);
    
    flush();
    int sent = write_bytes(packet, 6);
    if (sent != 6) return false;
    
    uint8_t response;
    int len = read_bytes(&response, 1, ROBOCLAW_RESPONSE_TIMEOUT_MS);
    return (len == 1 && response == 0xFF);
}

// Mixed motor control functions
bool ForwardMixed(uint8_t address, uint8_t speed) {
    return send_command_with_byte(address, MIXEDFORWARD, speed);
}

bool BackwardMixed(uint8_t address, uint8_t speed) {
    return send_command_with_byte(address, MIXEDBACKWARD, speed);
}

bool TurnRightMixed(uint8_t address, uint8_t speed) {
    return send_command_with_byte(address, MIXEDRIGHT, speed);
}

bool TurnLeftMixed(uint8_t address, uint8_t speed) {
    return send_command_with_byte(address, MIXEDLEFT, speed);
}

bool ForwardBackwardMixed(uint8_t address, uint8_t speed) {
    return send_command_with_byte(address, MIXEDFB, speed);
}

bool LeftRightMixed(uint8_t address, uint8_t speed) {
    return send_command_with_byte(address, MIXEDLR, speed);
}

// Speed with acceleration control
bool SpeedAccelM1(uint8_t address, uint32_t accel, uint32_t speed) {
    uint8_t packet[10];
    packet[0] = address;
    packet[1] = M1SPEEDACCEL;
    packet[2] = (uint8_t)(accel >> 24);
    packet[3] = (uint8_t)(accel >> 16);
    packet[4] = (uint8_t)(accel >> 8);
    packet[5] = (uint8_t)(accel);
    packet[6] = (uint8_t)(speed >> 24);
    packet[7] = (uint8_t)(speed >> 16);
    packet[8] = (uint8_t)(speed >> 8);
    packet[9] = (uint8_t)(speed);
    
    uint16_t crc = roboclaw_crc16(packet, 10);
    
    uint8_t full_packet[12];
    memcpy(full_packet, packet, 10);
    full_packet[10] = (uint8_t)(crc >> 8);
    full_packet[11] = (uint8_t)(crc & 0xFF);
    
    flush();
    int sent = write_bytes(full_packet, 12);
    if (sent != 12) return false;
    
    uint8_t response;
    int len = read_bytes(&response, 1, ROBOCLAW_RESPONSE_TIMEOUT_MS);
    return (len == 1 && response == 0xFF);
}

bool SpeedAccelM2(uint8_t address, uint32_t accel, uint32_t speed) {
    uint8_t packet[10];
    packet[0] = address;
    packet[1] = M2SPEEDACCEL;
    packet[2] = (uint8_t)(accel >> 24);
    packet[3] = (uint8_t)(accel >> 16);
    packet[4] = (uint8_t)(accel >> 8);
    packet[5] = (uint8_t)(accel);
    packet[6] = (uint8_t)(speed >> 24);
    packet[7] = (uint8_t)(speed >> 16);
    packet[8] = (uint8_t)(speed >> 8);
    packet[9] = (uint8_t)(speed);
    
    uint16_t crc = roboclaw_crc16(packet, 10);
    
    uint8_t full_packet[12];
    memcpy(full_packet, packet, 10);
    full_packet[10] = (uint8_t)(crc >> 8);
    full_packet[11] = (uint8_t)(crc & 0xFF);
    
    flush();
    int sent = write_bytes(full_packet, 12);
    if (sent != 12) return false;
    
    uint8_t response;
    int len = read_bytes(&response, 1, ROBOCLAW_RESPONSE_TIMEOUT_MS);
    return (len == 1 && response == 0xFF);
}

// Stop functions
bool StopM1(uint8_t address) {
    return ForwardM1(address, 0);
}

bool StopM2(uint8_t address) {
    return ForwardM2(address, 0);
}

bool StopAll(uint8_t address) {
    bool m1_stopped = StopM1(address);
    bool m2_stopped = StopM2(address);
    return m1_stopped && m2_stopped;
}