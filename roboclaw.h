#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "roboclaw_crc.h"

// Default RoboClaw address
#define ROBOCLAW_DEFAULT_ADDRESS 0x80

// RoboClaw error status bits
#define ROBOCLAW_STATUS_NORMAL     0x00
#define ROBOCLAW_STATUS_M1_OVERCURRENT  0x01
#define ROBOCLAW_STATUS_M2_OVERCURRENT  0x02
#define ROBOCLAW_STATUS_ESTOP      0x04
#define ROBOCLAW_STATUS_TEMP_ERROR 0x08
#define ROBOCLAW_STATUS_TEMP2_ERROR 0x10
#define ROBOCLAW_STATUS_MBATT_HIGH 0x20
#define ROBOCLAW_STATUS_LBATT_HIGH 0x40
#define ROBOCLAW_STATUS_LBATT_LOW  0x80

// RoboClaw command definitions (from Arduino library)
enum {
    M1FORWARD = 0,
    M1BACKWARD = 1,
    SETMINMB = 2,
    SETMAXMB = 3,
    M2FORWARD = 4,
    M2BACKWARD = 5,
    M17BIT = 6,
    M27BIT = 7,
    MIXEDFORWARD = 8,
    MIXEDBACKWARD = 9,
    MIXEDRIGHT = 10,
    MIXEDLEFT = 11,
    MIXEDFB = 12,
    MIXEDLR = 13,
    GETM1ENC = 16,
    GETM2ENC = 17,
    GETM1SPEED = 18,
    GETM2SPEED = 19,
    RESETENC = 20,
    GETVERSION = 21,
    SETM1ENCCOUNT = 22,
    SETM2ENCCOUNT = 23,
    GETMBATT = 24,
    GETLBATT = 25,
    SETMINLB = 26,
    SETMAXLB = 27,
    SETM1PID = 28,
    SETM2PID = 29,
    GETM1ISPEED = 30,
    GETM2ISPEED = 31,
    M1DUTY = 32,
    M2DUTY = 33,
    MIXEDDUTY = 34,
    M1SPEED = 35,
    M2SPEED = 36,
    MIXEDSPEED = 37,
    M1SPEEDACCEL = 38,
    M2SPEEDACCEL = 39,
    MIXEDSPEEDACCEL = 40,
    M1SPEEDDIST = 41,
    M2SPEEDDIST = 42,
    MIXEDSPEEDDIST = 43,
    M1SPEEDACCELDIST = 44,
    M2SPEEDACCELDIST = 45,
    MIXEDSPEEDACCELDIST = 46,
    GETBUFFERS = 47,
    GETPWMS = 48,
    GETCURRENTS = 49,
    MIXEDSPEED2ACCEL = 50,
    MIXEDSPEED2ACCELDIST = 51,
    M1DUTYACCEL = 52,
    M2DUTYACCEL = 53,
    MIXEDDUTYACCEL = 54,
    READM1PID = 55,
    READM2PID = 56,
    SETMAINVOLTAGES = 57,
    SETLOGICVOLTAGES = 58,
    GETMINMAXMAINVOLTAGES = 59,
    GETMINMAXLOGICVOLTAGES = 60,
    SETM1POSPID = 61,
    SETM2POSPID = 62,
    READM1POSPID = 63,
    READM2POSPID = 64,
    M1SPEEDACCELDECCELPOS = 65,
    M2SPEEDACCELDECCELPOS = 66,
    MIXEDSPEEDACCELDECCELPOS = 67,
    SETM1DEFAULTACCEL = 68,
    SETM2DEFAULTACCEL = 69,
    SETPINFUNCTIONS = 74,
    GETPINFUNCTIONS = 75,
    SETDEADBAND = 76,
    GETDEADBAND = 77,
    GETENCODERS = 78,
    GETISPEEDS = 79,
    RESTOREDEFAULTS = 80,
    GETTEMP = 82,
    GETTEMP2 = 83,
    GETERROR = 90,
    GETENCODERMODE = 91,
    SETM1ENCODERMODE = 92,
    SETM2ENCODERMODE = 93,
    WRITENVM = 94,
    READNVM = 95,
    SETCONFIG = 98,
    GETCONFIG = 99,
    SETM1MAXCURRENT = 133,
    SETM2MAXCURRENT = 134,
    GETM1MAXCURRENT = 135,
    GETM2MAXCURRENT = 136,
    SETPWMMODE = 148,
    GETPWMMODE = 149,
    FLAGBOOTLOADER = 255
};

// Motor control functions
bool ForwardM1(uint8_t address, uint8_t speed);
bool BackwardM1(uint8_t address, uint8_t speed);
bool ForwardM2(uint8_t address, uint8_t speed);
bool BackwardM2(uint8_t address, uint8_t speed);
bool SpeedM1(uint8_t address, uint32_t speed);
bool SpeedM2(uint8_t address, uint32_t speed);

// --- Duty cycle control ---
bool DutyM1(uint8_t address, int16_t duty);
bool DutyM2(uint8_t address, int16_t duty);

// --- Acceleration control ---
bool SpeedAccelM1(uint8_t address, uint32_t accel, uint32_t speed);
bool SpeedAccelM2(uint8_t address, uint32_t accel, uint32_t speed);

// Encoder functions
uint32_t ReadEncM1(uint8_t address, uint8_t *status, bool *valid);
uint32_t ReadEncM2(uint8_t address, uint8_t *status, bool *valid);
bool ResetEncoders(uint8_t address);

// --- Speed ---
uint32_t ReadSpeedM1(uint8_t address, uint8_t *status, bool *valid);
uint32_t ReadSpeedM2(uint8_t address, uint8_t *status, bool *valid);

// --- Currents ---
bool ReadCurrents(uint8_t address, int32_t *motor1_current, int32_t *motor2_current);

// --- Temperature ---
uint16_t ReadTemp(uint8_t address, bool *valid);
uint16_t ReadTemp2(uint8_t address, bool *valid);

// --- Battery voltages ---
uint16_t ReadMainBatteryVoltage(uint8_t address, bool *valid);
uint16_t ReadLogicBatteryVoltage(uint8_t address, bool *valid);

// Version and diagnostics
bool ReadVersion(uint8_t address, char *version);
bool ReadError(uint8_t address, uint8_t *error);

// --- Optional: Mixed controls (not yet implemented in .c, just listed for completeness) ---
bool ForwardMixed(uint8_t address, uint8_t speed);
bool BackwardMixed(uint8_t address, uint8_t speed);
bool TurnRightMixed(uint8_t address, uint8_t speed);
bool TurnLeftMixed(uint8_t address, uint8_t speed);
bool ForwardBackwardMixed(uint8_t address, uint8_t speed);
bool LeftRightMixed(uint8_t address, uint8_t speed);

// --- Optional: Stop helpers (not implemented yet, but declared) ---
bool StopM1(uint8_t address);
bool StopM2(uint8_t address);
bool StopAll(uint8_t address);

