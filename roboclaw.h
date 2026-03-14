#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "roboclaw_crc.h"

// Default RoboClaw address
#define ROBOCLAW_DEFAULT_ADDRESS 0x80

// RoboClaw 32-bit error/warning status bits (from command 90)
#define ROBOCLAW_ERROR_NONE           0x00000000
#define ROBOCLAW_ERROR_ESTOP          0x00000001
#define ROBOCLAW_ERROR_TEMP           0x00000002
#define ROBOCLAW_ERROR_TEMP2          0x00000004
#define ROBOCLAW_ERROR_LBATT_HIGH     0x00000010
#define ROBOCLAW_ERROR_LBATT_LOW      0x00000020
#define ROBOCLAW_ERROR_SPEED1         0x00000100
#define ROBOCLAW_ERROR_SPEED2         0x00000200
#define ROBOCLAW_ERROR_POS1           0x00000400
#define ROBOCLAW_ERROR_POS2           0x00000800
#define ROBOCLAW_ERROR_CURRENT_M1     0x00001000
#define ROBOCLAW_ERROR_CURRENT_M2     0x00002000
#define ROBOCLAW_WARN_OVERCURRENT_M1  0x00010000
#define ROBOCLAW_WARN_OVERCURRENT_M2  0x00020000
#define ROBOCLAW_WARN_MBATT_HIGH      0x00040000
#define ROBOCLAW_WARN_MBATT_LOW       0x00080000
#define ROBOCLAW_WARN_TEMP            0x00100000
#define ROBOCLAW_WARN_TEMP2           0x00200000
#define ROBOCLAW_WARN_S4              0x00400000
#define ROBOCLAW_WARN_S5              0x00800000
#define ROBOCLAW_WARN_BOOT            0x20000000
#define ROBOCLAW_WARN_OVERREGEN_M1    0x40000000
#define ROBOCLAW_WARN_OVERREGEN_M2    0x80000000

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
    SETTIMEOUT = 14,
    GETTIMEOUT = 15,
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
    GETSTATUS = 73,
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
bool DutyM1M2(uint8_t address, int16_t duty1, int16_t duty2);

// --- Duty cycle with acceleration ---
bool DutyAccelM1(uint8_t address, int16_t duty, uint32_t accel);
bool DutyAccelM2(uint8_t address, int16_t duty, uint32_t accel);
bool DutyAccelM1M2(uint8_t address, int16_t duty1, uint32_t accel1, int16_t duty2, uint32_t accel2);

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

// --- Timeout ---
// timeout_ds: timeout in deciseconds (tenths of a second), 0-255 (0 = disabled, 10 = 1 second)
bool SetTimeout(uint8_t address, uint8_t timeout_ds);
uint8_t GetTimeout(uint8_t address, bool *valid);

// --- Voltage limits ---
// Voltages in tenths of a volt (e.g., 60 = 6.0V, 340 = 34.0V)
bool SetMainVoltages(uint8_t address, uint16_t min, uint16_t max, uint8_t auto_max);
bool SetLogicVoltages(uint8_t address, uint16_t min, uint16_t max);
bool ReadMinMaxMainVoltages(uint8_t address, uint16_t *min, uint16_t *max, uint8_t *auto_max);
bool ReadMinMaxLogicVoltages(uint8_t address, uint16_t *min, uint16_t *max);

// --- Current limits ---
// Current values in 10mA units (e.g., 1000 = 10A)
bool SetM1MaxCurrent(uint8_t address, uint32_t max, uint32_t min);
bool SetM2MaxCurrent(uint8_t address, uint32_t max, uint32_t min);
bool ReadM1MaxCurrent(uint8_t address, uint32_t *max, uint32_t *min);
bool ReadM2MaxCurrent(uint8_t address, uint32_t *max, uint32_t *min);

// --- Buffers and PWMs ---
bool ReadBuffers(uint8_t address, uint8_t *depth1, uint8_t *depth2);
bool ReadPWMs(uint8_t address, int16_t *pwm1, int16_t *pwm2);

// --- NVM and config ---
bool WriteNVM(uint8_t address);
bool ReadNVM(uint8_t address);
bool RestoreDefaults(uint8_t address);

// --- Status and diagnostics ---
bool ReadVersion(uint8_t address, char *version);
uint32_t ReadError(uint8_t address, bool *valid);
bool GetStatus(uint8_t address, uint32_t *tick, uint32_t *state,
               uint16_t *temp1, uint16_t *temp2,
               uint16_t *main_batt, uint16_t *logic_batt,
               int16_t *pwm1, int16_t *pwm2,
               int16_t *current1, int16_t *current2,
               uint32_t *enc1, uint32_t *enc2,
               uint32_t *speed1, uint32_t *speed2,
               uint32_t *ispeed1, uint32_t *ispeed2,
               uint16_t *speed_error1, uint16_t *speed_error2,
               uint16_t *pos_error1, uint16_t *pos_error2);

// --- Optional: Mixed controls ---
bool ForwardMixed(uint8_t address, uint8_t speed);
bool BackwardMixed(uint8_t address, uint8_t speed);
bool TurnRightMixed(uint8_t address, uint8_t speed);
bool TurnLeftMixed(uint8_t address, uint8_t speed);
bool ForwardBackwardMixed(uint8_t address, uint8_t speed);
bool LeftRightMixed(uint8_t address, uint8_t speed);

// --- Stop helpers ---
bool StopM1(uint8_t address);
bool StopM2(uint8_t address);
bool StopAll(uint8_t address);

