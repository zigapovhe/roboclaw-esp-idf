# RoboClaw ESP-IDF Library

Unofficial port of the [Basicmicro Arduino Library](https://github.com/basicmicro/basicmicro_arduino) for ESP-IDF.

**This is unofficial - not made by or endorsed by BasicMicro!**

## Features

### Motor Control
- `ForwardM1()`, `BackwardM1()`, `ForwardM2()`, `BackwardM2()` — basic speed control
- `DutyM1()`, `DutyM2()`, `DutyM1M2()` — direct PWM duty cycle (-32767 to +32767)
- `DutyAccelM1()`, `DutyAccelM2()`, `DutyAccelM1M2()` — duty cycle with hardware acceleration ramp
- `SpeedM1()`, `SpeedM2()` — encoder-based speed control
- `SpeedAccelM1()`, `SpeedAccelM2()` — speed with acceleration
- `ForwardMixed()`, `BackwardMixed()`, `TurnRightMixed()`, `TurnLeftMixed()` — mixed mode
- `StopM1()`, `StopM2()`, `StopAll()` — stop helpers

### Encoders
- `ReadEncM1()`, `ReadEncM2()` — read encoder counts
- `ReadSpeedM1()`, `ReadSpeedM2()` — read encoder speeds
- `ResetEncoders()` — reset encoder counts to zero

### Diagnostics
- `ReadVersion()` — firmware version string
- `ReadError()` — 32-bit error/warning status with detailed flags
- `GetStatus()` — comprehensive status in one call (voltages, temps, currents, encoders, speeds, errors)
- `ReadMainBatteryVoltage()`, `ReadLogicBatteryVoltage()` — battery voltages
- `ReadTemp()`, `ReadTemp2()` — board temperatures
- `ReadCurrents()` — motor currents
- `ReadBuffers()` — command buffer depths
- `ReadPWMs()` — current PWM output values

### Configuration
- `SetTimeout()`, `GetTimeout()` — communication timeout (auto-stop if no commands received)
- `SetMainVoltages()`, `ReadMinMaxMainVoltages()` — main battery voltage limits
- `SetLogicVoltages()`, `ReadMinMaxLogicVoltages()` — logic battery voltage limits
- `SetM1MaxCurrent()`, `SetM2MaxCurrent()` — per-motor current limits
- `ReadM1MaxCurrent()`, `ReadM2MaxCurrent()` — read current limits
- `WriteNVM()` — save settings to flash
- `ReadNVM()` — reload settings from flash
- `RestoreDefaults()` — factory reset

### Thread Safety
- `roboclaw_enable_thread_safety()` — opt-in UART mutex for multi-task access (zero overhead if not enabled)

## Hardware Setup

**Default pins:**
- TX: GPIO 17 → RoboClaw S1 (RX)
- RX: GPIO 16 ← RoboClaw S2 (TX)
- Don't forget GND!

**Default baud rate:** 38400

Multiple RoboClaws can share the same TX/RX pins using different addresses (0x80, 0x81, etc.).

## Basic Usage

```c
#include "roboclaw.h"
#include "roboclaw_uart.h"

#define ROBOCLAW_ADDRESS 0x80

void app_main(void) {
    begin(38400);

    // Optional: enable thread safety if using multiple FreeRTOS tasks
    // roboclaw_enable_thread_safety();

    char version[64];
    if (ReadVersion(ROBOCLAW_ADDRESS, version)) {
        printf("Version: %s\n", version);
    }

    // Set duty with acceleration ramp
    DutyAccelM1M2(ROBOCLAW_ADDRESS, 16000, 5000, 16000, 5000);
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Stop
    DutyM1M2(ROBOCLAW_ADDRESS, 0, 0);
}
```

## Installation

1. Copy the `components/roboclaw/` folder to your ESP-IDF project
2. Include the headers in your code
3. Build with `idf.py build`

## Configuration

Want different pins? Edit `roboclaw_uart.c`:
```c
#define TXD_PIN 17  // Your TX pin
#define RXD_PIN 16  // Your RX pin
```

Want different baud rate? Just change it:
```c
begin(115200);  // Instead of 38400
```

## Original Library

Based on BasicMicro's [official Arduino library](https://github.com/basicmicro/basicmicro_arduino).

## Compatibility

- ESP-IDF 5.0+
- ESP32, ESP32-S2, ESP32-S3, ESP32-C3
- Any RoboClaw with packet serial communication

## Issues?

For RoboClaw hardware problems: contact [BasicMicro](https://www.basicmicro.com/)
For this ESP-IDF port: open an issue here
