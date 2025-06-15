# RoboClaw ESP-IDF Library

Unofficial port of the [RoboClaw Arduino Library](https://github.com/basicmicro/roboclaw_arduino_library) for ESP-IDF.

## Why I made this

I needed to use a RoboClaw motor controller in my ESP-IDF project, but the existing library is for Arduino. So I ported it to work with ESP-IDF.

**⚠️ This is unofficial - not made by or endorsed by BasicMicro!**

## What works

- Motor control: `ForwardM1()`, `BackwardM1()`, `SpeedM1()`, etc.
- Reading encoders: `ReadEncM1()`, `ReadEncM2()`
- Reading speeds: `ReadSpeedM1()`, `ReadSpeedM2()`
- Battery voltage: `ReadMainBatteryVoltage()`, `ReadLogicBatteryVoltage()`
- Version info: `ReadVersion()`
- Reset encoders: `ResetEncoders()`

## Hardware setup

**Default pins:**
- TX: GPIO 17 → RoboClaw S1 (RX)
- RX: GPIO 16 ← RoboClaw S2 (TX)
- Don't forget GND!

**Default baud rate:** 38400

## Basic usage

```c
#include "roboclaw.h"
#include "roboclaw_uart.h"

#define ROBOCLAW_ADDRESS 0x80

void app_main(void) {
    // Start UART
    begin(38400);
    
    // Read version
    char version[64];
    if (ReadVersion(ROBOCLAW_ADDRESS, version)) {
        printf("Version: %s\n", version);
    }
    
    // Move motor forward at 50% speed
    ForwardM1(ROBOCLAW_ADDRESS, 64);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Stop
    ForwardM1(ROBOCLAW_ADDRESS, 0);
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

## How it's different from Arduino version

- Uses ESP-IDF UART instead of Arduino Serial
- Simpler communication (no complex retries)
- Works with FreeRTOS
- Only includes the functions I actually needed

## Original library

Based on BasicMicro's [official Arduino library](https://github.com/basicmicro/roboclaw_arduino_library).

## Compatibility

- ESP-IDF 5.0+
- ESP32, ESP32-S2, ESP32-S3, ESP32-C3
- Any RoboClaw with serial communication

## Issues?

For RoboClaw hardware problems: contact [BasicMicro](https://www.basicmicro.com/)  
For this ESP-IDF port: open an issue here

---

**Note:** This is just my personal port. For official ESP32 support, ask BasicMicro.