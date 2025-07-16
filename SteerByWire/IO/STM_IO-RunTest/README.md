# STM_IO-Blink

Simple LED blink project for STM32F103RC hoverboard using Arduino framework.

## Description

This project demonstrates basic GPIO control by blinking an LED connected to pin PB2 at 1Hz frequency (500ms ON, 500ms OFF).

## Hardware Requirements

- STM32F103RC based hoverboard
- LED connected to pin PB2 (or use onboard LED if available)
- ST-Link programmer/debugger

## Pin Configuration

- **LED_PIN**: PB2 - LED output pin

## Features

- Simple LED blinking at 1Hz frequency
- Serial output for debugging (115200 baud)
- Non-blocking timing using millis()
- Clean Arduino framework implementation

## Building and Uploading

### Prerequisites

1. Install PlatformIO extension in VS Code
2. Connect ST-Link to hoverboard
3. Ensure hoverboard is powered

### Upload Instructions

1. Open this project folder in VS Code
2. PlatformIO should automatically detect the project
3. Connect your ST-Link debugger to the hoverboard
4. Click the PlatformIO upload button or use Ctrl+Alt+U
5. The LED on PB2 should start blinking

### Serial Monitor

To view debug output:
1. Open PlatformIO terminal
2. Use command: `pio device monitor`
3. Or click the monitor icon in PlatformIO toolbar

## Configuration

You can modify the blink frequency by changing the `interval` variable in `src/main.cpp`:

```cpp
const long interval = 500; // 500ms = 1Hz, 250ms = 2Hz, 100ms = 5Hz
```

## Troubleshooting

1. **Upload fails**: Check ST-Link connection and power
2. **LED not blinking**: Verify pin PB2 connection
3. **No serial output**: Check USB connection and baud rate (115200)

## Hardware Connections

```
ST-Link    Hoverboard
-------    ----------
3.3V   ->  3.3V
GND    ->  GND
SWDIO  ->  SWDIO
SWCLK  ->  SWCLK
```

## Development

This project uses:
- Platform: ST STM32
- Framework: Arduino
- Board: Generic STM32F103RC
- MCU: STM32F103RCT6
- Clock: 72MHz
