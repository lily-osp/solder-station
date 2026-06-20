# Soldering Iron Controller

## Table of Contents
1. [Introduction](#introduction)
2. [Features](#features)
3. [Hardware Requirements](#hardware-requirements)
4. [Software Dependencies](#software-dependencies)
5. [Installation](#installation)
6. [Wiring Guide](#wiring-guide)
7. [Usage](#usage)
8. [Temperature Control System](#temperature-control-system)
9. [Display Options](#display-options)
10. [Safety Features](#safety-features)
11. [Persistent Settings](#persistent-settings)
12. [Troubleshooting](#troubleshooting)
13. [Maintenance](#maintenance)
14. [Simulation](#simulation)
15. [Adaptive PID Control](#adaptive-pid-control)
16. [Flowchart](#flowchart)
17. [Contributing](#contributing)

## Introduction

The **Soldering Iron Controller** is an Arduino-based system designed to precisely manage soldering iron temperatures. It supports PID-based control, a user-friendly rotary encoder, and a vibrant LED status indicator, making it an ideal tool for hobbyists and professionals.

## Features

- Precise temperature control (100°C to 500°C range)
- PID-based temperature regulation with adaptive tuning (steady-state only)
- Exponential Moving Average (EMA) filtering for stable PID inputs
- Temperature ramping (rapid heating with customizable duration and target temperature)
- Two-stage standby: Sleep Mode (heats to 150°C or sleep setting) and Auto Shut-Off
- Boost Mode (+50°C for 45 seconds via double-click)
- Configuration Menu (Sleep Temp, Sleep Time, Off Time, Temp Unit, Ramp Time, Ramp Temp) with infinite scrolling loop
- OLED (SSD1306) or 16x2 I2C LCD display support
- WS2812 RGB LED for status indication (pulsing blue in sleep, warning colors)
- Software-debounced input controls (50ms window)
- EEPROM write preservation (asynchronous save after 2 seconds of inactivity)
- Watchdog timer for system reliability
- Sensor fault, thermal runaway, and overheat protections

## Hardware Requirements

- Arduino (e.g., Arduino Uno, Nano)
- OLED Display (SSD1306) or 16x2 I2C LCD Display
- WS2812 RGB LED
- Rotary Encoder with push button
- Analog temperature sensor
- MOSFET (for iron control)
- Soldering iron
- Power supply
- Buzzer
- Resistors, capacitors (refer to wiring guide)

## Software Dependencies

Ensure the following libraries are installed:

- **Wire.h** (built-in)
- **Adafruit_GFX.h** (for OLED)
- **Adafruit_SSD1306.h** (for OLED)
- **LiquidCrystal_I2C.h** (for LCD)
- **BigNumbers_I2C.h** (for large font on LCD)
- **Adafruit_NeoPixel.h** (for WS2812 LED)
- **EEPROM.h** (built-in)
- **PID_v1.h** (for PID control)

## Installation

1. Clone or download this repository.
2. Open the `.ino` file in the Arduino IDE.
3. Install required libraries via Library Manager.
4. Adjust pin assignments in the code to match your wiring.
5. Choose between OLED and LCD display by toggling the `USE_OLED` define in the code.
6. Upload the code to your Arduino board.

## Wiring Guide

Follow the wiring instructions for correct setup:

1. **Temperature Sensing**:
   - Connect the sensor to Arduino pin A0.

2. **Iron Control**:
   - Connect the MOSFET gate to pin D10.

3. **Display**:
   - OLED: Connect SDA to A4, SCL to A5.
   - LCD: Same connections (SDA to A4, SCL to A5).

4. **WS2812 LED**:
   - Data pin connects to D12.

5. **Rotary Encoder**:
   - CLK to D3, DT to D4, switch to D5.

6. **Buzzer**:
   - Connect to D2.

7. **LED Off Indicator**:
   - Connect to D8.

8. Ensure all components share a common ground and proper power connections.

## Usage

1. Power on the controller; the last used temperature will be loaded.
2. Adjust the temperature with the rotary encoder when in active mode.
3. Status LEDs indicate:
   - **Red**: Heating
   - **Green**: Ready
   - **Blue**: Cooling (or pulsing blue in sleep)
   - **Yellow**: Warning (near max temperature, or dangerously hot when OFF)
   - **Purple**: Ramping
   - **Flashing Red/Beeping**: Error State
   - **Off**: Iron off or auto-shutoff (unless still dangerously hot)
4. Button Controls:
   - **Single Click (Active)**: Start Ramping Mode (heat to 500°C for 20s) or toggle OFF (if already ramping).
   - **Single Click (OFF/Sleep/Error)**: Wake up / Toggle ON / Clear active errors.
   - **Double Click (Active)**: Toggle Boost Mode (+50°C for 45s).
   - **Long Press (1.5s)**: Enter/Exit Settings Menu (saves settings to EEPROM on exit).
5. Settings Menu:
   - Navigate settings by rotating the encoder (infinite loop wrapping).
   - Click to enter Edit Mode (indicated by brackets e.g. `[value]`), rotate to adjust value, click again to save.
   - Hold button for 1.5s to save and exit back to operation mode.

## Temperature Control System

### PID Control
The controller uses a PID algorithm to maintain stable temperatures:
- **Kp**: 2
- **Ki**: 5
- **Kd**: 1

### Temperature Ramping
- Activated by pressing the encoder button when system is active.
- Ramps to the max temperature (500°C) over 20 seconds, displaying a countdown.
- Returns to the previous setpoint after ramping.

## Display Options

Choose between two display types:
1. **OLED Display (SSD1306)**
2. **16x2 I2C LCD**

Switch displays by commenting/uncommenting the `USE_OLED` define in the code.

## Safety Features

- **Sensor Disconnection & Short-Circuit Detection**: Continuous analog read checking. Raw values `<= 2` or `>= 1021` shut down the heater, activate the buzzer alarm, flash the status LED, and lock the system in an error state.
- **Thermal Runaway Protection**: Activates if the heater applies significant power (`PWM > 150`) but the temperature fails to rise by at least 3.0°C over a 15-second window. Prevents heater damage if the sensor detaches.
- **Overheat Protection**: Shuts down the system if the temperature exceeds a safe threshold of `MAX_TEMP + 10` (510°C).
- **Error Recovery**: Pressing the button once resolves and clears error states after checking hardware connections.
- **Automatic Shut-off**: After 10 minutes of inactivity, the iron is turned off.
- **Watchdog Timer**: Resets the system if responsive loop hangs.

## Persistent Settings

The system saves the last temperature setting in EEPROM, allowing it to persist across power cycles. It saves asynchronously after 2 seconds of knob inactivity to protect EEPROM write lifespan.

## Troubleshooting

- **Inaccurate Temperature Readings**: Check sensor wiring.
- **Iron Not Heating**: Check MOSFET and power supply.
- **LED Not Working**: Ensure WS2812 wiring and library installation.
- **System Freezing**: Verify connections and PID stability.
- **PID Tuning Issues**: Adjust Kp, Ki, Kd as needed.
- **Sensor/Thermal Error State**: Verify connection and press the button once to clear.

## Maintenance

- Inspect connections regularly.
- Clean the temperature sensor periodically.
- Update firmware as new versions become available.
- Recalibrate the sensor if readings become inaccurate.

## Simulation

Test the setup in [Wokwi](https://wokwi.com/projects/408754608532252673).

## Adaptive PID Control

The controller adapts PID values based on system behavior:
- **Steady-State Monitoring**: The system adapts PID values every 225 loops, but only when within 15°C of the setpoint. This prevents gains from winding up during transient heating or ramping phases.
- **Error Handling**: Sum of errors and counts are logged for adaptation.
- **Overshoot Protection**: Limits the temperature overshoot to 10°C.

## Flowchart

The workflow diagram is stored in [flowchart.md](etc/flowchart.md).

## Contributing

Contributions are welcome! Fork this repository, make your changes, and submit a pull request.
