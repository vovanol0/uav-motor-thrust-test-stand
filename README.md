# uav-motor-thrust-test-stand
Arduino-based UAV motor thrust test stand with real-time telemetry visualization and logging.

The system collects telemetry from a load cell, voltage sensor and current sensor, sends the data over a serial connection and visualizes it in a Python desktop application. The application displays live plots and can save test sessions into log files for later analysis.

<img width="1920" height="1080" alt="image" src="https://github.com/user-attachments/assets/f16c5061-9f50-45aa-b677-05f6fc870f89" />

## Features

- Real-time thrust measurement
- Battery voltage monitoring
- Motor current measurement
- Serial telemetry from Arduino
- Automatic serial port detection
- Live plots for thrust, current and voltage
- 5-second median values for smoother readings
- Automatic session detection when battery voltage is detected
- Test session logging into separate files

## Hardware

- Arduino-compatible board
- Load cell with amplifier
- Current sensor
- Voltage sensor
- ESC
- BLDC motor
- UAV propeller
- Battery
- Custom mechanical test stand

## Software

The desktop application is written in Python and uses:

- pyserial for serial communication
- Tkinter for GUI
- built-in plotting on Canvas
- file logging for telemetry data

## Telemetry Format

The Arduino firmware sends named values over Serial:

```text
weight_g: 123.45, current_A: 4.32, voltage_V: 15.80
