# Embedded Systems Drone

## Description

This project implements a WiFi-controlled drone using ESP32 microcontroller with MPU6050 accelerometer/gyroscope for stabilization. The drone can be controlled via a web interface that displays real-time orientation data and allows adjustment of PID parameters.

### Features

The system has the following features:

- PID control for drone stabilization
- Web-based controller dashboard
- UDP communication for emergency stop and telemetry
- Power-saving sleep mode
- Watchdog timer for system reliability

## Prerequisites

### Hardware

#### Drone:
- ESP32CAM development board
- ESP32-CAM-MB micro USB programmer
- Jumper wires
- MPU6050 6-axis accelerometer/gyroscope
- 4x 8520 brushed motors
- 4x MOSFET-based motor drivers
- 2x LiPo batteries
- WiFi network
- Drone frame STL files

#### Telemetry & Emergency Stop Controller:
- ESP32 development board
- 16x2 I2C LCD
- Button
- 10kΩ resistor

### Software

- ESP-IDF
- Arduino Core for ESP32
- Required libraries:
  - `MPU6050` (I2C communication)
  - `WiFi`
  - `ESPAsyncWebServer`
  - `WiFiUdp`

## Get Started

1. Clone this repository:
   ```bash
   git clone https://github.com/kadirhan0168/Embedded-Systems-Drone.git
   ```
2. Set up ESP-IDF development environment.
3. Add Arduino as a component to your project.
4. Configure the project:

   ```bash
   idf.py menuconfig
   ```

## Setup

### Drone:

1. Build the drone.
2. Wire all components according to `docs/schematics/embedded-systems-drone.pdf`.
3. Adjust WiFi credentials in `main/main.cpp`.
4. Connect ESP32-CAM-MB micro USB programmer to the onboard ESP32CAM module through jumper wires. Make sure to also connect IO0 to GND when flashing.
5. Set correct serial port.
6. Build and flash the project using UART.
7. Open the serial monitor to find the controller dashboard IP.

### Telemetry & Emergency Stop Controller:

1. Build the Telemetry & Emergency Stop Controller according to `docs/schematics/embedded-systems-telemetry-controller.pdf`.
2. Power the ESP32 through USB.

## Usage

The drone will initialize its system and connect to WiFi when powered on. Connect to the controller dashboard by accessing the drone's IP address (acquired from serial monitor) in a web browser. 

The power slider increases the power output of the motors. The power value is the PWM signal with 0 being the lowest and 255 being the highest power output. The directional buttons move the drone along the X- and Y-axes through roll & pitch movements. The rotation controls handle the yaw movements to rotate the drone along the Z-axis.

The PID terms can be adjusted through the controller dahsboard using the `Kp`, `Ki`, and `Kd` fields. :warning:**Attention**:warning: — The PID terms **are not** saved, they will get lost if the connection to the drone disconnects. Save the PID terms manually if they are of importance.

The Telemetry & Emergency Stop Controller will initialize its system and attempt connecting to the drone's access point when powered on. It will display the drone's pitch and roll values on the LCD when the connection has been set up. Pressing the button will trigger an interrupt cutting all power to the drone's motors immediately.