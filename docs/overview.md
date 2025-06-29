# Project Overview: Embedded Systems Drone

This document provides a high-level overview of the system design and architecture for the Embedded Systems Drone project. It links to all major documentation files, including hardware schematics, software architecture, 3D designs, API documentation, and test reports.

---

## 1. Hardware Design

### 1.1 Drone Hardware

- [Drone Circuit Schematics (PDF)](schematics/embedded-systems-drone.pdf)
  
  Wiring diagram for the drone's components, including the ESP32-CAM, MPU6050 IMU, motor drivers, and power supply.

- [Drone Frame STL File (V3)](designs/V3/drone.stl)
  
  3D-printable STL models of the drone's frame designed for an X-configuration with four arms:
  - V1 was designed for use with 716 motors.
  - V2 was designed for use with 8520 motors.
  - V3 is a bigger frame whic was also designed for use with 8520 motors.

### 1.2 Telemetry & Emergency Stop Controller

- [Controller Circuit Schematics (PDF)](schematics/embedded-systems-telemetry-controller.pdf)

  Full schematic for the telemetry controller, including button wiring, I²C LCD, and UDP communication to the drone.

### 1.3 Hardware Architecture

- [System Architecture Diagram](diagrams/architecture-design-embedded-systems.png)
  
  High-level component diagram showing the hardware and subsystem layout of both the drone and the telemetry controller. Arrows represent interaction and data flow between physical components and modules.

---

## 2. Software Design

### 2.1 Software Architecture
- [Data Flow Diagram](diagrams/dataflow-embedded-systems.png)

  Diagram outlining how data moves between components in the system, such as I²C, UDP, PWM.

### 2.2 Flowcharts

- [Drone Flowchart](diagrams/flowchart-drone-embedded-systems.png)

  Step-by-step control logic of the drone’s firmware, including tasks like PID control, WiFi, telemetry broadcast, UDP listener.

- [Telemetry Controller Flowchart](diagrams/flowchart-telemetry-embedded-systems.png)

  Logical flow of the telemetry controller from setup to emergency stop trigger and telemetry display.

### 2.3 HTTP API

- [HTTP Endpoint Documentation](HTTP/HTTP_API.md)

  Full list and explanation of HTTP endpoints exposed by the drone’s web server, including `/setMotorPower`, `/getMPUData`, and `/setPID`.

---

## 3. Test Reports

- [Test Reports](test-reports/drone-test-reports.md)

  Results from testing the drone's core features.

---

## Related Code Files

| Module                    | Location                        |
|---------------------------|----------------------------------|
| Drone Firmware            | `firmware/drone/main/main.cpp`            |
| Telemetry Controller Code | `firmware/telemetry_controller/drone_telemetry_controller.ino` |

---

## 🔗 Quick Links

- [Project README](../README.md)
- [Setup Instructions](../README.md#get-started)
