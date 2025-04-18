# Test report - ESP32 drone functionalities

## Test 1: Watchdog timer

**Context**
This test verifies whether the watchdog timer in the ESP32 microcontroller functions correctly by detecting system faults and restarting the drone.

**Requirement**  
  The ESP32 must use a watchdog timer that automatically restarts the system in case of errors.

**Acceptance criteria**  
  The watchdog must restart the drone within 3 seconds after detecting a system error or freeze.

**Materials needed**  
  1. ESP32 Microcontroller  
  2. Debug interface for fault injection and monitoring

- **Description**  
  This test verifies whether the ESP32's watchdog timer functions correctly by detecting a system error and automatically restarting the drone.

- **Results**  
  The watchdog timer functioned correctly. The drone restarted within the expected time after detecting a system error. No communication issues were observed during the test.

- **Conclusion**  
  The watchdog timer test was successful. The function works as required and restarts the drone within the specified time.

---

## Test 2: Sleep mode

**Context**
This test checks the functionality of the sleep mode on the ESP32, which helps save energy during inactivity.

**Requirement**  
  The ESP32 must automatically enter sleep mode during inactivity to conserve energy.

**Acceptance criteria**  
  The drone must enter sleep mode within 30 seconds of inactivity and correctly wake up upon receiving an input signal.

**Materials needed**  
  1. ESP32 Microcontroller  
  2. Power consumption monitoring (optional)  

**Description**  
  This test checks if the sleep mode functions properly and whether the drone correctly wakes up after a trigger.

**Results**  
  The sleep mode functioned as expected. The drone entered sleep mode during inactivity and was reactivated within the required time after a trigger.

**Conclusion**  
  Sleep mode is effective, and the drone wakes correctly upon receiving input. The test passed successfully.

---

## Test 3: Use of RTOS / Embedded Linux

**Context**
This test ensures that the ESP32 is capable of running multiple tasks simultaneously using an RTOS or Embedded Linux.

**Requirement**  
  The drone must run on an RTOS or Embedded Linux system to handle concurrent task execution.

**Acceptance criteria**  
  The system must be capable of running multiple tasks simultaneously without delays or system crashes.

- **Materials needed**  
  1. ESP32 Microcontroller  
  2. RTOS or Embedded Linux distribution  
  3. Multitasking test environment

**Description**  
  This test verifies whether RTOS usage works correctly on the ESP32 and whether multiple processes can run stably.

**Results**  
  RTOS functionality was successfully tested. The drone was able to execute multiple tasks simultaneously without delays or crashes.

**Conclusion**  
  RTOS functionality is working well and supports concurrent task processing on the ESP32.

---

## Test 4: Reading the MPU6050 sensor via I2C

**Context**
This test verifies whether the ESP32-CAM can read data from the MPU6050 sensor via I2C communication.

**Requirement**  
  The ESP32-CAM must be able to read data from a connected MPU6050 sensor via I2C.

**Acceptance criteria**  
  - The sensor must return correct values (acceleration and gyroscopic data).  
  - Data readings must be repeatable and stable without errors.

**Materials needed**  
  1. ESP32-CAM  
  2. MPU6050 sensor  
  3. I2C connection  
  4. Serial monitor (ESP-IDF)  

**Description**  
  This test verifies the reading of sensor values from the MPU6050 via I2C. The ESP32-CAM receives acceleration and rotation data from the sensor and displays it via the serial monitor.

**Results**  
  The reading of the MPU6050 via the ESP32-CAM was successful. Accelerometer and gyroscope values were continuously displayed on the serial monitor without any issues.

**Conclusion**  
  The MPU6050 readout works as expected. The data is reliable, and communication is stable. The test passed successfully.
