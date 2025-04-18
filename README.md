# Test 1: Watchdog timer

**Context**  
This test verifies whether the watchdog timer in the ESP32 microcontroller functions correctly by detecting system faults and restarting the drone.

**Preparation**  
The ESP32 microcontroller and a debug interface for fault detection were set up.

**Execution**  
The test involved simulating a system fault to check if the watchdog timer would restart the drone within the required 3 seconds.

**Results**  
The watchdog timer functioned correctly. The drone was restarted within the expected time after detecting a system fault. No communication issues were observed during the test.

**Conclusion**  
The watchdog timer test was successful. The function works as required and restarts the drone within the specified time.

---

# Test 2: Sleep mode

**Context**  
This test checks the functionality of the sleep mode on the ESP32, which helps save energy during inactivity.

**Preparation**  
The ESP32 microcontroller and, if necessary, energy consumption measuring equipment were set up.

**Execution**  
The test involved verifying if the drone entered sleep mode within 30 seconds of inactivity and successfully woke up upon receiving an input signal.

**Results**  
The sleep mode worked as expected. The drone entered sleep mode during inactivity and was activated within the required time after a trigger.

**Conclusion**  
The sleep mode is effective, and the drone wakes up correctly after receiving input. The test was successful.

---

# Test 3: Use of RTOS / Embedded Linux

**Context**  
This test ensures that the ESP32 is capable of running multiple tasks simultaneously using an RTOS or Embedded Linux.

**Preparation**  
An RTOS or Embedded Linux system was set up on the ESP32, and a multitasking test environment was prepared.

**Execution**  
The test involved running multiple tasks simultaneously on the drone to check for delays or system crashes.

**Results**  
The RTOS functionality was successfully tested. The drone was able to perform multiple tasks simultaneously without delays or crashes.

**Conclusion**  
The RTOS functionality is working well and supports concurrent task processing on the ESP32.

---

# Test 4: Reading data from the MPU6050 sensor with I2C

**Context**  
This test verifies whether the ESP32-CAM can read data from the MPU6050 sensor via I2C communication.

**Preparation**  
Components such as the ESP32-CAM, MPU6050 sensor, I2C connection, and serial monitor were set up.

**Execution**  
The test involved reading accelerometer and gyroscope data from the MPU6050 sensor and displaying this data on the serial monitor.

**Results**  
The reading of the MPU6050 via the ESP32-CAM was successful. Accelerometer and gyroscope values were continuously and reliably displayed on the serial monitor without any errors.

**Conclusion**  
The reading of the MPU6050 works as expected. The data is reliable, and the communication is stable. The test was successful!
