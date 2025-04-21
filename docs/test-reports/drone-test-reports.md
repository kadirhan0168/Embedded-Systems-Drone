# Test report - ESP32 drone functionalities

## Test 1: Hosting webserver as controller dashboard

### Context

This test verifies that the ESP32CAM module is able to connect to WiFi and host a webserver. The webserver contains the HTML for the controller dashboard.

### Requirement

Hosting webserver for controls.

### Acceptance criteria

The ESP32CAM connects to WiFi and hosts the webserver correctly.

### Materials needed

- ESP32CAM
- ESP32-CAM-MB micro USB programmer
- WiFi network
- main.cpp

### Preparation

1. Clone the repository.
2. Connect the ESP32-CAM-MB micro USB programmer to the ESP32CAM module.
3. Select the correct serial port.
4. Build and flash main.cpp.

### Execution

1. Open serial monitor and wait for the IP address to be printed.
2. Connect to the IP address through the browser.

### Results

The browser connected to the webserver and the correct HTML page was being hosted.

### Conclusion

The test passed. The ESP32CAM module was able to connect to WiFi and host the webserver.

----

## Test 2: Throttle

### Context

This test verifies that the drone is able to increase and decrease thrust to all motors using the controller dashboard.

### Requirement  

Ascend
Descend

### Acceptance criteria

Changing the PWM signal using the controllers dashboard's slider should increase or decrease motor power output accordingly.

### Materials needed 

- Fully built drone
- Jumper wires
- ESP32-CAM-MB micro USB programmer
- WiFi network
- main.cpp
- 2x LiPo batteries

### Preparation

1. Clone the repository.
2. Connect the ESP32-CAM-MB micro USB programmer to the ESP32CAM module.
3. Change the WiFi credentials in main.cpp
4. Select the correct serial port.
5. Build and flash main.cpp.
6. Connect LiPo batteries to the drone.

### Execution

1. Open serial monitor and wait for the IP address to be printed.
2. Connect to the IP address through the browser.
3. Adjust the throttle using the slider.
4. Observe whether the drone's motors spin faster as the throttle increases, and slower as it decreases.

### Results

The drone's motors spin faster as the throttle increases. The motors spin slower when the throttle decreases and come to a full stop at a PWM signal of 0.

### Conclusion

The test passed. The motors responded accordingly to the throttle inputs.

----

## Test 3: Pitch & roll in all directions

### Context

This test verifies the movement of the drone along the X- and Y-axes.

### Requirement  

Pitch in four directions.

### Acceptance criteria

The test passes if the drone is able to pitch forward, backward, and roll to the sides to move in all four directions. It also tests wether the motors are configured correctly.

### Materials needed 

- Fully built drone
- Jumper wires
- ESP32-CAM-MB micro USB programmer
- WiFi network
- main.cpp
- 2x LiPo batteries

### Preparation

1. Clone the repository.
2. Connect the ESP32-CAM-MB micro USB programmer to the ESP32CAM module.
3. Change the WiFi credentials in main.cpp
4. Select the correct serial port.
5. Build and flash main.cpp.
6. Connect LiPo batteries to the drone.

### Execution

1. Open serial monitor and wait for the IP address to be printed.
2. Connect to the IP address through the browser.
3. Increase the throttle using the slider while holding the drone at a level where you can feel it starting to lift.
4. Press each of the four directional buttons individually and feel which way the drone wants to move.

### Results

Pressing the directional buttons made the drone want to move in the correct direction. The thrust was high enough to see the drone pulling into each direction.

### Conclusion

The test passed. The drone is able to pitch and roll accordingly based on user inputs. This also verifies that the motors are configured correctly.

----

## Test 4: Yaw left and right

### Context

This test verifies the rotational movement of the drone along the Z-axis (yaw).

### Requirement  

Yaw left and right.

### Acceptance criteria

The test passes if the drone is able to rotate clockwise and counterclockwise when the respective yaw buttons are pressed.

### Materials needed 

- Fully built drone
- Jumper wires
- ESP32-CAM-MB micro USB programmer
- WiFi network
- main.cpp
- 2x LiPo batteries

### Preparation

1. Clone the repository.
2. Connect the ESP32-CAM-MB micro USB programmer to the ESP32CAM module.
3. Change the WiFi credentials in main.cpp
4. Select the correct serial port.
5. Build and flash main.cpp.
6. Connect LiPo batteries to the drone.

### Execution

1. Open serial monitor and wait for the IP address to be printed.
2. Connect to the IP address through the browser.
3. Increase the throttle using the slider while holding the drone at a level where you can feel it starting to lift.
4. Press each of the two yaw buttons individually and feel which way the drone wants to move.

### Results

Pressing the yaw buttons made the drone want to move in the correct direction.

### Conclusion

The test passed. The drone is able to rotate along the Z-axis in both directions.

## Test 5: Reading the MPU6050 sensor via I2C

### Context
This test verifies whether the ESP32CAM can read data from the MPU6050 sensor via I2C communication.

### Requirement
The ESP32CAM must be able to read data from a connected MPU6050 sensor via I2C.

### Acceptance criteria
The sensor must return correct acceleration and gyroscopic data.

### Materials needed
- ESP32CAM
- ESP32-CAM-MB micro USB programmer
- MPU6050
- Jumper wires
- main.cpp

### Preparation
1. Connect the MPU6050 to the ESP32CAM module using jumper wires.
2. Select the correct serial port.
3. Build and flash main.cpp.

### Execution

1. Open the serial monitor.
2. Move the MPU6050 around and check whether the incoming data is correct and reliable. The stream of data should be stable and uninterrupted.

### Conclusion
The test passed. The MPU6050 readout works as expected. The data is reliable, and communication is stable. The test passed successfully.

----

## Test 6: Balance drone using MPU6050

### Context

This test verifies the balancing of the drone using the MPU6050 module and PID controller.

### Requirement  

Balance drone using gyroscope.

### Acceptance criteria

The test passes if the drone is able to ascend, maintain altitude and balance itself without drifting away more than 10 centimeters from it's initial position during 30 seconds of flight.

### Materials needed 

- Fully built drone
- Jumper wires
- ESP32-CAM-MB micro USB programmer
- WiFi network
- main.cpp
- 2x LiPo batteries
- Duct tape
- Stopwatch

### Preparation

1. Clone the repository.
2. Connect the ESP32-CAM-MB micro USB programmer to the ESP32CAM module.
3. Change the WiFi credentials in main.cpp
4. Select the correct serial port.
5. Build and flash main.cpp.
6. Connect LiPo batteries to the drone.
7. Stick a piece of duct tape to the floor and place the drone on top of it.

### Execution

1. Open serial monitor and wait for the IP address to be printed.
2. Connect to the IP address through the browser.
3. Increase the throttle using the slider until the drone starts ascending and is approximately 50 centimeters above the floor.
4. Start the stopwatch and observer the drone.

### Results

The drone struggles to maintain balance and altitude. It loses balance and crashes shortly after ascending.

### Conclusion

The test failed. The drone tries to stabilize too aggressively is not able to maintain flight. The PID controller needs further calibration.

----

## Test 7: Watchdog timer

### Context
This test verifies whether the watchdog timer in the ESP32 microcontroller functions correctly by detecting system faults and restarting the drone.

### Requirement
The ESP32 must use a watchdog timer that automatically restarts the system in case of errors.

### Acceptance criteria
The watchdog must restart the drone within 3 seconds after detecting a system error or freeze.

### Materials needed
- Fully built drone
- ESP32-CAM-MB micro USB programmer
- Jumper wires
- main.cpp

### Execution
1. Open the serial monitor.
2. Observe whether the watchdog timer gets triggered.

### Results
The watchdog timer functioned correctly. The drone restarted within the expected time after detecting a system error. No communication issues were observed during the test.

### Conclusion
The watchdog timer test was successful. The function works as required and restarts the drone within the specified time.

----

## Test 8: Sleep mode

### Context
This test checks the functionality of the sleep mode on the ESP32, which helps save energy during inactivity.

### Requirement
The ESP32 must automatically enter sleep mode during inactivity to conserve energy.

### Acceptance criteria
The drone must enter sleep mode within a minute of inactivity and correctly wake up upon receiving an input signal.

### Materials needed
- Fully built drone
- ESP32-CAM-MB micro USB programmer
- Jumper wires
- main.cpp
- WiFi network
- Stopwatch

### Execution
1. Open serial monitor and wait for the IP address to be printed.
2. Connect to the IP address through the browser.
3. Start the stopwatch and wait for the drone to go into sleep mode.

### Results
The sleep mode functioned as expected. The drone entered sleep mode after a minute of inactivity and was reactivated immediately after a trigger. A message was displayed on the controller dashboard to notify the user that the drone was in sleep mode. The MPU6050 was also powered off during this state.

### Conclusion
Sleep mode is effective, and the drone wakes correctly upon receiving input. The test passed successfully.