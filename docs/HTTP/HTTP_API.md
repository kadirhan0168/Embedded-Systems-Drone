# Drone Web Server API Endpoints

## General Control

| Endpoint         | Method | Parameters                    | Description                                                    | Example Response (JSON)                   |
|------------------|--------|-------------------------------|----------------------------------------------------------------|-------------------------------------------|
| `/setMotorPower` | GET    | `value` (0–255)               | Sets the motor power. Wakes up drone if sleeping.              | `{"status":"OK","pitch":1.2,"roll":-0.5}` |
| `/getMPUData`    | GET    | —                             | Returns current pitch and roll values.                         | `{"pitch":0.0,"roll":1.8}`                |
| `/getSleepStatus`| GET    | —                             | Returns whether the drone is currently sleeping.               | `{"isSleeping":false}`                    |

---

## Movement Commands

| Endpoint        | Method | Parameters | Description                          | Response              |
|-----------------|--------|------------|--------------------------------------|-----------------------|
| `/moveForward`  | GET    | —          | Move drone forward                   | `"Moving Up"`         |
| `/moveBackward` | GET    | —          | Move drone backward                  | `"Moving Down"`       |
| `/moveLeft`     | GET    | —          | Move drone to the left               | `"Moving Left"`       |
| `/moveRight`    | GET    | —          | Move drone to the right              | `"Moving Right"`      |
| `/rotateLeft`   | GET    | —          | Rotate drone counter-clockwise       | `"Rotating Left"`     |
| `/rotateRight`  | GET    | —          | Rotate drone clockwise               | `"Rotating Right"`    |
| `/stop`         | GET    | —          | Stops all directional movement       | `"Stopping Movement"` |

---

## PID Tuning

| Endpoint     | Method | Parameters                | Description                          | Example Response                      |
|--------------|--------|---------------------------|-------------------------------------|----------------------------------------|
| `/setPID`    | GET    | `Kp`, `Ki`, `Kd` (floats) | Updates the PID controller constants| `"PID Updated: Kp=2.0 Ki=0.01 Kd=0.5"` |

| Endpoint  | Method | Parameters (floats)  | Description                          | Defaults       | Example Response                       |
|-----------|--------|----------------------|--------------------------------------|----------------|----------------------------------------|
| `/setPID` | GET    | `Kp`, `Ki`, `Kd`     | Updates the PID controller constants | `Kp=1.5`       | `"PID Updated: Kp=2.0 Ki=0.01 Kd=0.5"` |
|           |        |                      |                                      | `Ki=0.003`     |                                        |
|           |        |                      |                                      | `Kd=0.0`       |                                        |