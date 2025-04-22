#include <stdio.h>
#include <string.h>
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mpu6050.h"
#include <driver/i2c.h>
#include "driver/ledc.h"
#include <math.h>
#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <WiFiUdp.h>
#include "esp_task_wdt.h"

#define SSID "ssid"
#define PASSWORD "password"

AsyncWebServer server(80);

#define M_PI 3.14159265358979323846

#define PIN_SDA 14
#define PIN_CLK 15
#define I2C_ADDRESS 0x68

#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_PWR_MGMT_1   0x6B

#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x)   do { esp_err_t rc = (x); if (rc != ESP_OK) { ESP_LOGE("err", "esp_err_t = %d", rc); assert(0 && #x);} } while(0);

// PWM definitions
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES LEDC_TIMER_8_BIT
#define LEDC_FREQUENCY 4000

// MPU6050 definitions
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_SDA_IO 14
#define I2C_MASTER_SCL_IO 15
#define MPU6050_ADDR 0x68

// Motor pins
int motorPin[4] = {
    4, 
    3, 
    13, 
    12
};

// UDP configuration
WiFiUDP udp;
const unsigned int localPort = 1234;
const unsigned int remotePort = 1234;
IPAddress broadcastIP(192, 168, 4, 255);
TaskHandle_t udpTaskHandle = NULL;

// Emergency stop variables
volatile bool emergency_stop = false;
portMUX_TYPE emergency_mux = portMUX_INITIALIZER_UNLOCKED;

// Pet every 500ms
const int WDT_TIMEOUT_SEC = 5;
const int PET_INTERVAL_MS = 500;

static char mpuTag[] = "mpu6050";
static char droneTag[] = "droneControl";
static char emergencyTag[] = "emergencyController";

float pitch = 0.0;
float roll = 0.0;
float targetPitch = 0.0;
float targetRoll = 0.0;

float Kp = 1.5;
float Ki = 0.003; 
float Kd = 0.0;

float pitchError = 0.0;
float rollError = 0.0;
float pitchIntegral = 0.0;
float rollIntegral = 0.0;
float previousPitchError = 0.0;
float previousRollError = 0.0;
float pitchCorrection = 0.0;
float rollCorrection = 0.0;

int motorPower = 0;

bool isSleeping = false;
uint64_t lastActivityTime = 0;
const uint64_t sleepTimeout = 60000;

// I2C initialization
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (err != ESP_OK) {
        return err;
    }

    return i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

// MPU6050 initialization
void setupMPU() {
    esp_err_t ret = i2c_master_init();
    if (ret != ESP_OK) {
        ESP_LOGE(mpuTag, "I2C initialization failed: %s", esp_err_to_name(ret));
        return;
    }

    // Wake up MPU6050
    uint8_t data[2] = {0x6B, 0x00}; // PWR_MGMT_1 register
    ret = i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR, data, sizeof(data), pdMS_TO_TICKS(1000));
    
    if (ret == ESP_OK) {
        ESP_LOGI(mpuTag, "MPU6050 connected and initialized");
    } else {
        ESP_LOGE(mpuTag, "MPU6050 connection failed: %s", esp_err_to_name(ret));
        esp_restart();
    }
}

// Read MPU6050 data
void readMPU() {
    uint8_t accel_data[6];
    uint8_t gyro_data[6];
    
    // Read accelerometer data
    uint8_t reg_addr = 0x3B; // ACCEL_XOUT_H register
    esp_err_t ret = i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_ADDR, 
                                               &reg_addr, 1, accel_data, 6, pdMS_TO_TICKS(1000));
    
    if (ret != ESP_OK) {
        ESP_LOGE(mpuTag, "Failed to read accelerometer data: %s", esp_err_to_name(ret));
        return;
    }
    
    if (ret != ESP_OK) {
        ESP_LOGE(mpuTag, "Failed to read gyroscope data: %s", esp_err_to_name(ret));
        return;
    }
    
    int16_t ax = (accel_data[0] << 8) | accel_data[1];
    int16_t ay = (accel_data[2] << 8) | accel_data[3];
    int16_t az = (accel_data[4] << 8) | accel_data[5];

    // Calculate pitch and roll
    pitch = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / M_PI;
    roll = atan2(-ax, az) * 180.0 / M_PI;
}

// PID controller update
void updatePID() {
    if (motorPower <= 0) {
        pitchIntegral = 0;
        rollIntegral = 0;
        return;
    }

    pitchError = targetPitch - pitch;
    rollError = targetRoll - roll;
    
    ESP_LOGD(mpuTag, "Pitch: %.2f | Roll: %.2f | Pitch Error: %.2f | Roll Error: %.2f", 
             pitch, roll, pitchError, rollError);

    pitchIntegral += pitchError;
    rollIntegral += rollError;

    float pitchDerivative = pitchError - previousPitchError;
    float rollDerivative = rollError - previousRollError;

    pitchCorrection = Kp * pitchError + Ki * pitchIntegral + Kd * pitchDerivative;
    rollCorrection = Kp * rollError + Ki * rollIntegral + Kd * rollDerivative;

    previousPitchError = pitchError;
    previousRollError = rollError;
}

ledc_channel_t led_channels[4] = {
    LEDC_CHANNEL_0, 
    LEDC_CHANNEL_1, 
    LEDC_CHANNEL_2, 
    LEDC_CHANNEL_3
};

// Motor PWM setup
void setupMotors() {
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    for (int i = 0; i < 4; i++) {
        ledc_channel_config_t ledc_channel = {
            .gpio_num    = motorPin[i],
            .speed_mode  = LEDC_LOW_SPEED_MODE,
            .channel     = led_channels[i],
            .intr_type   = LEDC_INTR_DISABLE,
            .timer_sel   = LEDC_TIMER_0,
            .duty        = 0,
            .hpoint      = 0
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));
    }
}

// Stabilize drone
void stabilizeDrone() {
    float motor1Power, motor2Power, motor3Power, motor4Power;
    
    if (motorPower <= 0) {
        motor1Power = motor2Power = motor3Power = motor4Power = 0;
    } else if (motorPower <= 10) {
        motor1Power = motor2Power = motor3Power = motor4Power = motorPower;
    } else {
        motor1Power = motorPower + pitchCorrection + rollCorrection;
        motor2Power = motorPower + pitchCorrection - rollCorrection;
        motor3Power = motorPower - pitchCorrection - rollCorrection;
        motor4Power = motorPower - pitchCorrection + rollCorrection;
    }

    // Constrain motor power values
    motor1Power = constrain(motor1Power, 0, 255);
    motor2Power = constrain(motor2Power, 0, 255);
    motor3Power = constrain(motor3Power, 0, 255);
    motor4Power = constrain(motor4Power, 0, 255);

    ESP_LOGD(droneTag, "Motor Powers: %.1f | %.1f | %.1f | %.1f", 
             motor1Power, motor2Power, motor3Power, motor4Power);

    // Write PWM to motors
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, (uint32_t)motor1Power));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0));
    
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, (uint32_t)motor2Power));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1));
    
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_2, (uint32_t)motor3Power));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_2));
    
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_3, (uint32_t)motor4Power));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_3));
}

// Movement functions
void moveForward() {
    ESP_LOGI(droneTag, "Moving Forward");
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, motorPower));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, motorPower / 2));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_2, motorPower));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_3, motorPower / 2));
    
    // Update duty cycles
    for (int i = 0; i < 4; i++) {
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, led_channels[i]));
    }
}

void moveBackward() {
    ESP_LOGI(droneTag, "Moving Backward");
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, motorPower / 2));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, motorPower));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_2, motorPower / 2));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_3, motorPower));
    
    for (int i = 0; i < 4; i++) {
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, led_channels[i]));
    }
}

void moveLeft() {
    ESP_LOGI(droneTag, "Moving Left");
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, motorPower));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, motorPower));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_2, motorPower / 2));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_3, motorPower / 2));
    
    for (int i = 0; i < 4; i++) {
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, led_channels[i]));
    }
}

void moveRight() {
    ESP_LOGI(droneTag, "Moving Right");
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, motorPower / 2));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, motorPower / 2));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_2, motorPower));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_3, motorPower));
    
    for (int i = 0; i < 4; i++) {
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, led_channels[i]));
    }
}

void rotateLeft() {
    ESP_LOGI(droneTag, "Rotating Left");
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, motorPower));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, motorPower / 2));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_2, motorPower / 2));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_3, motorPower));
    
    for (int i = 0; i < 4; i++) {
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, led_channels[i]));
    }
}

void rotateRight() {
    ESP_LOGI(droneTag, "Rotating Right");
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, motorPower / 2));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, motorPower));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_2, motorPower));
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_3, motorPower / 2));
    
    for (int i = 0; i < 4; i++) {
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, led_channels[i]));
    }
}

void thrust() {
    lastActivityTime = esp_timer_get_time() / 1000;
    if (isSleeping) {
        return;
    }

    ESP_LOGI(droneTag, "Changing thrust");
    for (int i = 0; i < 4; i++) {
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, led_channels[i], motorPower));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, led_channels[i]));
    }
}

void stopMovement() {
    ESP_LOGI(droneTag, "Stopping Movement");
    thrust();
}

void enterSleepMode() {
    ESP_LOGI(droneTag, "Entering sleep mode");
    isSleeping = true;
    
    // Turn off all motors
    for (int i = 0; i < 4; i++) {
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, led_channels[i], 0));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, led_channels[i]));
    }
    
    // Disable MPU6050 to save power
    uint8_t data[2] = {0x6B, 0x40}; // PWR_MGMT_1 register
    i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR, data, sizeof(data), pdMS_TO_TICKS(1000));
}

void checkSleepCondition() {
    if (motorPower == 0 && !isSleeping) {
        uint64_t currentTime = esp_timer_get_time() / 1000; // Convert to milliseconds
        if (currentTime - lastActivityTime > sleepTimeout) {
            enterSleepMode();
        }
    }
}

void wakeupFromSleep() {
    ESP_LOGI(droneTag, "Waking from sleep mode");
    isSleeping = false;
    lastActivityTime = esp_timer_get_time() / 1000;
    
    // Wake up MPU6050
    uint8_t data[2] = {0x6B, 0x00};
    i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDR, data, sizeof(data), pdMS_TO_TICKS(1000));
    setupMotors();
}

void setupWebServer() {
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        String html = "<!DOCTYPE html><html><head><meta charset='UTF-8'><title>Drone Controller</title>";
        html += "<style>body { font-family: Arial, sans-serif; background-color: #1a1a1a; color: white; text-align: center; }";
        html += ".controller { display: grid; grid-template-columns: 1fr 2fr 1fr; gap: 50px; width: 600px; margin: auto; align-items: center; }";
        html += ".left-controls, .right-controls { display: flex; flex-direction: column; align-items: center; gap: 20px; }";
        html += ".middle-controls { display: grid; grid-template-columns: 60px 60px 60px; grid-template-rows: 60px 60px 60px; gap: 10px; justify-content: center; align-items: center; }";
        html += "button { background-color: #444; color: white; border: none; padding: 15px; font-size: 18px; border-radius: 8px; cursor: pointer; width: 60px; height: 60px; }";
        html += "button:hover { background-color: #4CAF50; } input[type=range] { writing-mode: vertical-lr; -webkit-appearance: slider-vertical; width: 20px; height: 200px; transform: rotate(180deg); }";
        html += ".sleep-notification { position: fixed; top: 20px; right: 20px; background-color: #ff5722; padding: 10px 20px; border-radius: 5px; display: ";
        html += isSleeping ? "block" : "none";
        html += "; }";
        html += "</style></head><body>";
        html += "<div class='sleep-notification' id='sleepNotification'>DRONE IS SLEEPING</div>";
        html += "<h1>Drone Controller</h1><div class='controller'>";
        html += "<div class='left-controls'><input type='range' min='0' max='255' value='" + String(motorPower) + "' id='motorPowerSlider' oninput='updateMotorPower()'><label>Motor Power: <span id='motorPowerValue'>" + String(motorPower) + "</span></label></div>";
        html += "<div class='middle-controls'><div></div> <button onclick='moveForward()'>&#9650;</button> <div></div>";
        html += "<button onclick='moveLeft()'>&#9664;</button> <div></div> <button onclick='moveRight()'>&#9654;</button>";
        html += "<div></div> <button onclick='moveBackward()'>&#9660;</button> <div></div></div>";
        html += "<div class='right-controls'><button onclick='rotateLeft()'>&#10226;</button><button onclick='rotateRight()'>&#10227;</button></div></div>";
        
        // Real time pitch/roll data
        html += "<h3>Realtime MPU Data</h3>";
        html += "<p>Pitch: <span id='pitchValue'>0.00</span>°</p>";
        html += "<p>Roll: <span id='rollValue'>0.00</span>°</p>";
    
        html += "<h3>Set PID Values</h3>";
        html += "<label>Kp: <input type='text' id='KpInput' value='" + String(Kp) + "'></label><br>";
        html += "<label>Ki: <input type='text' id='KiInput' value='" + String(Ki) + "'></label><br>";
        html += "<label>Kd: <input type='text' id='KdInput' value='" + String(Kd) + "'></label><br>";
        html += "<button onclick='sendPID()'>Update PID</button>";

        html += "<script>";
        html += "function checkSleepStatus() {";
        html += "  fetch('/getSleepStatus')";
        html += "    .then(response => response.json())";
        html += "    .then(data => {";
        html += "      const notification = document.getElementById('sleepNotification');";
        html += "      if (data.isSleeping) {";
        html += "        notification.style.display = 'block';";
        html += "      } else {";
        html += "        notification.style.display = 'none';";
        html += "      }";
        html += "    });";
        html += "}";
        html += "setInterval(checkSleepStatus, 1000);";
        html += "</script>";
    
        // Retrieve real time data
        html += "<script>";
        html += "function updateMotorPower() {";
        html += "    const power = document.getElementById('motorPowerSlider').value;";
        html += "    document.getElementById('motorPowerValue').innerText = power;";
        html += "    sendRequest(`/setMotorPower?value=${power}`);";
        html += "    document.getElementById('updateStatus').innerText = 'Motor Power wordt geüpdatet...';";
        html += "}";
        html += "function moveForward() { sendCommand('moveForward'); }";
        html += "function moveBackward() { sendCommand('moveBackward'); }";
        html += "function moveLeft() { sendCommand('moveLeft'); }";
        html += "function moveRight() { sendCommand('moveRight'); }";
        html += "function rotateLeft() { sendCommand('rotateLeft'); }";
        html += "function rotateRight() { sendCommand('rotateRight'); }";
        html += "function stopMovement() { sendCommand('stop'); }";
        html += "function sendCommand(command) { sendRequest(`/${command}`); }";
        html += "function sendRequest(url) { const xhr = new XMLHttpRequest(); xhr.open('GET', url, true); xhr.send(); }";
    
        // Update pitch and roll
        html += "function updateMPUData() {";
        html += "  fetch('/getMPUData')";
        html += "    .then(response => response.json())";
        html += "    .then(data => {";
        html += "      document.getElementById('pitchValue').innerText = data.pitch.toFixed(2);";
        html += "      document.getElementById('rollValue').innerText = data.roll.toFixed(2);";
        html += "    })";
        html += "    .catch(error => console.error('Error fetching MPU data:', error));";
        html += "}";
    
        html += "function sendPID() {";
        html += "  const Kp = document.getElementById('KpInput').value;";
        html += "  const Ki = document.getElementById('KiInput').value;";
        html += "  const Kd = document.getElementById('KdInput').value;";
        html += "  const url = `/setPID?Kp=${Kp}&Ki=${Ki}&Kd=${Kd}`;";
        html += "  fetch(url)";
        html += "    .then(response => response.text())";
        html += "    .then(data => {";
        html += "      console.log('PID Updated:', data);";
        html += "      alert('PID values updated successfully');";
        html += "    })";
        html += "    .catch(error => {";
        html += "      console.error('Error updating PID:', error);";
        html += "      alert('Failed to update PID values');";
        html += "    });";
        html += "};";
    
        // Update every 100ms
        html += "setInterval(updateMPUData, 100);";
        html += "</script></body></html>";
    
        request->send(200, "text/html", html);
    });

    // Return sleep status
    server.on("/getSleepStatus", HTTP_GET, [](AsyncWebServerRequest *request){
        String json = "{\"isSleeping\": ";
        json += isSleeping ? "true" : "false";
        json += "}";
        request->send(200, "application/json", json);
    });
    
    // Endpoint retrieving MPU data
    server.on("/getMPUData", HTTP_GET, [](AsyncWebServerRequest *request){
        String json = "{\"pitch\": ";
        json += String(pitch);
        json += ", \"roll\": ";
        json += String(roll);
        json += "}";
    
        request->send(200, "application/json", json);
    });
    
    // Endpoint for updating motorpower
    server.on("/setMotorPower", HTTP_GET, [](AsyncWebServerRequest *request){
        if (request->hasParam("value")) {
            String motorPowerStr = request->getParam("value")->value();
            int newPower = motorPowerStr.toInt();
            
            // Wake up when power changes
            if (isSleeping && newPower > 0) {
                wakeupFromSleep();
            }
            
            motorPower = newPower;
            lastActivityTime = esp_timer_get_time() / 1000;
            Serial.println("Motor Power Set to: " + String(motorPower));
            thrust();
    
            // Send confirmation with motor power, pitch, and roll 
            String responseMessage = "{\"status\": \"Motor power updated\", \"pitch\": ";
            responseMessage += String(pitch);
            responseMessage += ", \"roll\": ";
            responseMessage += String(roll);
            responseMessage += "}";
    
            request->send(200, "application/json", responseMessage);
        }
    });

    server.on("/moveForward", HTTP_GET, [](AsyncWebServerRequest *request){ moveForward(); request->send(200, "text/plain", "Moving Up"); });
    server.on("/moveBackward", HTTP_GET, [](AsyncWebServerRequest *request){ moveBackward(); request->send(200, "text/plain", "Moving Down"); });
    server.on("/moveLeft", HTTP_GET, [](AsyncWebServerRequest *request){ moveLeft(); request->send(200, "text/plain", "Moving Left"); });
    server.on("/moveRight", HTTP_GET, [](AsyncWebServerRequest *request){ moveRight(); request->send(200, "text/plain", "Moving Right"); });
    server.on("/rotateLeft", HTTP_GET, [](AsyncWebServerRequest *request){ rotateLeft(); request->send(200, "text/plain", "Rotating Left"); });
    server.on("/rotateRight", HTTP_GET, [](AsyncWebServerRequest *request){ rotateRight(); request->send(200, "text/plain", "Rotating Right"); });
    server.on("/stop", HTTP_GET, [](AsyncWebServerRequest *request){ stopMovement(); request->send(200, "text/plain", "Stopping Movement"); });
    
    // Update PID based on input fields
    server.on("/setPID", HTTP_GET, [](AsyncWebServerRequest *request){
        if (request->hasParam("Kp")) {
        Kp = request->getParam("Kp")->value().toFloat();
        }
        if (request->hasParam("Ki")) {
        Ki = request->getParam("Ki")->value().toFloat();
        }
        if (request->hasParam("Kd")) {
        Kd = request->getParam("Kd")->value().toFloat();
        }
        String response = "Updated PID -> Kp: " + String(Kp) + ", Ki: " + String(Ki) + ", Kd: " + String(Kd);
        Serial.println(response);
        request->send(200, "text/plain", response);
    });
    
    Serial.println("Starting Web Server...");
    server.begin();
    Serial.println("Web Server Started!");
}

void initWatchdog() {
    // Check if already initialized
    esp_err_t wdt_status = esp_task_wdt_status(NULL);
    
    // Configure if not initialized
    if (wdt_status == ESP_ERR_NOT_FOUND) {
        esp_task_wdt_config_t wdt_config = {
            .timeout_ms = WDT_TIMEOUT_SEC * 1000,
            .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
            .trigger_panic = true,
        };
        
        esp_task_wdt_init(&wdt_config);
        ESP_LOGI(droneTag, "Watchdog initialized with %d second timeout", WDT_TIMEOUT_SEC);
    } else if (wdt_status == ESP_OK) {
        ESP_LOGI(droneTag, "Watchdog already initialized");
    } else {
        ESP_LOGE(droneTag, "Watchdog status error: %s", esp_err_to_name(wdt_status));
    }
    
    esp_err_t add_result = esp_task_wdt_add(NULL);
    if (add_result == ESP_OK) {
        ESP_LOGI(droneTag, "Current task added to watchdog.");
    } else if (add_result == ESP_ERR_INVALID_STATE) {
        ESP_LOGW(droneTag, "Current task already added to watchdog.");
    } else {
        ESP_LOGE(droneTag, "Failed to add current task to watchdog: %s", esp_err_to_name(add_result));
    }
}

void petWatchdog() {
    static uint64_t lastPetTime = 0;
    uint64_t currentTime = esp_timer_get_time() / 1000;
    
    // Pet when 500ms reached
    if (currentTime - lastPetTime >= PET_INTERVAL_MS) {
        esp_task_wdt_reset();
        lastPetTime = currentTime;
    }
}

void handle_emergency_stop() {
    ESP_LOGE(emergencyTag, "EMERGENCY STOP ACTIVATED!");
    
    // Cut power to motors
    for (int i = 0; i < 4; i++) {
        ledc_set_duty(LEDC_MODE, led_channels[i], 0);
        ledc_update_duty(LEDC_MODE, led_channels[i]);
    }

    enterSleepMode();
    
    // Send emergency notification
    udp.beginPacket(broadcastIP, remotePort);
    const char active[] = "EMERGENCY_ACTIVATED";
    udp.write((const uint8_t*)active, sizeof(active)-1);
    udp.endPacket();
    
    esp_restart();
}

// Software interrupt
void udp_listener_task(void *pvParameters) {
    // Add task to watchdog
    esp_task_wdt_add(NULL);
    
    // Start UDP
    Serial.println("Starting UDP listener");
    udp.begin(localPort);
    Serial.printf("UDP listener started on port %d\n", localPort);
    
    // Check for incoming packets
    while (1) {
        int packetSize = udp.parsePacket();
        if (packetSize) {
            char incomingPacket[50];
            int len = udp.read(incomingPacket, sizeof(incomingPacket));

            if (len > 0) {
                incomingPacket[len] = '\0';
                
                // Emergency stop triggered
                if (strstr(incomingPacket, "EMERGENCY_STOP")) {
                    portENTER_CRITICAL(&emergency_mux);
                    emergency_stop = true;
                    handle_emergency_stop();
                    portEXIT_CRITICAL(&emergency_mux);
                    
                    // Acknowledge
                    udp.beginPacket(udp.remoteIP(), udp.remotePort());
                    const char ack[] = "STOP_ACK";
                    udp.write((const uint8_t*)ack, sizeof(ack)-1);
                    udp.endPacket();
                }
            }
        }
        petWatchdog();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Send telemetry data
void telemetryBroadcastTask(void *pvParameters) {
    esp_task_wdt_add(NULL);
    
    while (1) {
        if (!isSleeping && !emergency_stop) {
            char telemetry[50];
            int len = snprintf(telemetry, sizeof(telemetry), "PITCH:%.1f,ROLL:%.1f", pitch, roll);
            
            // Send telemetry data
            udp.beginPacket(broadcastIP, remotePort);
            udp.write((const uint8_t*)telemetry, len);
            udp.endPacket();
        }
        petWatchdog();
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void wifiTask(void *pvParameters) {
    // Initialize WiFi
    WiFi.mode(WIFI_AP_STA);
    vTaskDelay(pdMS_TO_TICKS(100));
    WiFi.setAutoReconnect(true);
    WiFi.persistent(false);
    
    Serial.println("Connecting to WiFi...");
    WiFi.begin(SSID, PASSWORD);

    // AP for telemetry controller
    WiFi.softAP("drone", "drone123");
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nWiFi connection failed!");
        ESP.restart();
    }
    
    Serial.printf("\nAP IP: %s, STA IP: %s\n", WiFi.softAPIP().toString().c_str(), WiFi.localIP().toString().c_str());
    setupWebServer();

    // Check WiFi status every second
    while (1) {
        if (WiFi.status() != WL_CONNECTED) {
            WiFi.reconnect();
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void droneControlTask(void *pvParameters) {
    ESP_LOGI(droneTag, "Task started");
    setupMPU();
    ESP_LOGI(droneTag, "MPU initialized");
    setupMotors();
    ESP_LOGI(droneTag, "Motors initialized");

    lastActivityTime = esp_timer_get_time() / 1000;

    esp_task_wdt_add(NULL);
    
    // Timing control
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(33);
    static uint32_t max_loop_time = 0;

    while (1) {
        TickType_t loop_start = xTaskGetTickCount();
        petWatchdog();

        if (!isSleeping) {
            ESP_LOGI(droneTag, "Reading MPU");
            readMPU();
            ESP_LOGI(droneTag, "Updating PID");
            updatePID();
            ESP_LOGI(droneTag, "Stabilizing");
            stabilizeDrone();
        }
        
        checkSleepCondition();
        petWatchdog();

        // 50Hz timing
        xTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

extern "C" void app_main() {
    // Initialize watchdog
    initWatchdog();

    esp_timer_init();

    // Initialize nvs
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize Arduino
    initArduino();
    
    Serial.begin(115200);

    // Drone control task
    xTaskCreate(
        droneControlTask,
        "DroneControl",
        4096,
        NULL,
        2,
        NULL
    );

    // WiFi task
    xTaskCreate(
        wifiTask,
        "WiFiTask",
        4096,
        NULL,
        1,
        NULL
    );

    // UDP emergency listener task
    xTaskCreate(
        udp_listener_task,
        "udp_listener",
        4096,
        NULL,
        2,
        &udpTaskHandle
    );

    // UDP telemetry broadcast task
    xTaskCreate(
        telemetryBroadcastTask,
        "Telemetry",
        4096,
        NULL,
        1,
        NULL
    );

    // Petting watchdog
    while (true) {
        petWatchdog();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
