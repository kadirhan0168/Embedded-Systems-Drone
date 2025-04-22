#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define BUTTON_PIN 18
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Drone AP credentials
const char* ssid = "drone";
const char* password = "drone123";
WiFiUDP udp;

// Drone IP & UDP port
IPAddress droneIP(192, 168, 4, 1);
const int udpPort = 1234;

volatile bool emergencyTriggered = false;
unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 200;

// Hardware interrupt
void IRAM_ATTR buttonISR() {
  if ((millis() - lastDebounceTime) > debounceDelay) {
    emergencyTriggered = true;
    lastDebounceTime = millis();
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);
  
  lcd.begin(16, 2);
  lcd.backlight();
  lcd.print("Initializing...");
  
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonISR, FALLING);
  
  // Connect to drone AP
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    lcd.setCursor(0, 1);
    lcd.print("Connecting...");
  }
  
  lcd.clear();
  lcd.print("Connected!");
  lcd.setCursor(0, 1);
  lcd.print(WiFi.localIP());
  
  udp.begin(udpPort);
}

// Sending emergency packet to drone
void sendEmergencyPacket() {
  char message[] = "EMERGENCY_STOP";
  udp.beginPacket(droneIP, udpPort);
  udp.write((uint8_t*)message, sizeof(message));
  udp.endPacket();
}

void updateDisplay(float pitch, float roll) {
  char pitchBuffer[8];
  char rollBuffer[8];

  // Format pitch and roll
  snprintf(pitchBuffer, sizeof(pitchBuffer), "%+06.2f", pitch);
  snprintf(rollBuffer, sizeof(rollBuffer), "%+06.2f", roll);

  // Print pitch data
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Pitch:");
  lcd.setCursor(9, 0);
  lcd.print(pitchBuffer);
  lcd.write(0xDF);

  // Print roll data
  lcd.setCursor(0, 1);
  lcd.print("Roll: ");
  lcd.setCursor(9, 1);
  lcd.print(rollBuffer);
  lcd.write(0xDF);
  delay(200);
}

void stopTriggerUpdate() {
  lcd.clear();
  lcd.print("Emergency Stop");
  lcd.setCursor(0, 1);
  lcd.print("Triggered!");
}

void loop() {
  if (emergencyTriggered) {
    lcd.setCursor(0, 0);
    sendEmergencyPacket();
    stopTriggerUpdate();
    
    while (digitalRead(BUTTON_PIN) == LOW) {
      delay(10);
    }
    
    emergencyTriggered = false;
  }
  
  // Check for telemetry data
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char incomingPacket[50];
    int len = udp.read(incomingPacket, sizeof(incomingPacket));
    if (len > 0) {
      incomingPacket[len] = '\0';
      
      // Parse telemetry data
      char* pitchStr = strstr(incomingPacket, "PITCH:");
      char* rollStr = strstr(incomingPacket, "ROLL:");
      
      // Extract data and convert to float
      if (pitchStr && rollStr) {
        float pitch = atof(pitchStr + 6);
        float roll = atof(rollStr + 5);
        updateDisplay(pitch, roll);
      }
    }
  }
  
  delay(10);
}