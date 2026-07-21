#include <Wire.h>
#include "Adafruit_VL53L0X.h"

// Define XSHUT pins for each sensor (Adjust GPIO pins to match your STM32 wiring)
#define XSHUT_FRONT       PA0
#define XSHUT_FRONT_RIGHT PA1
#define XSHUT_FRONT_LEFT  PA2
#define XSHUT_RIGHT       PA3
#define XSHUT_LEFT        PA4

// Define unique I2C addresses (Base default is 0x29)
#define ADDR_FRONT       0x30
#define ADDR_FRONT_RIGHT 0x31
#define ADDR_FRONT_LEFT  0x32
#define ADDR_RIGHT       0x33
#define ADDR_LEFT        0x34

// Create VL53L0X sensor instances
Adafruit_VL53L0X sensorFront      = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorFrontRight = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorFrontLeft  = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorRight      = Adafruit_VL53L0X();
Adafruit_VL53L0X sensorLeft       = Adafruit_VL53L0X();

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(1); }

  Serial.println(F("--- Initializing 5x VL53L0X Sensors ---"));

  // Set I2C pins for STM32
  Wire.setSCL(PB8);
  Wire.setSDA(PB9);
  Wire.begin();

  // Configure XSHUT pins as outputs
  pinMode(XSHUT_FRONT, OUTPUT);
  pinMode(XSHUT_FRONT_RIGHT, OUTPUT);
  pinMode(XSHUT_FRONT_LEFT, OUTPUT);
  pinMode(XSHUT_RIGHT, OUTPUT);
  pinMode(XSHUT_LEFT, OUTPUT);

  // 1. Put ALL sensors into shutdown mode
  digitalWrite(XSHUT_FRONT, LOW);
  digitalWrite(XSHUT_FRONT_RIGHT, LOW);
  digitalWrite(XSHUT_FRONT_LEFT, LOW);
  digitalWrite(XSHUT_RIGHT, LOW);
  digitalWrite(XSHUT_LEFT, LOW);
  delay(10);

  // 2. Initialize each sensor one by one
  
  // --- FRONT SENSOR ---
  digitalWrite(XSHUT_FRONT, HIGH);
  delay(10);
  if (!sensorFront.begin(ADDR_FRONT)) {
    Serial.println(F("Failed to boot FRONT sensor!"));
    while (1);
  }

  // --- FRONT RIGHT SENSOR ---
  digitalWrite(XSHUT_FRONT_RIGHT, HIGH);
  delay(10);
  if (!sensorFrontRight.begin(ADDR_FRONT_RIGHT)) {
    Serial.println(F("Failed to boot FRONT RIGHT sensor!"));
    while (1);
  }

  // --- FRONT LEFT SENSOR ---
  digitalWrite(XSHUT_FRONT_LEFT, HIGH);
  delay(10);
  if (!sensorFrontLeft.begin(ADDR_FRONT_LEFT)) {
    Serial.println(F("Failed to boot FRONT LEFT sensor!"));
    while (1);
  }

  // --- RIGHT SENSOR ---
  digitalWrite(XSHUT_RIGHT, HIGH);
  delay(10);
  if (!sensorRight.begin(ADDR_RIGHT)) {
    Serial.println(F("Failed to boot RIGHT sensor!"));
    while (1);
  }

  // --- LEFT SENSOR ---
  digitalWrite(XSHUT_LEFT, HIGH);
  delay(10);
  if (!sensorLeft.begin(ADDR_LEFT)) {
    Serial.println(F("Failed to boot LEFT sensor!"));
    while (1);
  }

  Serial.println(F("All 5 sensors configured successfully!\n"));
}

// Helper function to read and print distance for a given sensor
void readSensor(Adafruit_VL53L0X &sensor, const char* name) {
  VL53L0X_RangingMeasurementData_t measure;
  sensor.rangingTest(&measure, false);

  Serial.print(name);
  Serial.print(F(": "));
  if (measure.RangeStatus != 4) {
    Serial.print(measure.RangeMilliMeter);
    Serial.print(F(" mm\t"));
  } else {
    Serial.print(F("Out of range\t"));
  }
}

void loop() {
  readSensor(sensorFront, "Front");
  readSensor(sensorFrontRight, "Front-R");
  readSensor(sensorFrontLeft, "Front-L");
  readSensor(sensorRight, "Right");
  readSensor(sensorLeft, "Left");

  Serial.println(); // Newline after reading all sensors
  delay(50);       // Brief delay between scan passes
}