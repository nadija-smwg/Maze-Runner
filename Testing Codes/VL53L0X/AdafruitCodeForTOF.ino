#include <Wire.h>
#include "Adafruit_VL53L0X.h"

// Create an instance of the sensor object
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
  // Initialize serial communication for the Serial Monitor
  Serial.begin(115200);

  // Wait for serial port to open (needed for native USB boards)
  while (!Serial) {
    delay(1);
  }
  
  Serial.println("VL53L0X ToF Sensor Custom Pin Test");

  // Explicitly set the I2C pins for STM32 before starting the sensor
  // Connect SCL to PB6 and SDA to PB7
  Wire.setSCL(PB8);
  Wire.setSDA(PB9);
  
  // Initialize the sensor over the configured I2C port
  if (!lox.begin()) {
    Serial.println(F("Failed to boot VL53L0X! Check your wiring connections."));
    while(1); // Halt execution if sensor is not detected
  }
  
  Serial.println(F("VL53L0X initialized successfully on PB6/PB7."));
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;
    
  // Request a distance measurement from the sensor
  lox.rangingTest(&measure, false); 

  // Check if the measurement is valid (Phase Status 0 means valid measurement)
  if (measure.RangeStatus != 4) {  
    Serial.print("Distance (mm): ");
    Serial.println(measure.RangeMilliMeter);
  } else {
    Serial.println(" Out of range / Signal weak ");
  }
    
  // Wait 100ms before taking the next reading
  delay(100);
}
}