/**
 * @file calibration.cpp
 * @brief Calibration implementation.
 * @see calibration.h
 */

#include "calibration.h"
#include "../display/oled_driver.h"
#include "mpu6050.h"
#include <Arduino.h>


bool calibrate_all(void) {
  oled_clear();
  oled_print(0, 0, "Calibrating IMU");
  oled_print(0, 15, "Hands off!");
  oled_update();

  // Wait 5 seconds for the user to place the robot on the floor and remove
  // their hand
  delay(10000);

  oled_print(0, 30, "Calibrating...");
  oled_update();

  calibrate_gyro();

  oled_clear();
  oled_print(0, 0, "Calibration");
  oled_print(0, 15, "Complete!");
  oled_update();
  delay(1000);

  return true;
}

void calibrate_gyro(void) {
  // Collect 1000 samples for gyro calibration (~2 seconds)
  mpu6050_calibrate_gyro(1000);
}

void calibrate_distance_sensors(void) {
  // Future implementation
}

void calibrate_encoders(void) {
  // Future implementation
}
