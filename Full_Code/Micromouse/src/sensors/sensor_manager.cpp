/**
 * @file sensor_manager.cpp
 * @brief Sensor manager implementation.
 * @see sensor_manager.h
 */

#include "sensor_manager.h"
#include "distance_manager.h"
#include "mpu6050.h"
#include "../hardware/battery.h"
#include "../hardware/encoder.h"
#include <Arduino.h>

void sensor_manager_init(void) {
    battery_init();
    encoder_init();

    if (mpu6050_init()) {
        Serial.println("[Sensor] MPU6050 Initialized.");
    } else {
        Serial.println("[Sensor] ERROR: MPU6050 failed to init.");
    }

    distance_manager_init();
}

void sensor_manager_update(void) {
    distance_manager_update();
    // MPU6050 raw update can be done here, or in sensor_fusion loop
}

void sensor_manager_debug_print(void) {
    Serial.print("[Sensor] F:");
    Serial.print(distance_get_mm(TOF_FRONT));
    Serial.print(" FL:");
    Serial.print(distance_get_mm(TOF_FRONT_LEFT));
    Serial.print(" FR:");
    Serial.print(distance_get_mm(TOF_FRONT_RIGHT));
    Serial.print(" L:");
    Serial.print(distance_get_mm(TOF_LEFT));
    Serial.print(" R:");
    Serial.print(distance_get_mm(TOF_RIGHT));

    IMUScaledData imu;
    mpu6050_read_scaled(&imu);
    Serial.print(" | GyroZ: ");
    Serial.print(imu.gyro_z_dps, 1);
    
    Serial.print(" | Bat: ");
    Serial.print(battery_get_voltage_mv());
    Serial.println("mV");
}
