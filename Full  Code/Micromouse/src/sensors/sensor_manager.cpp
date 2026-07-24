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

void sensor_manager_init(void) {
    /**
     * TODO:
     * mpu6050_init();
     * distance_manager_init();
     * battery_init();
     * encoder_init();
     */
}

void sensor_manager_update(void) {
    /**
     * TODO:
     * distance_manager_update();
     * // MPU6050 might be updated directly in the fast control loop (sensor_fusion)
     * // instead of here, depending on timing requirements.
     */
}

void sensor_manager_debug_print(void) {
    /** TODO: Print distance, IMU, battery, encoder data to Serial */
}
