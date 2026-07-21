/**
 * @file vl53l0x.h
 * @brief I2C driver for multiple VL53L0X Time-of-Flight sensors.
 *
 * Micromouse usually uses 5 ToF sensors (Front, Front-Left, Front-Right,
 * Left, Right) sharing a single I2C bus. The XSHUT pin is used to assign
 * unique I2C addresses during initialization.
 *
 * Dependencies: pin_config
 */

#ifndef VL53L0X_H
#define VL53L0X_H

#include <stdint.h>
#include <stdbool.h>
#include "../config/pin_config.h"

/**
 * @brief VL53L0X sensor object struct.
 */
typedef struct {
    uint8_t xshut_pin;      /**< GPIO pin connected to XSHUT */
    uint8_t i2c_address;    /**< Target I2C address for this sensor */
    bool    initialized;    /**< Initialization success flag */
} VL53L0X_Sensor;

/**
 * @brief Initialize all VL53L0X sensors on the bus.
 *
 * This performs the address assignment sequence:
 * 1. Pull all XSHUT pins LOW (sensors in reset).
 * 2. Pull one XSHUT pin HIGH (wake up one sensor at default 0x29).
 * 3. Send I2C command to change its address to the new assigned address.
 * 4. Repeat for the remaining sensors.
 *
 * @param sensors Array of sensor configurations (modified in place)
 * @param count   Number of sensors in the array
 * @return        Number of sensors successfully initialized
 *
 * TODO: Implement initialization sequence and configuration (e.g., high speed mode).
 */
uint8_t vl53l0x_init_all(VL53L0X_Sensor *sensors, uint8_t count);

/**
 * @brief Start a continuous or single distance measurement.
 *
 * @param sensor Pointer to initialized sensor struct
 *
 * TODO: Implement measurement trigger via I2C commands.
 */
void vl53l0x_start_measurement(const VL53L0X_Sensor *sensor);

/**
 * @brief Read the measured distance in millimeters.
 *
 * Checks if data is ready, and if so, reads the distance register.
 *
 * @param sensor Pointer to initialized sensor struct
 * @return       Distance in mm, or 8190 if out of range, or 0 on error
 *
 * TODO: Implement data-ready check and result read via I2C.
 */
uint16_t vl53l0x_read_distance_mm(const VL53L0X_Sensor *sensor);

#endif /* VL53L0X_H */
