/**
 * @file vl53l0x.cpp
 * @brief VL53L0X driver implementation.
 * @see vl53l0x.h
 *
 * Note: Wrap the Adafruit_VL53L0X library inside this implementation
 * file, or write the I2C register access directly.
 */

#include "vl53l0x.h"
#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_VL53L0X.h"

/* TODO: Implement sensor array using Adafruit_VL53L0X instances. */

uint8_t vl53l0x_init_all(VL53L0X_Sensor *sensors, uint8_t count) {
    /**
     * TODO:
     * 1. Set all xshut_pin as OUTPUT and LOW (reset all sensors).
     *    delay(10);
     * 2. Loop through sensors:
     *    a. Set sensors[i].xshut_pin HIGH (wake up sensor i).
     *    b. delay(10);
     *    c. Initialize sensor at default address (0x29).
     *    d. Change I2C address to sensors[i].i2c_address.
     *    e. Configure sensor settings (timing budget, continuous mode, etc).
     *    f. Set sensors[i].initialized = true if successful.
     */
    return 0; // return number of successful inits
}

void vl53l0x_start_measurement(const VL53L0X_Sensor *sensor) {
    /**
     * TODO: Send command to start single-shot or continuous measurement
     * on the specific I2C address.
     */
}

uint16_t vl53l0x_read_distance_mm(const VL53L0X_Sensor *sensor) {
    /**
     * TODO:
     * 1. Check if measurement is ready (interrupt status register).
     * 2. If ready, read result registers (2 bytes).
     * 3. Clear interrupt.
     * 4. Return value in mm.
     */
    return 8190;
}
