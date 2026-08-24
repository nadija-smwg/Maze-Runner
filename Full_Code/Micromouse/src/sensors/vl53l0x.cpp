/**
 * @file vl53l0x.cpp
 * @brief VL53L0X driver implementation.
 * @see vl53l0x.h
 */

#include "vl53l0x.h"
#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>

#define MAX_TOF_SENSORS 5

static VL53L0X _pololu_sensors[MAX_TOF_SENSORS];
static uint8_t _mapped_xshut[MAX_TOF_SENSORS] = {0};
static uint8_t _active_sensor_count = 0;

uint8_t vl53l0x_init_all(VL53L0X_Sensor *sensors, uint8_t count) {
    if (count > MAX_TOF_SENSORS) count = MAX_TOF_SENSORS;
    _active_sensor_count = count;
    uint8_t success_count = 0;

    // 1. Set all xshut pins as OUTPUT and pull LOW to reset all sensors
    for (uint8_t i = 0; i < count; i++) {
        pinMode(sensors[i].xshut_pin, OUTPUT);
        digitalWrite(sensors[i].xshut_pin, LOW);
        _mapped_xshut[i] = sensors[i].xshut_pin; // Store mapping
    }
    delay(10); // Wait for sensors to reset

    // 2. Wake up and initialize sensors one by one
    for (uint8_t i = 0; i < count; i++) {
        digitalWrite(sensors[i].xshut_pin, HIGH);
        delay(10); // Wait for sensor to boot up at default address (0x29)

        _pololu_sensors[i].setTimeout(500);
        if (_pololu_sensors[i].init()) {
            _pololu_sensors[i].setAddress(sensors[i].i2c_address);
            _pololu_sensors[i].startContinuous();
            sensors[i].initialized = true;
            success_count++;
        } else {
            sensors[i].initialized = false;
        }
    }

    return success_count;
}

void vl53l0x_start_measurement(const VL53L0X_Sensor *sensor) {
    // Pololu library is in continuous mode, no need to trigger manually.
    (void)sensor;
}

uint16_t vl53l0x_read_distance_mm(const VL53L0X_Sensor *sensor) {
    if (!sensor || !sensor->initialized) return 8190;

    int index = -1;
    for (uint8_t i = 0; i < _active_sensor_count; i++) {
        if (_mapped_xshut[i] == sensor->xshut_pin) {
            index = i;
            break;
        }
    }

    if (index == -1) return 8190;

    uint16_t dist = _pololu_sensors[index].readRangeContinuousMillimeters();
    if (_pololu_sensors[index].timeoutOccurred()) {
        return 8190;
    }
    return dist;
}
