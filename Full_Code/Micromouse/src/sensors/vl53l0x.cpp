/**
 * @file vl53l0x.cpp
 * @brief VL53L0X driver implementation.
 * @see vl53l0x.h
 */

#include "vl53l0x.h"
#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_VL53L0X.h"

#define MAX_TOF_SENSORS 5

static Adafruit_VL53L0X _adafruit_sensors[MAX_TOF_SENSORS];
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

        // begin() automatically changes the I2C address to the requested one
        if (_adafruit_sensors[i].begin(sensors[i].i2c_address, false, &Wire, Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT)) {
            sensors[i].initialized = true;
            success_count++;
        } else {
            sensors[i].initialized = false;
        }
    }

    return success_count;
}

void vl53l0x_start_measurement(const VL53L0X_Sensor *sensor) {
    // The Adafruit library rangingTest() function is blocking and automatically 
    // starts the measurement when called. No separate start trigger is needed 
    // unless using continuous mode.
    (void)sensor;
}

uint16_t vl53l0x_read_distance_mm(const VL53L0X_Sensor *sensor) {
    if (!sensor || !sensor->initialized) return 8190;

    // Find the internal Adafruit object mapped to this sensor's XSHUT pin
    int index = -1;
    for (uint8_t i = 0; i < _active_sensor_count; i++) {
        if (_mapped_xshut[i] == sensor->xshut_pin) {
            index = i;
            break;
        }
    }

    if (index == -1) return 8190;

    VL53L0X_RangingMeasurementData_t measure;
    _adafruit_sensors[index].rangingTest(&measure, false);

    // RangeStatus == 4 means out of range
    if (measure.RangeStatus != 4) {
        return measure.RangeMilliMeter;
    } else {
        return 8190;
    }
}
