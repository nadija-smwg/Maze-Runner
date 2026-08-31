/**
 * @file vl53l0x.cpp
 * @brief VL53L0X driver implementation.
 * @see vl53l0x.h
 *
 * Key improvements over the original single-shot implementation:
 *
 *  1. Timing budget set to 33 ms per measurement — better accuracy/noise
 *     tradeoff vs the library default (~20 ms).
 *     Tune: lower = faster but noisier; higher = slower but more stable.
 *     Recommended range for micromouse: 20–40 ms.
 *
 *  2. Continuous ranging mode (startRangeContinuous) — sensor measures
 *     back-to-back without needing a fresh trigger each time. Readings
 *     are always fresh when rangingTest() is called.
 *
 *  3. RangeStatus checked: status 4 = phase fail / out of range → 8190.
 *     Any other non-zero status is also treated as invalid and returns
 *     8190 so the filter pipeline's validity stage rejects it.
 */

#include "vl53l0x.h"
#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_VL53L0X.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  Configuration Constants
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * Measurement timing budget in microseconds.
 * Controls the time each ranging cycle has available — more time = lower noise.
 *
 *   20 000 µs (20 ms) → fastest,  noisiest (~±5 mm typical)
 *   33 000 µs (33 ms) → balanced, moderate noise (~±3 mm typical)
 *   50 000 µs (50 ms) → slowest,  quietest  (~±2 mm typical)
 *
 * For a micromouse running at 100 Hz sensor update rate with 5 sensors
 * all sharing one I2C bus, 33 ms is a reasonable starting point.
 * TODO: Lower to 20 ms if sensor update rate needs to be higher.
 */
#define TOF_TIMING_BUDGET_US    33000UL

/** Maximum number of sensors this driver manages. */
#define MAX_TOF_SENSORS         5

/* ═══════════════════════════════════════════════════════════════════════════
 *  Private State
 * ═══════════════════════════════════════════════════════════════════════════ */

/** Adafruit library objects, one per sensor. */
static Adafruit_VL53L0X _adafruit_sensors[MAX_TOF_SENSORS];

/** XSHUT pin for each active sensor (used to look up by sensor struct). */
static uint8_t _mapped_xshut[MAX_TOF_SENSORS] = {0};

/** Number of sensors successfully initialized. */
static uint8_t _active_sensor_count = 0;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Initialization
 * ═══════════════════════════════════════════════════════════════════════════ */

uint8_t vl53l0x_init_all(VL53L0X_Sensor *sensors, uint8_t count)
{
    if (count > MAX_TOF_SENSORS)
        count = MAX_TOF_SENSORS;

    _active_sensor_count = count;
    uint8_t success_count = 0;

    /* 1. Pull all XSHUT pins LOW → all sensors held in reset */
    for (uint8_t i = 0; i < count; i++)
    {
        pinMode(sensors[i].xshut_pin, OUTPUT);
        digitalWrite(sensors[i].xshut_pin, LOW);
        _mapped_xshut[i] = sensors[i].xshut_pin;
    }
    delay(10); /* Allow sensors to fully reset */

    /* 2. Wake up and configure each sensor one at a time */
    for (uint8_t i = 0; i < count; i++)
    {
        /* Bring this sensor out of reset — it boots at default address 0x29 */
        digitalWrite(sensors[i].xshut_pin, HIGH);
        delay(10); /* Boot time for VL53L0X at default address */

        /*
         * begin() assigns the sensor its unique I2C address and performs
         * device identification. VL53L0X_SENSE_DEFAULT = standard ranging mode.
         */
        if (!_adafruit_sensors[i].begin(sensors[i].i2c_address,
                                        false,
                                        &Wire,
                                        Adafruit_VL53L0X::VL53L0X_SENSE_DEFAULT))
        {
            Serial.print("[TOF] Sensor ");
            Serial.print(i);
            Serial.print(" FAILED (addr 0x");
            Serial.print(sensors[i].i2c_address, HEX);
            Serial.println(")");
            sensors[i].initialized = false;
            continue;
        }

        /* 3. Set timing budget for better noise performance */
        _adafruit_sensors[i].setMeasurementTimingBudgetMicroSeconds(TOF_TIMING_BUDGET_US);

        /* 4. Start continuous ranging — sensor measures back-to-back */
        _adafruit_sensors[i].startRangeContinuous();

        sensors[i].initialized = true;
        success_count++;

        Serial.print("[TOF] Sensor ");
        Serial.print(i);
        Serial.print(" OK  addr=0x");
        Serial.print(sensors[i].i2c_address, HEX);
        Serial.print("  budget=");
        Serial.print(TOF_TIMING_BUDGET_US / 1000UL);
        Serial.println("ms  mode=continuous");
    }

    return success_count;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Measurement trigger (no-op in continuous mode)
 * ═══════════════════════════════════════════════════════════════════════════ */

void vl53l0x_start_measurement(const VL53L0X_Sensor *sensor)
{
    /*
     * In continuous ranging mode the sensor measures autonomously.
     * No explicit trigger is needed. This function is kept for API
     * compatibility but does nothing.
     */
    (void)sensor;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Distance Read
 * ═══════════════════════════════════════════════════════════════════════════ */

uint16_t vl53l0x_read_distance_mm(const VL53L0X_Sensor *sensor)
{
    if (!sensor || !sensor->initialized)
        return 8190u;

    /* Find the Adafruit object that corresponds to this sensor's XSHUT pin */
    int index = -1;
    for (uint8_t i = 0; i < _active_sensor_count; i++)
    {
        if (_mapped_xshut[i] == sensor->xshut_pin)
        {
            index = (int)i;
            break;
        }
    }
    if (index < 0)
        return 8190u;

    VL53L0X_RangingMeasurementData_t measure;
    _adafruit_sensors[index].rangingTest(&measure, false /* no debug print */);

    /*
     * RangeStatus meaning (ST API):
     *   0 = Range valid
     *   2 = Signal failure (weak return)
     *   3 = Min range fail
     *   4 = Phase failure / out of range → library returns 8190
     *
     * We treat status 0 as valid. Any other status → return 8190 so the
     * tof_filter_process() validity stage rejects the reading and holds
     * the last good filtered value.
     */
    if (measure.RangeStatus == 0)
        return measure.RangeMilliMeter;

    return 8190u;
}
