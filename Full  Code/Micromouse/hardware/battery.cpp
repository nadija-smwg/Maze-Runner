/**
 * @file battery.cpp
 * @brief Battery monitoring implementation.
 *
 * @see battery.h for public API documentation
 */

#include "battery.h"
#include <Arduino.h>

/* ═══════════════════════════════════════════════════════════════════════════
 *  Private State
 * ═══════════════════════════════════════════════════════════════════════════ */

static uint16_t _smoothed_voltage_mv = 0;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Implementation
 * ═══════════════════════════════════════════════════════════════════════════ */

void battery_init(void) {
    /**
     * TODO: Configure PIN_BATTERY_SENSE as analog input.
     * TODO: Take initial reading to seed the smoothing filter.
     */
}

uint16_t battery_get_voltage_mv(void) {
    /**
     * TODO: Read analog value from PIN_BATTERY_SENSE.
     * TODO: Convert to millivolts:
     *       raw_mv = analogRead(PIN_BATTERY_SENSE) * 3300 / 4095;
     *       battery_mv = raw_mv * BATTERY_DIVIDER_RATIO;
     * TODO: Apply exponential moving average for smoothing.
     */
    return 0;
}

uint8_t battery_get_percentage(void) {
    /**
     * TODO: Map voltage from [BATTERY_CRITICAL_MV, BATTERY_FULL_MV] to [0, 100].
     */
    return 0;
}

bool battery_is_low(void) {
    /**
     * TODO: return battery_get_voltage_mv() < BATTERY_LOW_MV;
     */
    return false;
}

bool battery_is_critical(void) {
    /**
     * TODO: return battery_get_voltage_mv() < BATTERY_CRITICAL_MV;
     */
    return false;
}
