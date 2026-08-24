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
    pinMode(PIN_BATTERY_SENSE, INPUT_ANALOG);
    analogReadResolution(12); // Ensure 12-bit ADC resolution (0 - 4095)

    // Take an initial raw reading to seed the smoothing filter without lag
    uint32_t raw_adc = analogRead(PIN_BATTERY_SENSE);
    float raw_mv = (raw_adc * 3300.0f) / 4095.0f;
    _smoothed_voltage_mv = (uint16_t)(raw_mv * BATTERY_DIVIDER_RATIO);
}

uint16_t battery_get_voltage_mv(void) {
    // 1. Read 12-bit ADC value (0-4095) from the voltage divider sense pin
    uint32_t raw_adc = analogRead(PIN_BATTERY_SENSE);

    // 2. Convert ADC counts to pin voltage (assuming 3.3V reference = 3300 mV)
    float pin_mv = (raw_adc * 3300.0f) / 4095.0f;

    // 3. Multiply by the voltage divider ratio (V_battery = V_pin * ratio)
    float current_mv = pin_mv * BATTERY_DIVIDER_RATIO;

    // 4. Exponential moving average (EMA) filter: 80% previous + 20% new sample
    if (_smoothed_voltage_mv == 0) {
        _smoothed_voltage_mv = (uint16_t)current_mv;
    } else {
        _smoothed_voltage_mv = (uint16_t)(0.8f * _smoothed_voltage_mv + 0.2f * current_mv);
    }

    return _smoothed_voltage_mv;
}

uint8_t battery_get_percentage(void) {
    uint16_t mv = battery_get_voltage_mv();
    if (mv <= BATTERY_CRITICAL_MV) return 0;
    if (mv >= BATTERY_FULL_MV) return 100;

    // Linearly map [BATTERY_CRITICAL_MV, BATTERY_FULL_MV] to [0, 100]%
    return (uint8_t)(((uint32_t)(mv - BATTERY_CRITICAL_MV) * 100) / (BATTERY_FULL_MV - BATTERY_CRITICAL_MV));
}

bool battery_is_low(void) {
    return battery_get_voltage_mv() < BATTERY_LOW_MV;
}

bool battery_is_critical(void) {
    return battery_get_voltage_mv() < BATTERY_CRITICAL_MV;
}
