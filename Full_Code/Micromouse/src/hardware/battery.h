/**
 * @file battery.h
 * @brief 2S LiPo battery voltage monitoring via ADC.
 *
 * Reads battery voltage through a voltage divider connected to an ADC pin.
 * Provides voltage level, percentage estimation, and low-battery warnings.
 *
 * Dependencies: pin_config, robot_config
 */

#ifndef BATTERY_H
#define BATTERY_H

#include "../config/pin_config.h"
#include "../config/robot_config.h"
#include <stdbool.h>
#include <stdint.h>


/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API
 * ═══════════════════════════════════════════════════════════════════════════
 */

/**
 * @brief Initialize the battery monitoring ADC.
 *
 * Configures PIN_BATTERY_SENSE as analog input with 12-bit resolution
 * and seeds the EMA smoothing filter with an initial reading.
 */
void battery_init(void);

/**
 * @brief Read the current battery voltage in millivolts.
 *
 * @return Battery voltage (mV)
 *
 * Reads 12-bit ADC, applies voltage divider ratio, and smooths
 * with an exponential moving average filter (80/20 blend).
 */
uint16_t battery_get_voltage_mv(void);

/**
 * @brief Get estimated battery percentage.
 *
 * @return Battery percentage (0–100)
 *
 * Linearly maps voltage between BATTERY_CRITICAL_MV and BATTERY_FULL_MV.
 */
uint8_t battery_get_percentage(void);

/**
 * @brief Check if battery is low (below warning threshold).
 *
 * @return true if voltage < BATTERY_LOW_MV
 *
 * Compares smoothed voltage against BATTERY_LOW_MV threshold.
 */
bool battery_is_low(void);

/**
 * @brief Check if battery is critically low (must stop robot).
 *
 * @return true if voltage < BATTERY_CRITICAL_MV
 *
 * Compares smoothed voltage against BATTERY_CRITICAL_MV threshold.
 */
bool battery_is_critical(void);

#endif /* BATTERY_H */
