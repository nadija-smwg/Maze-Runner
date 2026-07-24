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

#include <stdint.h>
#include <stdbool.h>
#include "../config/pin_config.h"
#include "../config/robot_config.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Initialize the battery monitoring ADC.
 *
 * TODO: Configure the battery sense pin as analog input.
 * TODO: Initialize ADC if not already done by analogRead().
 */
void battery_init(void);

/**
 * @brief Read the current battery voltage in millivolts.
 *
 * @return Battery voltage (mV)
 *
 * TODO: Read ADC value from PIN_BATTERY_SENSE.
 * TODO: Convert ADC value to voltage using BATTERY_DIVIDER_RATIO.
 * TODO: Apply smoothing filter to reduce noise.
 */
uint16_t battery_get_voltage_mv(void);

/**
 * @brief Get estimated battery percentage.
 *
 * @return Battery percentage (0–100)
 *
 * TODO: Map voltage between BATTERY_CRITICAL_MV and BATTERY_FULL_MV.
 */
uint8_t battery_get_percentage(void);

/**
 * @brief Check if battery is low (below warning threshold).
 *
 * @return true if voltage < BATTERY_LOW_MV
 *
 * TODO: Implement threshold check.
 */
bool battery_is_low(void);

/**
 * @brief Check if battery is critically low (must stop robot).
 *
 * @return true if voltage < BATTERY_CRITICAL_MV
 *
 * TODO: Implement threshold check.
 */
bool battery_is_critical(void);

#endif /* BATTERY_H */
