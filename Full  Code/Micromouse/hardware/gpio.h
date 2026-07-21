/**
 * @file gpio.h
 * @brief GPIO abstraction for motor direction pin control.
 *
 * Provides a clean interface for motor direction pins, isolating
 * the TB6612FNG pin logic from the motor module.
 *
 * Dependencies: pin_config
 */

#ifndef GPIO_H
#define GPIO_H

#include <Arduino.h>
#include <stdint.h>
#include <stdbool.h>
#include "../config/pin_config.h"

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Initialize all motor-related GPIO pins as outputs.
 *
 * Configures AIN1, AIN2, BIN1, BIN2, STBY as digital outputs.
 *
 * TODO: Implement using pinMode() for each direction pin.
 */
void gpio_init_motor_pins(void);

/**
 * @brief Set the STBY (standby) pin state.
 *
 * @param enable true = motors enabled, false = standby (all off)
 *
 * TODO: Implement using digitalWrite(PIN_MOTOR_STBY, enable ? HIGH : LOW).
 */
void gpio_set_standby(bool enable);

/**
 * @brief Set the left motor direction pins.
 *
 * @param in1 State of AIN1 (HIGH/LOW)
 * @param in2 State of AIN2 (HIGH/LOW)
 *
 * TODO: Implement using digitalWrite() for AIN1 and AIN2.
 */
void gpio_set_left_direction(uint8_t in1, uint8_t in2);

/**
 * @brief Set the right motor direction pins.
 *
 * @param in1 State of BIN1 (HIGH/LOW)
 * @param in2 State of BIN2 (HIGH/LOW)
 *
 * TODO: Implement using digitalWrite() for BIN1 and BIN2.
 */
void gpio_set_right_direction(uint8_t in1, uint8_t in2);

#endif /* GPIO_H */
