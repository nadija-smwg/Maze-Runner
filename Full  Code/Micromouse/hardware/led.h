/**
 * @file led.h
 * @brief LED indicator control.
 *
 * Controls onboard and external LEDs for status indication.
 *
 * Dependencies: pin_config
 */

#ifndef LED_H
#define LED_H

#include <stdbool.h>
#include "../config/pin_config.h"

/** @brief LED identifier enum. */
typedef enum {
    LED_STATUS = 0, /**< Onboard LED (PC13 on Black Pill) */
    LED_DEBUG  = 1  /**< External debug LED */
} LedID;

/** @brief Initialize LED pins as outputs. TODO: Implement. */
void led_init(void);

/** @brief Set an LED on or off. TODO: Implement. */
void led_set(LedID led, bool on);

/** @brief Toggle an LED. TODO: Implement. */
void led_toggle(LedID led);

/** @brief Start blinking an LED at the given interval (non-blocking).
 *  @param led LED identifier
 *  @param interval_ms Blink period in milliseconds
 *  TODO: Implement using millis()-based non-blocking toggle. */
void led_blink(LedID led, uint16_t interval_ms);

/** @brief Update LED blink state. Call from main loop. TODO: Implement. */
void led_update(void);

#endif /* LED_H */
