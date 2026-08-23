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

#include "../config/pin_config.h"
#include <stdbool.h>
#include <stdint.h>


/** @brief LED identifier enum. */
typedef enum {
  LED_STATUS = 0, /**< External status LED (PA5) */
  LED_DEBUG = 1   /**< Onboard debug LED (PC13) */
} LedID;

/** @brief Initialize LED pins as outputs and set both to OFF. */
void led_init(void);

/** @brief Set an LED on or off. Disables blink mode for that LED. */
void led_set(LedID led, bool on);

/** @brief Toggle an LED state. */
void led_toggle(LedID led);

/** @brief Start blinking an LED at the given interval (non-blocking).
 *  @param led LED identifier
 *  @param interval_ms Blink period in milliseconds (0 = disable blink)
 *  Uses millis()-based non-blocking toggle in led_update(). */
void led_blink(LedID led, uint16_t interval_ms);

/** @brief Update LED blink state. Call from main loop. */
void led_update(void);

#endif /* LED_H */
