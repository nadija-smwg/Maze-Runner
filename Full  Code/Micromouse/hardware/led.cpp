/**
 * @file led.cpp
 * @brief LED implementation.
 * @see led.h
 */

#include "led.h"
#include <Arduino.h>

void led_init(void)                  { /** TODO: pinMode for LED pins. */ }
void led_set(LedID led, bool on)     { /** TODO: digitalWrite(pin, on). Note: PC13 is active-LOW on Black Pill. */ }
void led_toggle(LedID led)           { /** TODO: digitalRead then write inverse. */ }
void led_blink(LedID led, uint16_t interval_ms) { /** TODO: Store blink state and interval. */ }
void led_update(void)                { /** TODO: Check millis() and toggle if interval elapsed. */ }
