/**
 * @file led.cpp
 * @brief LED implementation.
 * @see led.h
 */

#include "led.h"
#include <Arduino.h>

// Internal state tracking for non-blocking blink and logical state
static bool _blink_enabled[2]  = { false, false };
static uint16_t _blink_interval[2] = { 0, 0 };
static uint32_t _last_toggle_ms[2] = { 0, 0 };
static bool _led_state[2]      = { false, false };

void led_init(void) {
    pinMode(PIN_LED_STATUS, OUTPUT);
    pinMode(PIN_LED_DEBUG, OUTPUT);

    // Initialize both LEDs to OFF
    led_set(LED_STATUS, false);
    led_set(LED_DEBUG, false);
}

void led_set(LedID led, bool on) {
    if (led > LED_DEBUG) return;
    
    // Explicitly setting state disables blinking mode
    _blink_enabled[led] = false;
    _led_state[led] = on;

    if (led == LED_STATUS) {
        // Black Pill PC13 is active-LOW (LOW = ON, HIGH = OFF)
        digitalWrite(PIN_LED_STATUS, on ? LOW : HIGH);
    } else {
        // External debug LED is active-HIGH (HIGH = ON, LOW = OFF)
        digitalWrite(PIN_LED_DEBUG, on ? HIGH : LOW);
    }
}

void led_toggle(LedID led) {
    if (led > LED_DEBUG) return;
    bool new_state = !_led_state[led];
    _led_state[led] = new_state;

    if (led == LED_STATUS) {
        digitalWrite(PIN_LED_STATUS, new_state ? LOW : HIGH);
    } else {
        digitalWrite(PIN_LED_DEBUG, new_state ? HIGH : LOW);
    }
}

void led_blink(LedID led, uint16_t interval_ms) {
    if (led > LED_DEBUG) return;
    if (interval_ms == 0) {
        led_set(led, false);
        return;
    }
    _blink_enabled[led] = true;
    _blink_interval[led] = interval_ms;
    _last_toggle_ms[led] = millis();
}

void led_update(void) {
    uint32_t now = millis();
    for (int i = 0; i < 2; i++) {
        if (_blink_enabled[i]) {
            if ((now - _last_toggle_ms[i]) >= _blink_interval[i]) {
                _last_toggle_ms[i] = now;
                // Toggle without disabling blink mode
                bool new_state = !_led_state[i];
                _led_state[i] = new_state;
                if (i == LED_STATUS) {
                    digitalWrite(PIN_LED_STATUS, new_state ? LOW : HIGH);
                } else {
                    digitalWrite(PIN_LED_DEBUG, new_state ? HIGH : LOW);
                }
            }
        }
    }
}
