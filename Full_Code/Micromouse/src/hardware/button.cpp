/**
 * @file button.cpp
 * @brief Button debounce implementation.
 * @see button.h
 */

#include "button.h"
#include <Arduino.h>

static bool _pressed[2]      = {false, false};
static bool _just_pressed[2] = {false, false};
static bool _raw_state[2]    = {false, false};
static uint32_t _last_change_ms[2] = {0, 0};
static const uint32_t DEBOUNCE_DELAY_MS = 20;

void button_init(void) {
    // Configure buttons with internal pull-up resistors (unpressed = HIGH, pressed = LOW)
    pinMode(PIN_BUTTON_START, INPUT_PULLUP);
    pinMode(PIN_BUTTON_MODE, INPUT_PULLUP);
}

void button_update(void) {
    uint32_t now = millis();
    const uint32_t pins[2] = { PIN_BUTTON_START, PIN_BUTTON_MODE };

    for (int i = 0; i < 2; i++) {
        // Active-LOW: LOW means button is pressed
        bool reading = (digitalRead(pins[i]) == LOW);

        // If the switch changed state due to noise or pressing
        if (reading != _raw_state[i]) {
            _raw_state[i] = reading;
            _last_change_ms[i] = now;
        }

        // If the state has been stable for longer than the debounce delay
        if ((now - _last_change_ms[i]) >= DEBOUNCE_DELAY_MS) {
            if (reading != _pressed[i]) {
                _pressed[i] = reading;
                // Rising edge: button transitioned from unpressed to pressed
                if (_pressed[i]) {
                    _just_pressed[i] = true;
                }
            }
        }
    }
}

bool button_is_pressed(ButtonID btn)  {
    if (btn > BUTTON_MODE) return false;
    return _pressed[btn];
}

bool button_just_pressed(ButtonID btn) {
    if (btn > BUTTON_MODE) return false;
    bool event = _just_pressed[btn];
    _just_pressed[btn] = false; // Clear edge flag after reading
    return event;
}

void button_wait_press(ButtonID btn) {
    if (btn > BUTTON_MODE) return;
    
    // 1. Wait until button is pressed
    while (!button_is_pressed(btn)) {
        button_update();
        delay(10);
    }
    // 2. Wait until button is released
    while (button_is_pressed(btn)) {
        button_update();
        delay(10);
    }
}
