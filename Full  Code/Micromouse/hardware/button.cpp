/**
 * @file button.cpp
 * @brief Button debounce implementation.
 * @see button.h
 */

#include "button.h"
#include <Arduino.h>

static bool _pressed[2]      = {false, false};
static bool _just_pressed[2] = {false, false};

void button_init(void) {
    /** TODO: pinMode(PIN_BUTTON_START, INPUT_PULLUP); etc. */
}

void button_update(void) {
    /** TODO: Read pins, debounce, detect edges, update _pressed and _just_pressed. */
}

bool button_is_pressed(ButtonID btn)  { return _pressed[btn]; }
bool button_just_pressed(ButtonID btn) {
    /** TODO: Return and clear edge flag. */
    return false;
}

void button_wait_press(ButtonID btn) {
    /** TODO: While loop with button_update() and small delay. */
}
