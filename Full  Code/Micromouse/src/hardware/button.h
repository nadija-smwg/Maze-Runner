/**
 * @file button.h
 * @brief Debounced button input handling.
 *
 * Provides clean button state with software debouncing and edge detection.
 *
 * Dependencies: pin_config
 */

#ifndef BUTTON_H
#define BUTTON_H

#include "../config/pin_config.h"
#include <stdbool.h>


/**
 * @brief Button identifier enum.
 */
typedef enum {
  BUTTON_START = 0, /**< Start/run button */
  BUTTON_MODE = 1   /**< Mode select button */
} ButtonID;

/**
 * @brief Initialize button GPIO pins with internal pull-ups.
 *
 * Configures PIN_BUTTON_START and PIN_BUTTON_MODE as INPUT_PULLUP.
 */
void button_init(void);

/**
 * @brief Update button debounce state. Call this at regular intervals (~10ms).
 *
 * Implements 20ms software debouncing with edge detection.
 */
void button_update(void);

/**
 * @brief Check if a button is currently pressed.
 *
 * @param btn Button identifier
 * @return    true if pressed (debounced)
 */
bool button_is_pressed(ButtonID btn);

/**
 * @brief Check if a button was just pressed (rising edge).
 *
 * Returns true only once per press event.
 *
 * @param btn Button identifier
 * @return    true on press edge
 */
bool button_just_pressed(ButtonID btn);

/**
 * @brief Block until a button is pressed and released.
 *
 * @param btn Button identifier
 *
 * Polls button_update() in a tight loop with 10ms delay until press+release.
 */
void button_wait_press(ButtonID btn);

#endif /* BUTTON_H */
