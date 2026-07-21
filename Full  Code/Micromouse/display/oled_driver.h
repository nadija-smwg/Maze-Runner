/**
 * @file oled_driver.h
 * @brief SSD1306 OLED low-level driver.
 */

#ifndef OLED_DRIVER_H
#define OLED_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize the OLED display.
 * @return true if successful
 */
bool oled_init(void);

/**
 * @brief Clear the display buffer.
 */
void oled_clear(void);

/**
 * @brief Push the buffer to the display.
 */
void oled_update(void);

/**
 * @brief Print text at given cursor position.
 */
void oled_print(uint8_t x, uint8_t y, const char* str);

/**
 * @brief Draw a mini map of the maze.
 */
void oled_draw_maze(void);

#endif /* OLED_DRIVER_H */
