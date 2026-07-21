/**
 * @file menu.h
 * @brief Button-navigable menu system.
 */

#ifndef MENU_H
#define MENU_H

#include <stdbool.h>

/**
 * @brief Initialize the menu system.
 */
void menu_init(void);

/**
 * @brief Update the menu (process button presses and draw).
 * @return true if a run mode was selected (exit menu)
 */
bool menu_update(void);

#endif /* MENU_H */
