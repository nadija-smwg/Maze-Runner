/**
 * @file debug_screen.h
 * @brief Real-time debug data display on OLED.
 */

#ifndef DEBUG_SCREEN_H
#define DEBUG_SCREEN_H

/**
 * @brief Draw sensor values on the OLED.
 */
void debug_screen_draw_sensors(void);

/**
 * @brief Draw PID tuning values and errors.
 */
void debug_screen_draw_pid(void);

/**
 * @brief Draw current pose (x, y, theta).
 */
void debug_screen_draw_pose(void);

#endif /* DEBUG_SCREEN_H */
