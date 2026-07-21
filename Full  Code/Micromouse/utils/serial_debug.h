/**
 * @file serial_debug.h
 * @brief High-level serial communication and command parser.
 */

#ifndef SERIAL_DEBUG_H
#define SERIAL_DEBUG_H

/**
 * @brief Initialize serial communication.
 */
void serial_debug_init(void);

/**
 * @brief Process incoming serial commands.
 */
void serial_debug_update(void);

#endif /* SERIAL_DEBUG_H */
