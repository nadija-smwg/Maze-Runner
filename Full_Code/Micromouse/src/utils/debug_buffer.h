/**
 * @file debug_buffer.h
 * @brief RAM-based ring buffer for offline debug logging.
 *
 * Stores the last N debug snapshots in RAM while the robot drives untethered.
 * After stopping, plug in USB and press BUTTON_MODE (long press) to dump
 * the entire buffer as CSV to Serial Monitor.
 *
 * Memory usage: 200 entries × 20 bytes = 4000 bytes (~4KB of 96KB RAM)
 *
 * Usage:
 *   debug_buffer_init();                  // Call in setup()
 *   debug_buffer_sample(data);            // Call at 100Hz while driving
 *   debug_buffer_dump_serial();           // Call when user presses button
 *   debug_buffer_get_count();             // Show "LOG: 150/200" on OLED
 */

#ifndef DEBUG_BUFFER_H
#define DEBUG_BUFFER_H

#include <stdint.h>

// Maximum entries stored in RAM. 200 entries at 100Hz = 2 seconds of data.
// Increase to 500 for 5 seconds (costs ~10KB RAM).
#define DEBUG_BUFFER_SIZE 200

/**
 * @brief One snapshot of robot state, packed to minimize RAM usage.
 * 
 * Total: 20 bytes per entry × 200 entries = 4000 bytes
 */
typedef struct {
    uint16_t timestamp_ms;   // Relative timestamp (wraps at 65535ms = ~65s)
    int16_t  target_v;       // Target velocity (mm/s), range: -1000 to +1000
    int16_t  current_v;      // Measured velocity (mm/s)
    int16_t  left_speed;     // Left wheel speed (mm/s)
    int16_t  right_speed;    // Right wheel speed (mm/s)
    int16_t  left_pwm;       // Left motor PWM
    int16_t  right_pwm;      // Right motor PWM
    int16_t  heading_x10;    // Heading in degrees × 10 (e.g., 12.5° = 125)
    uint16_t battery_mv;     // Battery voltage in mV
} DebugEntry;

/**
 * @brief Initialize the debug buffer (clears all entries).
 */
void debug_buffer_init(void);

/**
 * @brief Record one snapshot into the ring buffer.
 * 
 * Call this at ~100Hz while the robot is driving.
 * Oldest entries are overwritten when buffer is full.
 *
 * @param entry The debug snapshot to record
 */
void debug_buffer_sample(const DebugEntry* entry);

/**
 * @brief Dump the entire buffer to Serial as CSV.
 *
 * Prints a CSV header followed by all stored entries in chronological order.
 * Call this after plugging in USB when the robot is idle.
 */
void debug_buffer_dump_serial(void);

/**
 * @brief Get the number of entries currently stored.
 * @return Count (0 to DEBUG_BUFFER_SIZE)
 */
uint16_t debug_buffer_get_count(void);

/**
 * @brief Check if the buffer is full (has wrapped around).
 * @return true if buffer has been fully filled at least once
 */
bool debug_buffer_is_full(void);

/**
 * @brief Clear all entries from the buffer.
 */
void debug_buffer_clear(void);

#endif /* DEBUG_BUFFER_H */
