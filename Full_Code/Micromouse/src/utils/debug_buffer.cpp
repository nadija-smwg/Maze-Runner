/**
 * @file debug_buffer.cpp
 * @brief RAM-based ring buffer for offline debug logging.
 * @see debug_buffer.h
 *
 * Stores debug snapshots in a circular buffer. When the buffer is full,
 * new entries overwrite the oldest ones. This means you always have
 * the LAST N entries (most recent data, which is usually what you need
 * when diagnosing a crash or oscillation).
 */

#include "debug_buffer.h"
#include <Arduino.h>

// The ring buffer lives in RAM
static DebugEntry _buffer[DEBUG_BUFFER_SIZE];
static uint16_t _write_index = 0;   // Next position to write
static uint16_t _count = 0;         // Total entries stored (caps at DEBUG_BUFFER_SIZE)
static bool _full = false;          // Has the buffer wrapped around?

void debug_buffer_init(void) {
    _write_index = 0;
    _count = 0;
    _full = false;
    // Zero out the buffer (optional but clean)
    memset(_buffer, 0, sizeof(_buffer));
}

void debug_buffer_sample(const DebugEntry* entry) {
    if (entry == NULL) return;
    
    _buffer[_write_index] = *entry;
    _write_index = (_write_index + 1) % DEBUG_BUFFER_SIZE;
    
    if (_count < DEBUG_BUFFER_SIZE) {
        _count++;
    } else {
        _full = true;
    }
}

void debug_buffer_dump_serial(void) {
    // Print CSV header
    Serial.println();
    Serial.println("=== DEBUG BUFFER DUMP START ===");
    Serial.println("time_ms,target_v,current_v,left_speed,right_speed,left_pwm,right_pwm,heading_deg,battery_mv");
    
    if (_count == 0) {
        Serial.println("(empty)");
        Serial.println("=== DEBUG BUFFER DUMP END ===");
        return;
    }
    
    // Calculate the start index (oldest entry)
    uint16_t start;
    uint16_t entries_to_print;
    
    if (_full) {
        // Buffer has wrapped — oldest entry is at _write_index
        start = _write_index;
        entries_to_print = DEBUG_BUFFER_SIZE;
    } else {
        // Buffer hasn't wrapped — oldest entry is at 0
        start = 0;
        entries_to_print = _count;
    }
    
    // Print all entries in chronological order
    for (uint16_t i = 0; i < entries_to_print; i++) {
        uint16_t idx = (start + i) % DEBUG_BUFFER_SIZE;
        DebugEntry* e = &_buffer[idx];
        
        // Print as CSV: timestamp, target, current, L_speed, R_speed, L_pwm, R_pwm, heading, battery
        Serial.print(e->timestamp_ms);
        Serial.print(',');
        Serial.print(e->target_v);
        Serial.print(',');
        Serial.print(e->current_v);
        Serial.print(',');
        Serial.print(e->left_speed);
        Serial.print(',');
        Serial.print(e->right_speed);
        Serial.print(',');
        Serial.print(e->left_pwm);
        Serial.print(',');
        Serial.print(e->right_pwm);
        Serial.print(',');
        // heading_x10 stores degrees × 10, convert back
        Serial.print(e->heading_x10 / 10.0f, 1);
        Serial.print(',');
        Serial.println(e->battery_mv);
    }
    
    Serial.print("=== DEBUG BUFFER DUMP END (");
    Serial.print(entries_to_print);
    Serial.println(" entries) ===");
}

uint16_t debug_buffer_get_count(void) {
    return _count;
}

bool debug_buffer_is_full(void) {
    return _full;
}

void debug_buffer_clear(void) {
    _write_index = 0;
    _count = 0;
    _full = false;
}
