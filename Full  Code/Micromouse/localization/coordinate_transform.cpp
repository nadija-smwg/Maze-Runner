/**
 * @file coordinate_transform.cpp
 * @brief Coordinate transform implementation.
 * @see coordinate_transform.h
 */

#include "coordinate_transform.h"
#include "../config/config.h"

uint8_t mm_to_cell(float mm) {
    /** TODO: return (uint8_t)(mm / CELL_SIZE_MM); */
    return 0;
}

float cell_to_mm(uint8_t cell_index) {
    /** TODO: return (cell_index * CELL_SIZE_MM) + (CELL_SIZE_MM / 2.0f); */
    return 0.0f;
}

float get_cell_offset_mm(float current_mm) {
    /** TODO: Implementation */
    return 0.0f;
}
