/**
 * @file coordinate_transform.cpp
 * @brief Coordinate transform implementation.
 * @see coordinate_transform.h
 */

#include "coordinate_transform.h"
#include "../config/config.h"

uint8_t mm_to_cell(float mm) {
    if (mm < 0.0f) return 0;
    return (uint8_t)(mm / CELL_SIZE_MM);
}

float cell_to_mm(uint8_t cell_index) {
    return (cell_index * (float)CELL_SIZE_MM) + ((float)CELL_SIZE_MM / 2.0f);
}

float get_cell_offset_mm(float current_mm) {
    uint8_t cell = mm_to_cell(current_mm);
    float cell_center = cell_to_mm(cell);
    return current_mm - cell_center;
}
