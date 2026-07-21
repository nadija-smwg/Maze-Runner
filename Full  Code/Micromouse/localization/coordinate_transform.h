/**
 * @file coordinate_transform.h
 * @brief Conversion between millimeter and cell coordinates.
 */

#ifndef COORDINATE_TRANSFORM_H
#define COORDINATE_TRANSFORM_H

#include <stdint.h>

/**
 * @brief Convert a millimeter coordinate to a cell index.
 */
uint8_t mm_to_cell(float mm);

/**
 * @brief Convert a cell index to the center coordinate in mm.
 */
float cell_to_mm(uint8_t cell_index);

/**
 * @brief Get the offset in mm from the center of the current cell.
 */
float get_cell_offset_mm(float current_mm);

#endif /* COORDINATE_TRANSFORM_H */
