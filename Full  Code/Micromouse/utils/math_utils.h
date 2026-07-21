/**
 * @file math_utils.h
 * @brief Math helper functions (mapping, constraining, wrapping).
 */

#ifndef MATH_UTILS_H
#define MATH_UTILS_H

/**
 * @brief Constrain a float value between a min and max.
 */
float math_constrain(float val, float min_val, float max_val);

/**
 * @brief Map a value from one range to another.
 */
float math_map(float x, float in_min, float in_max, float out_min, float out_max);

#endif /* MATH_UTILS_H */
