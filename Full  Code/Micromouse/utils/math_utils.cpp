/**
 * @file math_utils.cpp
 * @brief Math utilities implementation.
 * @see math_utils.h
 */

#include "math_utils.h"

float math_constrain(float val, float min_val, float max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

float math_map(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
