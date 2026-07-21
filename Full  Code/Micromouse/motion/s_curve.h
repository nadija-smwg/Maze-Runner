/**
 * @file s_curve.h
 * @brief S-curve profile adapter.
 */

#ifndef S_CURVE_H
#define S_CURVE_H

#include "motion_profile.h"

/**
 * @brief Adapt a linear profile to an S-curve.
 *
 * Smooths the acceleration transitions.
 *
 * @param profile Pointer to the linear profile
 * @param distance_so_far Current distance
 * @return Smoothed target speed
 */
float s_curve_apply(const LinearProfile *profile, float distance_so_far);

#endif /* S_CURVE_H */
