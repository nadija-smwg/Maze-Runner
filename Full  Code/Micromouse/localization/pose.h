/**
 * @file pose.h
 * @brief Pose structure and utilities.
 */

#ifndef POSE_H
#define POSE_H

/**
 * @brief Robot pose (position and orientation).
 */
typedef struct {
    float x_mm;      /**< X position in millimeters */
    float y_mm;      /**< Y position in millimeters */
    float theta_rad; /**< Heading angle in radians (0 = North, + is CCW) */
} Pose;

/**
 * @brief Normalize an angle to [-PI, PI].
 */
float pose_normalize_angle_rad(float angle);

/**
 * @brief Normalize an angle to [-180, 180].
 */
float pose_normalize_angle_deg(float angle);

#endif /* POSE_H */
