/**
 * @file odometry.cpp
 * @brief Odometry implementation.
 * @see odometry.h
 */

#include "odometry.h"
#include "../config/robot_config.h"
#include <math.h>

static Pose _current_pose = {0, 0, 0};

void odometry_init(void) {
    _current_pose = {0.0f, 0.0f, 0.0f};
}

void odometry_update(float left_delta_mm, float right_delta_mm) {
    /**
     * TODO:
     * float d_center = (left_delta_mm + right_delta_mm) / 2.0f;
     * float d_theta = (right_delta_mm - left_delta_mm) / WHEEL_BASE_MM;
     * _current_pose.theta_rad = pose_normalize_angle_rad(_current_pose.theta_rad + d_theta);
     * _current_pose.x_mm += d_center * cos(_current_pose.theta_rad);
     * _current_pose.y_mm += d_center * sin(_current_pose.theta_rad);
     */
}

Pose odometry_get_pose(void) {
    return _current_pose;
}

void odometry_set_pose(Pose new_pose) {
    _current_pose = new_pose;
}
