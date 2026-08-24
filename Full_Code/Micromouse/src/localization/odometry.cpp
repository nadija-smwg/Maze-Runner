/**
 * @file odometry.cpp
 * @brief Odometry implementation.
 * @see odometry.h
 */

#include "odometry.h"
#include "../config/robot_config.h"
#include "../hardware/encoder.h"
#include "pose.h"
#include <math.h>

static Pose _current_pose = {0, 0, 0};
static long _prev_left = 0;
static long _prev_right = 0;
static float _current_dtheta = 0.0f;

void odometry_init(void) {
    _current_pose = {0.0f, 0.0f, 0.0f};
    _prev_left = encoder_get_count(ENCODER_LEFT);
    _prev_right = encoder_get_count(ENCODER_RIGHT);
}

void odometry_update(void) {
    // 1. Read current encoder counts and calculate delta since last millisecond
    long curr_left = encoder_get_count(ENCODER_LEFT);
    long curr_right = encoder_get_count(ENCODER_RIGHT);
    
    float left_delta_mm = (curr_left - _prev_left) * MM_PER_COUNT;
    float right_delta_mm = (curr_right - _prev_right) * MM_PER_COUNT;

    _prev_left = curr_left;
    _prev_right = curr_right;

    // 2. How far did the center of the robot move?
    float d_center = (left_delta_mm + right_delta_mm) / 2.0f;

    // 3. How much did the robot rotate? 
    _current_dtheta = (right_delta_mm - left_delta_mm) / WHEEL_BASE_MM;

    // 4. Update Heading (Internal Odometry tracking)
    _current_pose.theta_rad = pose_normalize_angle_rad(_current_pose.theta_rad + _current_dtheta);

    // 4. Update X/Y Position (using basic trigonometry)
    _current_pose.x_mm += d_center * cos(_current_pose.theta_rad);
    _current_pose.y_mm += d_center * sin(_current_pose.theta_rad);
}

Pose odometry_get_pose(void) {
    return _current_pose;
}

void odometry_set_pose(Pose new_pose) {
    _current_pose = new_pose;
}

float odometry_get_dtheta(void) {
    return _current_dtheta;
}
