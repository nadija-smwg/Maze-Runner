/**
 * @file pose.cpp
 * @brief Pose utilities implementation.
 * @see pose.h
 */

#include "pose.h"
#include <math.h>

#ifndef PI
#define PI 3.14159265358979f
#endif

float pose_normalize_angle_rad(float angle) {
    while (angle > PI) angle -= 2 * PI;
    while (angle <= -PI) angle += 2 * PI;
    return angle;
}

float pose_normalize_angle_deg(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle <= -180.0f) angle += 360.0f;
    return angle;
}
