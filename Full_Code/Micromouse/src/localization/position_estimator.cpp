/**
 * @file position_estimator.cpp
 * @brief Position estimator implementation.
 * @see position_estimator.h
 */

#include "position_estimator.h"
#include "odometry.h"
#include "heading_estimator.h"
#include "coordinate_transform.h"
#include "../sensors/distance_manager.h"
#include <math.h>

#ifndef PI
#define PI 3.14159265358979f
#endif

static Pose _best_pose = {0, 0, 0};

void position_estimator_init(void) {
    _best_pose = {0, 0, 0};
}

void position_estimator_update(float dt) {
    // 1. Get raw odometry
    Pose odom = odometry_get_pose();
    
    // 2. Get fused heading
    float fused_heading_deg = heading_estimator_get();
    float fused_heading_rad = fused_heading_deg * (PI / 180.0f);
    
    // 3. Start with odometry position and fused heading
    _best_pose.x_mm = odom.x_mm;
    _best_pose.y_mm = odom.y_mm;
    _best_pose.theta_rad = fused_heading_rad;
    
    // 4. Wall Corrections (Only if driving perfectly straight)
    // To simplify for now, we only correct when facing North (Heading ~ 0)
    // Positive Y is Left. Left ToF measures distance to the wall on the left.
    /* TEMPORARILY DISABLED FOR PHASE 5 FLOOR TESTING 
     * If you are not in a maze, random objects will trigger this and artificially shift Y!
    if (fabs(fused_heading_deg) < 3.0f) {
        if (distance_has_wall_left()) {
            float expected_y = cell_to_mm(mm_to_cell(_best_pose.y_mm)) + 90.0f; 
            float actual_y = expected_y - distance_get_mm(TOF_LEFT);
            _best_pose.y_mm = (0.95f * _best_pose.y_mm) + (0.05f * actual_y);
        }
        if (distance_has_wall_right()) {
            float expected_y = cell_to_mm(mm_to_cell(_best_pose.y_mm)) - 90.0f;
            float actual_y = expected_y + distance_get_mm(TOF_RIGHT);
            _best_pose.y_mm = (0.95f * _best_pose.y_mm) + (0.05f * actual_y);
        }
    }
    */
    
    // 5. Push corrected position and fused heading back to Odometry to prevent cumulative drift
    odometry_set_pose(_best_pose);
}

Pose position_estimator_get_pose(void) {
    return _best_pose;
}
