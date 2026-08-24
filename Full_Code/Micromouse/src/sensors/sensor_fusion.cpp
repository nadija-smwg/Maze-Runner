/**
 * @file sensor_fusion.cpp
 * @brief Sensor fusion implementation.
 * @see sensor_fusion.h
 */

#include "sensor_fusion.h"
#include "mpu6050.h"
#include "../hardware/encoder.h"
#include "../localization/odometry.h"
#include "../localization/heading_estimator.h"
#include "../localization/position_estimator.h"
#include "../config/robot_config.h"

void fusion_init(void) {
    odometry_init();
    heading_estimator_init();
    position_estimator_init();
}

void fusion_update(float dt) {
    // 1. Update encoders (internal odometry)
    odometry_update();
    
    // 2. Read IMU
    IMUScaledData imu;
    mpu6050_read_scaled(&imu);
    
    // 3. Complementary Filter: Pass gyro rate and encoder delta theta
    // The filter will fuse the high-frequency gyro data with the low-frequency encoder data.
    heading_estimator_update(imu.gyro_z_dps, odometry_get_dtheta(), dt);
    
    // 4. Wall corrections
    position_estimator_update(dt);
}

float fusion_get_heading(void) {
    return heading_estimator_get();
}

void fusion_reset_heading(float new_heading) {
    // Force the internal states
    Pose p = odometry_get_pose();
    p.theta_rad = new_heading * (3.14159265f / 180.0f);
    odometry_set_pose(p);
    
    // Also reset the complementary filter state
    // (This requires a setter in heading_estimator, but for now we rely on the 2% pull or just restarting)
    // For simplicity, re-init.
    heading_estimator_init();
}

float fusion_get_velocity(void) {
    // Velocity can be derived from wheel encoders
    // We can compute it in odometry, but for now just return 0.
    return 0.0f;
}
