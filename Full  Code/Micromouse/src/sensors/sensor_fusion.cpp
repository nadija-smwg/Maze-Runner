/**
 * @file sensor_fusion.cpp
 * @brief Sensor fusion implementation — gyro + encoder complementary filter.
 * @see sensor_fusion.h
 *
 * Heading estimation architecture:
 *
 *   mpu6050_get_filtered()  →  gz (°/s)
 *          │
 *   integrate: _gyro_yaw += gz * dt
 *          │
 *   encoder_get_delta() L + R  →  Δθ_enc (°)
 *          │
 *   _enc_yaw += Δθ_enc
 *          │
 *   complementary fuse:
 *     _heading = 0.98 * _gyro_yaw + 0.02 * _enc_yaw
 *          │
 *   normalize to [-180, +180]
 *          │
 *   fusion_get_heading()
 *
 * The 98/2 split trusts the gyro for short-term precision while the
 * encoders slowly correct long-term drift caused by gyro integration error.
 *
 * Velocity is the average of left and right encoder speeds (mm/s).
 */

#include "sensor_fusion.h"
#include "mpu6050.h"
#include "../hardware/encoder.h"
#include "../config/robot_config.h"
#include <math.h>

/* ═══════════════════════════════════════════════════════════════════════════
 *  Fusion Constants
 * ═══════════════════════════════════════════════════════════════════════════ */

/** Gyro contribution to heading (0.98 → trust gyro for short-term accuracy) */
#define FUSION_GYRO_WEIGHT      0.98f

/** Encoder contribution to heading (0.02 → slow long-term drift correction) */
#define FUSION_ENC_WEIGHT       0.02f

/* ═══════════════════════════════════════════════════════════════════════════
 *  Private State
 * ═══════════════════════════════════════════════════════════════════════════ */

/** Gyro-only integrated yaw (degrees). Reset by fusion_reset_heading(). */
static float _gyro_yaw = 0.0f;

/** Encoder-only integrated yaw (degrees). Tracks differential wheel travel. */
static float _enc_yaw  = 0.0f;

/** Fused heading output (degrees, range [-180, +180]). */
static float _heading  = 0.0f;

/** Latest fused linear velocity estimate (mm/s). */
static float _velocity = 0.0f;

/* ═══════════════════════════════════════════════════════════════════════════
 *  Private Helpers
 * ═══════════════════════════════════════════════════════════════════════════ */

/**
 * @brief Normalize an angle to the range [-180, +180] degrees.
 */
static float normalize_180(float angle)
{
    while (angle >  180.0f) angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

/* ═══════════════════════════════════════════════════════════════════════════
 *  Public API
 * ═══════════════════════════════════════════════════════════════════════════ */

void fusion_init(void)
{
    _gyro_yaw = 0.0f;
    _enc_yaw  = 0.0f;
    _heading  = 0.0f;
    _velocity = 0.0f;
}

void fusion_update(float dt)
{
    /* ── 1. Gyro integration ──────────────────────────────────────────────── */
    IMUScaledData imu;
    mpu6050_get_filtered(&imu);

    float gz = imu.gyro_z_dps; /* Filtered Gz in °/s */

    _gyro_yaw += gz * dt;
    _gyro_yaw  = normalize_180(_gyro_yaw);

    /* ── 2. Encoder heading delta ─────────────────────────────────────────── */
    /*
     * Signed encoder deltas since last call (direction-corrected inside
     * encoder_get_delta — positive = forward for both wheels).
     *
     * Δθ_enc (rad) = (DR_mm - DL_mm) / WHEEL_BASE_MM
     * Δθ_enc (°)   = Δθ_enc_rad × (180 / π)
     *
     * Positive Δθ → robot turned right (right wheel traveled further).
     * Negative Δθ → robot turned left.
     */
    int32_t delta_L = encoder_get_delta(ENCODER_LEFT);
    int32_t delta_R = encoder_get_delta(ENCODER_RIGHT);

    float dl_mm = (float)delta_L * MM_PER_COUNT;
    float dr_mm = (float)delta_R * MM_PER_COUNT;

    float enc_delta_deg = (dr_mm - dl_mm) / WHEEL_BASE_MM
                          * (180.0f / (float)M_PI);

    _enc_yaw += enc_delta_deg;
    _enc_yaw  = normalize_180(_enc_yaw);

    /* ── 3. Complementary fuse ───────────────────────────────────────────── */
    _heading = FUSION_GYRO_WEIGHT * _gyro_yaw
             + FUSION_ENC_WEIGHT  * _enc_yaw;

    _heading = normalize_180(_heading);

    /* ── 4. Velocity estimate ─────────────────────────────────────────────── */
    /*
     * Average forward speed of both wheels. Encoder deltas over dt give
     * instantaneous speed in mm/s. No additional filtering here — the
     * velocity controller in velocity_controller.cpp handles its own LPF.
     */
    if (dt > 0.0f)
    {
        float v_left  = dl_mm / dt;
        float v_right = dr_mm / dt;
        _velocity = (v_left + v_right) * 0.5f;
    }
}

float fusion_get_heading(void)
{
    return _heading;
}

void fusion_reset_heading(float new_heading)
{
    /*
     * Sync all internal yaw states to the new heading.
     * Call this after squaring up against a wall or after a known turn.
     */
    _heading  = new_heading;
    _gyro_yaw = new_heading;
    _enc_yaw  = new_heading;
}

float fusion_get_velocity(void)
{
    return _velocity;
}
