# Phase 3 — Kinematics & Control Systems

Prerequisite: working PWM motors, encoders, and (ideally) gyro from Phase 2.

## 3.1 Dead Reckoning — Position from Encoder Ticks

### Constants you need (measure these physically)
```c
#define TICKS_PER_REV      1560.0f   // from your Phase 2 checkpoint measurement
#define WHEEL_DIAMETER_MM  32.0f
#define WHEEL_BASE_MM      75.0f     // distance between the two wheel contact patches
#define MM_PER_TICK        ((PI * WHEEL_DIAMETER_MM) / TICKS_PER_REV)
```

### Differential-drive odometry
Every control tick (e.g. 1kHz from Phase 1's TIM10), compute incremental
distance traveled by each wheel, then update robot pose `(x, y, theta)`:

```c
typedef struct {
    float x_mm, y_mm;      // position in the maze's global frame
    float theta_rad;       // heading, 0 = facing +x (e.g. "east")
} Pose;

Pose pose = {0};

void update_odometry(int32_t delta_left_ticks, int32_t delta_right_ticks) {
    float d_left  = delta_left_ticks  * MM_PER_TICK;
    float d_right = delta_right_ticks * MM_PER_TICK;
    float d_center = (d_left + d_right) / 2.0f;
    float d_theta   = (d_right - d_left) / WHEEL_BASE_MM;  // radians

    // Midpoint integration — more accurate than simple Euler for curved segments
    float theta_mid = pose.theta_rad + d_theta / 2.0f;
    pose.x_mm += d_center * cosf(theta_mid);
    pose.y_mm += d_center * sinf(theta_mid);
    pose.theta_rad += d_theta;

    // Normalize theta to [-pi, pi] to avoid unbounded growth / precision loss
    if (pose.theta_rad > PI)  pose.theta_rad -= 2*PI;
    if (pose.theta_rad < -PI) pose.theta_rad += 2*PI;
}
```

This is pure kinematics — no algorithm cleverness, just correct geometry.
The **midpoint heading approximation** matters: at 1kHz with small per-tick
angle changes it's a minor correction, but it's free accuracy and standard
practice, so use it.

**Known limitation:** wheel slip (especially during aggressive turns or wall
contact) corrupts this silently — encoders don't know the difference between
"wheel rotated" and "wheel rotated AND scuffed sideways." This is exactly why
Phase 3.3 (sensor fusion) exists — gyro yaw doesn't suffer from this failure
mode and can correct `theta` errors that pure odometry accumulates.

**Checkpoint 3.1:** Push the robot manually in a straight 500mm line, confirm
`pose.x_mm`/`y_mm` tracks reasonably. Push it through a 90° turn in place,
confirm `theta_rad` changes by ~π/2.

---

## 3.2 PID Controller Implementation

You'll need (at minimum) **two independent PID controllers running
simultaneously** during a straight-line move:
1. **Velocity/speed PID** per wheel (or a combined forward-speed PID) — keeps
   the robot moving at the commanded speed.
2. **Heading PID** — keeps the robot going straight by correcting for
   left/right speed mismatch (motor variance, friction asymmetry, etc.),
   using `theta` error as input, output as a differential added to left/right
   wheel speed commands.

For turns-in-place, you instead run a heading PID targeting a specific
`theta` setpoint (e.g. current + 90°), with both wheels driven in opposite
directions.

### Generic PID class (C++, since you're comfortable there)
```cpp
class PID {
public:
    PID(float kp, float ki, float kd, float out_min, float out_max)
        : kp_(kp), ki_(ki), kd_(kd), out_min_(out_max ? out_min : out_min),
          out_max_(out_max), integral_(0), prev_error_(0), first_call_(true) {}

    float update(float setpoint, float measured, float dt) {
        float error = setpoint - measured;

        integral_ += error * dt;
        // Anti-windup: clamp integral contribution to output range
        float i_term = ki_ * integral_;
        if (i_term > out_max_) { i_term = out_max_; integral_ = i_term / ki_; }
        if (i_term < out_min_) { i_term = out_min_; integral_ = i_term / ki_; }

        float derivative = first_call_ ? 0.0f : (error - prev_error_) / dt;
        first_call_ = false;
        prev_error_ = error;

        float output = kp_ * error + i_term + kd_ * derivative;
        if (output > out_max_) output = out_max_;
        if (output < out_min_) output = out_min_;
        return output;
    }

    void reset() { integral_ = 0; prev_error_ = 0; first_call_ = true; }

private:
    float kp_, ki_, kd_;
    float out_min_, out_max_;
    float integral_, prev_error_;
    bool first_call_;
};
```

Note the two non-negotiable production details baked in:
- **Anti-windup clamping** — without it, the integral term explodes during
  saturation (e.g. commanded speed the motor physically can't reach), causing
  massive overshoot when the error finally reverses sign. This bites everyone
  once; build it in from day one.
- **`first_call_` guard on derivative** — otherwise your first control call
  computes a derivative against garbage `prev_error_ = 0`, causing a kick.

### Using it for straight-line motion
```cpp
PID heading_pid(HEADING_KP, HEADING_KI, HEADING_KD, -300, 300); // output: differential speed
PID speed_pid(SPEED_KP, SPEED_KI, SPEED_KD, -1000, 1000);       // output: base motor command

void run_control_loop() {
    int32_t dl = read_delta_left_ticks();
    int32_t dr = read_delta_right_ticks();
    update_odometry(dl, dr);

    float dt = 0.001f; // 1kHz loop

    float current_speed_mm_s = ((dl + dr) / 2.0f * MM_PER_TICK) / dt;
    float base_cmd = speed_pid.update(target_speed_mm_s, current_speed_mm_s, dt);

    float heading_error = target_theta - pose.theta_rad;
    // normalize heading_error to [-pi, pi] — shortest angular path
    if (heading_error > PI) heading_error -= 2*PI;
    if (heading_error < -PI) heading_error += 2*PI;
    float correction = heading_pid.update(0, -heading_error, dt); // drive error to 0

    set_motor_left(base_cmd - correction);
    set_motor_right(base_cmd + correction);
}
```

### Turning in place
Same `heading_pid`, but `target_speed_mm_s = 0`, and instead of correcting a
base forward command, output drives both wheels directly in opposite signs:
```cpp
void turn_to(float target_theta_rad) {
    heading_pid.reset();
    while (fabsf(angle_diff(target_theta_rad, pose.theta_rad)) > TURN_TOLERANCE_RAD) {
        // called from control loop context; pseudo-code for clarity here
        float err = angle_diff(target_theta_rad, pose.theta_rad);
        float turn_cmd = heading_pid.update(0, -err, 0.001f);
        set_motor_left(-turn_cmd);
        set_motor_right(turn_cmd);
    }
    set_motor_left(0); set_motor_right(0);
}
```

### Tuning methodology (Ziegler-Nichols-adjacent, practical version)
1. Set Ki = Kd = 0. Increase Kp until the robot oscillates steadily around
   the setpoint (straight line or turn). Note this "critical" Kp.
2. Back Kp off to ~50-60% of that critical value.
3. Add Kd gradually — it damps oscillation/overshoot, letting you push Kp a
   bit higher again. Watch for high-frequency jitter (Kd too high amplifies
   encoder noise — this is why an IC filter in Phase 2.2 mattered).
4. Add small Ki only if you see persistent steady-state error (e.g. always
   drifting slightly left) — most micromice run with very small or zero Ki
   on the heading loop since gyro/encoder correction is fast enough without it.

**Checkpoint 3.2:** Tune a straight-line PID that reliably crosses one 180mm
maze cell without drifting into a wall. Tune a 90° in-place turn PID that
consistently lands within a few degrees of target with minimal overshoot.

---

## 3.3 Sensor Fusion — Encoder + Gyro for Drift Correction

Pure encoder-based `theta` (Phase 3.1) drifts under wheel slip. Pure gyro
integration drifts too, from bias residual and integration error accumulating
over time. **Complementary filter** is the standard lightweight fix — no need
for a full Kalman filter at this scale (a Kalman filter is defensible if you
want to push further, but a complementary filter gets you 90% of the benefit
at 10% of the complexity and CPU cost, which matters on an F401 running a
1kHz loop with maze logic also competing for cycles).

```cpp
float fused_theta = 0;

void update_fused_heading(float gyro_dps, float encoder_theta_rad, float dt) {
    float gyro_delta_rad = gyro_dps * (PI / 180.0f) * dt;
    float gyro_theta = fused_theta + gyro_delta_rad;

    // Complementary filter: trust gyro short-term (low noise, no slip-sensitivity),
    // trust encoder long-term (zero long-run drift since it's tied to real distance/turns)
    const float ALPHA = 0.98f;
    fused_theta = ALPHA * gyro_theta + (1.0f - ALPHA) * encoder_theta_rad;
}
```

Use `fused_theta` (not raw `pose.theta_rad`) as the heading PID's feedback
signal. Keep computing `pose.theta_rad` from odometry too — it's still your
best source for **position** (x, y), just not the most trustworthy source for
instantaneous heading during a turn.

**A note on where fusion actually earns its keep in a micromouse specifically:**
during **fast, aggressive turns**, wheel slip is worst, meaning encoder-based
theta is least trustworthy exactly when you need heading accuracy most. This
is precisely when the gyro's higher weighting (`ALPHA = 0.98`) pays off. During
straight-line search-run driving at moderate speed, slip is minimal and the
two sources should already roughly agree — the complementary filter smoothly
handles both regimes without you needing to switch modes explicitly.

**Checkpoint 3.3:** Compare `pose.theta_rad` (encoder-only) vs `fused_theta`
after 10 consecutive 90° turns interspersed with short straight runs — the
fused value should track a physically-verified heading (e.g. checked against
a printed protractor/turntable) noticeably better than encoder-only, especially
if you deliberately induce some wheel slip.

---

## Phase 3 Summary — What You Should Now Be Able To Do
- Compute robot pose from encoder deltas using correct differential-drive
  kinematics with midpoint heading integration.
- Implement and tune a PID controller with anti-windup, using it for both
  straight-line speed/heading holding and in-place 90° turns.
- Explain why raw odometry drifts and how a complementary filter combining
  gyro and encoder data mitigates it, and implement that fusion.

Confirm and we'll move to **[Phase 4: Maze Solving
Algorithms](04-phase4-maze-solving.md)** — this is where your DSA background
gets to shine: maze representation, Flood Fill, and translating the algorithm's
output into the motion primitives you just built.
