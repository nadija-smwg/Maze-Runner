# Motion Control Debugging Report

I understand how frustrating it is when the robot just vibrates and stutters instead of moving. This report breaks down **exactly what was implemented**, **how the data flows**, and **the most likely reasons** for this failure so we can systematically fix it.

---

## 1. How the Control System is Currently Implemented

Every 1 millisecond (1kHz), the following chain of events happens in `Micromouse.ino` when you press `START`:

1. **State Machine (`Micromouse.ino`)**
   - Target Linear Velocity ($v$) is set to `150 mm/s`.
   - Target Heading is locked to whatever direction the robot is facing.
   - It calls `heading_controller_update()`.

2. **Heading PID (`heading_controller.cpp`)**
   - Compares Target Heading with Current IMU Heading.
   - Outputs an angular velocity correction, $\omega$ (omega). If the robot is perfectly straight, $\omega = 0$.

3. **Kinematics Engine (`velocity_controller.cpp`)**
   - Converts the robot's target $(v, \omega)$ into specific wheel targets:
     - $v_{left} = v - (\omega \times \text{WheelBase} / 2)$
     - $v_{right} = v + (\omega \times \text{WheelBase} / 2)$
   - Reads the raw encoder counts from the last 1 millisecond.
   - Multiplies the raw counts by 1000 to get counts/sec, converts to mm/s, and passes them through a **Low Pass Filter** to smooth out the jagged values.
   - Passes the target and current wheel speeds to the `speed_controller`.

4. **Wheel Speed PID (`speed_controller.cpp`)**
   - Applies **Feedforward**: Instantly guesses the required PWM. (`150 mm/s * 5.0 = 750 PWM`).
   - Applies **PID**: Looks at the error between the filtered target speed and current speed, and adjusts the PWM. (Currently $K_p = 2.0, K_i = 1.0$).
   - Calls `motor_set_both()` to physically drive the motors.

---

## 2. Most Likely Causes for the Violent Stuttering

Based on the symptoms (rapid start/stop, loud noise, no real forward movement), here are the absolute most likely culprits:

### Culprit A: Hardware Polarity Mismatch (Positive Feedback Loop)
This is the **#1 cause of this exact behavior in robotics**. 
If sending a **positive** PWM signal to drive the motor forward physically results in the encoder counting **backwards** (negative speed), the PID loop breaks completely.
* **What happens:** The PID wants `+150 mm/s`. It sends positive PWM. The robot moves forward, but the encoder reads `-50 mm/s`. The PID sees a massive error of `200 mm/s`. It sends *more* positive PWM. The encoder reads `-100 mm/s`. The PID violently maxes out the PWM at `4200` in a millisecond, triggering electrical noise or brownouts, and stuttering.

### Culprit B: IMU Vibration Feedback
The `heading_controller` relies on the IMU. N20 metal gear motors cause physical vibrations. If those vibrations are picked up by the MPU6050, the IMU heading rapidly fluctuates by a few degrees.
* **What happens:** The `heading_controller` sees the heading rapidly shifting left and right, so it violently swings the target angular velocity ($\omega$) between +5 and -5 rad/s. This commands the left and right wheels to rapidly alternate going extremely fast and extremely slow, resulting in a horrible stuttering noise.

### Culprit C: The Low-Pass Filter Lag
If the Low Pass Filter in the velocity controller is too aggressive ($\alpha = 0.05$), the speed reading is extremely smooth, but it is **delayed** by about 20-30 milliseconds.
* **What happens:** The PID applies power, but doesn't "see" the speed increase until 30ms later. It thinks the motor is stuck, so it pumps more power. 30ms later, the speed reading catches up and tells the PID it is going *way* too fast, so the PID slams on the brakes (negative PWM). This results in a heavy oscillation.

---

## 3. Step-by-Step Debugging Plan

To fix this, we must isolate the systems. You'll need to run a few tests and let me know the results.

### Step 1: Verify Feedforward (Open Loop Test)
We must ensure the motors can drive smoothly *without* the PID.
1. Open `src/control/speed_controller.cpp`.
2. Change the PID gains to exactly zero:
   ```cpp
   #define SPEED_KP 0.0f
   #define SPEED_KI 0.0f
   #define SPEED_KD 0.0f
   ```
3. Upload and press `START` on Phase 5.
**What to look for:** Does the robot drive forward smoothly (even if it drifts left/right)? If it still stutters, the problem is IMU Vibration (Culprit B). If it drives smoothly, we know the motors and PWM are fine.

### Step 2: Verify Encoder Polarity
We must ensure that physically pushing the robot forward results in POSITIVE speeds.
1. Change `p5_state == 0` (IDLE mode) in `Micromouse.ino` to print the raw encoder counts to the OLED.
2. Push the robot forward by hand. 
**What to look for:** Do both the Left and Right encoder counts go UP? If one or both go DOWN, we have Culprit A. We simply need to negate the output in `encoder_get_count()` / `encoder_get_delta()` for that wheel.

### Step 3: Disable Heading Controller (Isolate Wheels)
If the motors drive smoothly in Step 1, we must figure out if the IMU is causing the issue or the Speed PID.
1. Open `Micromouse.ino` line ~633 inside Phase 5.
2. Change `float omega = heading_controller_update(...)` to `float omega = 0.0f;`.
3. Put the PID gains back to `2.0` and `1.0`.
4. Upload and press `START`.
**What to look for:** Does it drive smoothly now? If yes, the `heading_controller` is the problem (IMU vibrations or bad heading PID tuning). If it still stutters violently, the Speed PID is the problem (Culprit C - filter lag).

Let me know which test you would like me to write the code for, or if you want to try editing the values yourself based on this report!
