# Current Progress Report & Roadmap

## 1. Overall Status
Based on the completed bring-up phase, the hardware foundation is essentially finished. This constitutes approximately 25–30% of a competition micromouse. The remaining work focuses predominantly on robotics algorithms, control systems, localization, and maze solving.

### Completed Milestones (Hardware Bring-up)
- [x] STM32F401 running reliably
- [x] Register-level PWM (TIM1)
- [x] Hardware quadrature encoders (TIM2/TIM3)
- [x] MPU6050 with calibration and complementary filter
- [x] Motor driver abstraction
- [x] Distance measurement
- [x] Differential drive functions
- [x] Hardware architecture finalized

## 2. Overall Development Roadmap

- **Stage 1**: Hardware Bring-up ✅ DONE
- **Stage 2**: Motion Control ⬅️ NEXT (Crucial stage)
- **Stage 3**: Sensor Integration
- **Stage 4**: Robot Localization
- **Stage 5**: Motion Primitives
- **Stage 6**: Wall Following
- **Stage 7**: Maze Mapping
- **Stage 8**: Flood Fill
- **Stage 9**: Speed Runs
- **Stage 10**: Competition Optimization

---

## 3. Immediate Goal: Stage 2 — Motion Control

This is the most critical stage. Without precise motion control, subsequent algorithms (like maze mapping and wall following) become unreliable.

**Target Accuracy:**
- **Command:** Forward 180 mm ➡️ **Actual:** 180.02 mm (Heading error = 0.3°)
- *NOT* 176 mm with a Heading error of 8°.

### Module 1: Characterize Motors
Before implementing PID, you must understand the motors.
- Map **PWM** ➡️ **Speed** ➡️ **Encoder counts/sec**.
- Record Left/Right encoder counts per second for PWM values (e.g., 500, 1000, ... 4999).
- Graphing this will help identify:
  - **Deadband** (e.g., 0-600: No movement)
  - **Linear Region** (e.g., 600-4200: Linear response)
  - **Saturation** (e.g., 4200-4999: Saturated)
- *Note:* Never run the PID controller inside the saturation region.

### Module 2: Wheel Speed Calculation
Translate encoder counts into meaningful physical units:
1. `deltaCounts` ➡️ `counts/sec`
2. `RPM = (counts/sec × 60) / CPR`
3. `Linear velocity (mm/s) = (RPM × πD) / 60` (where `D` = wheel diameter in mm)

### Module 3: Speed PID
Transition from open-loop PWM commands to closed-loop speed commands.
- Command a **Desired Speed** (e.g., 400 mm/s), and the PID loop calculates the necessary PWM.
- **Loop:** Desired Speed ➡️ PID ➡️ PWM ➡️ Motor ➡️ Encoder ➡️ Measured Speed ➡️ PID
- Run this loop at **200-500 Hz**.
- *Crucial:* Never use `delay()`; use timer-based execution like `micros()` or hardware timers.

### Module 4: Straight Line Controller
Prevent the robot from drifting by using encoders combined with the gyroscope.
- `Heading error = Target heading - Current heading`
- Calculate `Correction` using a Heading PID.
- `Left PWM = Base PWM - Correction`
- `Right PWM = Base PWM + Correction`

### Module 5: Position PID
Move exact distances using a cascaded controller.
- Command a **Desired Distance**.
- **Loop Structure:** Position PID ➡️ Desired Speed ➡️ Speed PID ➡️ Motor
- This structure is standard in professional robotics.

### Module 6: Rotation Controller
Execute precise turns (e.g., 90°, 180°, 45°).
- Input: Gyroscope yaw.
- Controller: Angle PID.
- Goal for a 90° turn: **90.00° ±0.5°**.

---

## 4. Future Stages (Post-Motion Control)

- **Stage 3: Sensor Fusion**
  - Connect VL53L0X sensors. Start with the front sensor and calibrate against actual physical measurements. Then add left, right, and diagonal sensors.
- **Stage 4: Robot Localization**
  - Combine Encoders + Gyroscope to estimate `x`, `y`, and `heading` using odometry equations.
- **Stage 5: Motion Primitives**
  - Build high-level functions for the maze solver: `moveOneCell()`, `moveHalfCell()`, `turnLeft90()`, `turnRight90()`, `turn180()`, `stopExactly()`, `alignToWall()`, `centerBetweenWalls()`.
- **Stage 6: Wall Following**
  - Use left and right distance sensors to calculate error (`Left - Right`) and feed it into a PID to adjust heading and motor PWM, keeping the robot centered.
- **Stage 7: Maze Mapping**
  - Represent the maze as a 16×16 grid. Each cell stores walls (North, South, East, West), Visited status, and Cost.
- **Stage 8: Flood Fill**
  - Implement maze-solving algorithms (Flood Fill, DFS, BFS, A*). The flow: Explore ➡️ Build map ➡️ Compute shortest path ➡️ Speed run.
- **Stage 9: Speed Runs**
  - Optimize the shortest path: increase velocity, smooth acceleration/deceleration, minimize stops, optimize turns.

---

## 5. Software Architecture Recommendations

As the project scales, transition from a single monolithic file to a modular structure:

```text
Core/
    main.cpp

Drivers/
    mpu6050.cpp
    encoder.cpp
    motor.cpp
    vl53l0x.cpp

Control/
    pid.cpp
    speed_pid.cpp
    heading_pid.cpp
    position_pid.cpp

Localization/
    odometry.cpp

Motion/
    motion.cpp
    trajectory.cpp

Maze/
    maze.cpp
    floodfill.cpp

Utilities/
    timer.cpp
    filter.cpp
```

## 6. Suggested Implementation Order (Do Not Skip)

1. [x] Motor driver
2. [x] Encoder driver
3. [x] MPU6050 driver
4. [ ] Motor characterization
5. [ ] Encoder speed calculation
6. [ ] Speed PID
7. [ ] Straight-line controller
8. [ ] Distance (position) PID
9. [ ] 90° turn controller
10. [ ] VL53L0X integration
11. [ ] Wall centering
12. [ ] Odometry
13. [ ] Motion primitives
14. [ ] Maze representation
15. [ ] Flood Fill
16. [ ] Exploration
17. [ ] Shortest-path speed run
18. [ ] Competition optimization

---

## 7. Strategic Recommendations

> [!IMPORTANT]
> **Do not connect the VL53L0X sensors yet.** Your absolute next milestone should be to achieve precise motion control using *only* the encoders and MPU6050.

**Immediate Validation Goals:**
1. Drive a commanded distance (e.g., 180 mm) with less than ±1 mm error.
2. Drive straight for 1 meter with less than ±2° heading drift.
3. Execute 90° turns with less than ±0.5° error.
4. Build stable cascaded speed, heading, and position PID controllers.

Once these foundational controls are reliable, integrating the distance sensors and implementing maze-solving algorithms becomes significantly more straightforward because the robot's underlying motion is already accurate and repeatable.
