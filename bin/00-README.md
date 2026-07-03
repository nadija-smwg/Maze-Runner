# Micromouse on STM32 Black Pill — Full Curriculum
### SLIIT ROBOFEST 2026 Prep | Beginner → Expert Embedded Track

This is a 5-phase curriculum. **Rule: don't open Phase N+1 until you can explain
Phase N's concepts back without notes.** Each file is self-contained but assumes
everything before it.

## Target Hardware Assumptions
| Component | Part | Notes |
|---|---|---|
| MCU | STM32F411CEU6 "Black Pill" | 100 MHz Cortex-M4F, 512KB Flash, 128KB RAM |
| Motor Driver | TB6612FNG | Dual H-bridge, better efficiency than L298N |
| Motors | N20 micro gearmotors w/ magnetic encoders | ~1000+ CPR after gearbox |
| Wall Sensors | 3–5x IR (Sharp/analog) or VL53L0X ToF | Front + front-diagonal + side pairs |
| IMU (optional) | MPU6050 | Yaw drift correction during turns |
| Power | 1S/2S LiPo + buck regulator to 3.3V/5V rail | Black Pill needs clean 5V on VBUS or 3.3V direct |

## Phase Map
1. **[Phase 1 — STM32 Ecosystem & Bare-Metal Basics](01-phase1-baremetal.md)**
   Toolchain, clocks, GPIO, EXTI, SysTick, hardware timers.
2. **[Phase 2 — Sensor & Actuator Interfacing](02-phase2-sensors-actuators.md)**
   PWM motor control, quadrature encoders, ADC, I2C/SPI (MPU6050, VL53L0X).
3. **[Phase 3 — Kinematics & Control Systems](03-phase3-kinematics-control.md)**
   Dead reckoning, PID for straights/turns, sensor fusion.
4. **[Phase 4 — Maze Solving Algorithms](04-phase4-maze-solving.md)**
   16×16 maze modeling, Flood Fill, path→motion translation.
5. **[Phase 5 — Competition Optimization](05-phase5-competition-optimization.md)**
   Search-run/fast-run state machine, speed tuning, robustness, debugging.

## How We'll Work Through This
For each phase I'll:
- Explain the **theory** (control theory / DSA / hardware) at the depth your
  background supports — no hand-holding on basic C++ or algorithms, but full
  rigor on the embedded-specific and control-theory parts.
- Give you **working HAL-based code** with register-level explanations of what
  HAL is doing underneath, so you're never treating it as magic.
- Give you a **checkpoint task** at the end of each section. Tell me your
  answer/result and I'll correct or advance you.

Reply "start phase 1" (or ask a question about the overview) and we'll begin.
