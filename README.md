<p align="center">
  <h1 align="center">🐭 Maze Runner — Autonomous Micromouse</h1>
  <p align="center">
    <strong>A championship-grade autonomous maze-solving robot built for MicroMaze 3 (IIT IEEE RAS)</strong>
  </p>
  <p align="center">
    <img src="https://img.shields.io/badge/MCU-STM32F401-blue?style=flat-square&logo=stmicroelectronics" alt="STM32"/>
    <img src="https://img.shields.io/badge/Language-C%20%7C%20C++-00599C?style=flat-square&logo=c" alt="C"/>
    <img src="https://img.shields.io/badge/IDE-Arduino%20%2B%20GCC-green?style=flat-square" alt="IDE"/>
    <img src="https://img.shields.io/badge/Competition-MicroMaze%203-red?style=flat-square" alt="Competition"/>
    <img src="https://img.shields.io/badge/Status-In%20Development-orange?style=flat-square" alt="Status"/>
  </p>
</p>

---

## 📖 Overview

**Maze Runner** is an autonomous micromouse robot designed to navigate and solve a standard IEEE 16×16 competition maze. The robot explores an unknown maze, maps its walls using onboard sensors, computes the optimal path using advanced graph algorithms, and then executes a blazing-fast speed run — all without any human intervention.

Built for the **MicroMaze 3** competition organized by the Informatics Institute of Technology (IIT) IEEE Robotics & Automation Society, with a **final competition date of August 22, 2026**.

### 🏆 Competition Details

| Detail | Info |
|---|---|
| **Event** | MicroMaze 3 — Inter-University Micromouse Competition |
| **Organizer** | IIT IEEE Student Branch (IEEE RAS Student Chapter) |
| **Maze** | 16×16 grid, 18cm × 18cm cells, 5cm walls, 1.2cm thick |
| **Max Robot Size** | 14.5cm × 14.5cm (no height limit) |
| **Trial Time** | 8 minutes per round, up to 10 runs in finals |
| **Prize Pool** | 1st: 100,000 LKR · 2nd: 70,000 LKR · 3rd: 40,000 LKR |

---

## 🏗️ System Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                    MAZE RUNNER — SOFTWARE STACK               │
├──────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │  Tier 1: SEARCH RUN                                     │ │
│  │  ┌─────────────────────────────────────────────┐        │ │
│  │  │  🌊 Modified Flood Fill (BFS)                │        │ │
│  │  │  • Multi-source BFS from goal cells         │        │ │
│  │  │  • Straight-preference tie-breaking         │        │ │
│  │  │  • Recomputes per cell on new wall data     │        │ │
│  │  └─────────────────────────────────────────────┘        │ │
│  └─────────────────────────────────────────────────────────┘ │
│                          ▼                                   │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │  Tier 2: FAST-RUN PATH PLANNING                         │ │
│  │  ┌─────────────────────────────────────────────┐        │ │
│  │  │  🔬 Weighted Dijkstra (3D State Space)       │        │ │
│  │  │  • State = (x, y, heading) → 1024 states    │        │ │
│  │  │  • Turn penalties: 90° = +12, 180° = +30    │        │ │
│  │  │  • Binary min-heap priority queue            │        │ │
│  │  └─────────────────────────────────────────────┘        │ │
│  └─────────────────────────────────────────────────────────┘ │
│                          ▼                                   │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │  Tier 3: MOTION OPTIMIZATION                             │ │
│  │  ┌─────────────────────────────────────────────┐        │ │
│  │  │  🎯 Path Smoother (Look-Ahead)               │        │ │
│  │  │  • Straight-run merging                     │        │ │
│  │  │  • Rolling turn classification              │        │ │
│  │  │  • Diagonal detection (optional)            │        │ │
│  │  └─────────────────────────────────────────────┘        │ │
│  └─────────────────────────────────────────────────────────┘ │
│                          ▼                                   │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │  Tier 4: VELOCITY PROFILE GENERATION                     │ │
│  │  ┌─────────────────────────────────────────────┐        │ │
│  │  │  🏎️ S-Curve & Rolling Turn Profiles          │        │ │
│  │  │  • Jerk-limited S-curve acceleration        │        │ │
│  │  │  • Circular arc turn profiles               │        │ │
│  │  │  • Differential wheel speed computation     │        │ │
│  │  └─────────────────────────────────────────────┘        │ │
│  └─────────────────────────────────────────────────────────┘ │
│                          ▼                                   │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │  LOCOMOTION LAYER                                        │ │
│  │  PID Control → TB6612FNG Motor Driver → N20 Motors      │ │
│  └─────────────────────────────────────────────────────────┘ │
└──────────────────────────────────────────────────────────────┘
```

---

## ⚙️ Hardware Platform

| Component | Part | Specification |
|---|---|---|
| **MCU** | STM32F401CCU6 "Black Pill" | 84 MHz ARM Cortex-M4F, 256KB Flash, 64KB RAM |
| **Motor Driver** | TB6612FNG | Dual H-bridge, 1.2A continuous per channel |
| **Motors** | N20 micro gearmotors | 300 RPM with magnetic quadrature encoders |
| **Wall Sensors** | VL53L0X (ToF) | 3–5× I2C Time-of-Flight distance sensors |
| **IMU** | MPU6050 | 6-axis accelerometer + gyroscope for yaw correction |
| **Power** | 2× 3.7V LiPo | 7.4V total, buck-regulated to 3.3V/5V rails |
| **IDE** | Arduino IDE + GCC | STM32duino core for HAL, GCC for algorithm testing |

### 📌 Pin Assignments

| Function | Pin(s) | Peripheral |
|---|---|---|
| Motor A PWM | `PA8` | TIM1 CH1 |
| Motor A Direction | `PB12`, `PB13` | GPIO |
| Motor B PWM | `PA9` | TIM1 CH2 |
| Motor B Direction | `PB15`, `PA10` | GPIO |
| Motor Driver Standby | `PB14` | GPIO |
| Left Encoder (A/B) | `PA0`, `PA1` | TIM2 CH1/CH2 (Encoder Mode 3) |
| Right Encoder (A/B) | `PA6`, `PA7` | TIM3 CH1/CH2 (Encoder Mode 3) |
| VL53L0X XSHUT | `PA4` | GPIO |
| I2C (IMU + ToF) | `PB8` (SCL), `PB9` (SDA) | I2C1 |

---

## 📂 Project Structure

```
Maze-Runner/
│
├── src/                            # Core algorithm suite (pure C, desktop-testable)
│   ├── config.h                    # All tunable parameters (speeds, costs, geometry)
│   ├── maze.h                      # Maze data structures, wall operations, direction system
│   ├── flood_fill.h / .c           # Tier 1: Modified Flood Fill (BFS) for search run
│   ├── dijkstra_weighted.h / .c    # Tier 2: Weighted Dijkstra with turn penalties
│   ├── path_smoother.h / .c        # Tier 3: Waypoint → motion command converter
│   ├── motion_profile.h / .c       # Tier 4: S-curve & arc turn velocity profiles
│   ├── solver.h / .c               # Top-level orchestrator (ties all tiers together)
│   ├── main.c                      # Desktop test harness & algorithm demo
│   ├── maze_test.exe               # Compiled test binary (Windows)
│   └── worksheet.md                # Algorithm deep-dive & developer manual
│
├── Testing Codes/                  # Hardware component validation sketches (Arduino)
│   ├── MPU6050/MPU6050.ino         # IMU calibration, complementary filter, yaw tracking
│   ├── VL53L0X/VL53L0X.ino        # ToF sensor register-level distance reading
│   ├── Motors/Motors.ino           # Basic motor forward/reverse/stop test
│   ├── Motors (Advanced)/          # Advanced motor control test
│   └── Motors_With_Encoders/       # Quadrature encoder (register-level TIM2/TIM3)
│       └── Motors_With_Encoders.ino
│
├── bin/                            # Learning curriculum (5-phase embedded track)
│   ├── 00-README.md                # Curriculum overview & phase map
│   ├── 01-phase1-baremetal.md      # STM32 toolchain, clocks, GPIO, timers
│   ├── 02-phase2-sensors-actuators.md  # PWM, encoders, ADC, I2C/SPI
│   ├── 03-phase3-kinematics-control.md # Dead reckoning, PID, sensor fusion
│   ├── 04-phase4-maze-solving.md   # Maze modeling, flood fill, path translation
│   └── 05-phase5-competition-optimization.md  # State machine, speed tuning
│
├── plan/                           # Build guide & competition documentation
│   ├── 00_INDEX.md                 # Master document index & build order
│   ├── 01_HARDWARE_OVERVIEW.md     # Component roles & specifications
│   ├── 02_ADDITIONAL_COMPONENTS.md # Bill of materials & procurement list
│   ├── 03_PIN_DIAGRAM_WIRING.md    # Full wiring tables & connection diagrams
│   ├── 04_PCB_PLACEMENT_THERMAL.md # Physical layout & thermal management
│   ├── 05_TESTING_STAGES.md        # Step-by-step hardware testing procedures
│   ├── 06_MAIN_LOGIC_CODE.md       # Maze-solving logic & final firmware code
│   ├── 07_TUNING_CALIBRATION.md    # PID tuning & sensor calibration guide
│   ├── 08_TROUBLESHOOTING.md       # Common problems & fixes
│   ├── 09_SPEED_RUN.md             # Speed run strategy & velocity profiling
│   ├── 10_COMPETITION_PREP.md      # Competition rules & checklists
│   ├── 11_DATA_LOGGING.md          # Telemetry, Bluetooth, Python visualization
│   └── MicroMaze_3_Delegate_Booklet.md  # Official competition rulebook
│
└── README.md                       # ← You are here
```

---

## 🧠 Algorithm Suite

### Tier 1 — Modified Flood Fill (Search Run)
The robot explores the unknown maze using a **multi-source BFS** flood fill. At each cell, it reads wall data from sensors, updates the internal maze map, and recomputes shortest distances to the goal. A critical **straight-preference tie-breaker** avoids unnecessary turns during exploration.

- **Complexity:** O(256) per recompute — executes in <10µs on Cortex-M4 @ 100MHz
- **Queue:** Fixed-size ring buffer (zero heap allocation)

### Tier 2 — Weighted Dijkstra (Fast-Run Path)
After exploration, the robot computes a **time-optimal** path using Dijkstra's algorithm on a 3D state space `(x, y, heading)` = 1,024 states. Turn penalties make it prefer straighter paths even if slightly longer in cell count.

- **Cost Model:** `STRAIGHT = 10` | `+TURN_90 = 12` | `+TURN_180 = 30`
- **Priority Queue:** Custom binary min-heap (executes in <2ms on STM32)

### Tier 3 — Path Smoother (Motion Commands)
Raw waypoints are converted into optimized motor commands:
- Consecutive same-heading moves → single `STRAIGHT(n)` command
- Heading changes with entry/exit momentum → `SMOOTH_ROLLING_TURN` (no stop)
- Dead-end turns → `IN_PLACE_TURN` (stop, rotate, resume)

### Tier 4 — S-Curve Motion Profile
Velocity targets are computed using **jerk-limited S-curve profiles** to prevent wheel slip:
- Sinusoidal acceleration ramps: `v = v_start + (v_peak - v_start) × sin²(progress)`
- Rolling arc turn profiles with differential wheel speed computation
- **Look-ahead planning**: pre-computes deceleration for upcoming turns

---

## 🔧 Memory Footprint

The entire algorithm suite uses **zero dynamic memory allocation** — all structures are fixed-size and statically allocated, critical for reliability on embedded systems.

| Structure | Size | Notes |
|---|---|---|
| `MazeMap` (16×16 cells) | ~1,280 bytes | 5 bytes per cell × 256 cells |
| `DijkstraResult` | ~1,030 bytes | 256 waypoints + metadata |
| `SmoothedPath` | ~640 bytes | 128 motion commands |
| `Solver` (complete state) | ~3,200 bytes | Includes maze + paths + commands |
| **Total Algorithm RAM** | **< 4 KB** | Out of 128 KB available |

---

## 📊 Current Progress

### Development Phases

| Phase | Description | Status |
|:---:|---|:---:|
| 1 | **STM32 Bare-Metal Fundamentals** — Toolchain, clocks, GPIO, timers | ✅ Complete |
| 2 | **Sensor & Actuator Interfacing** — Motors, encoders, VL53L0X, MPU6050 | ✅ Complete |
| 3 | **Kinematics & Control** — Dead reckoning, PID, sensor fusion | 🔧 In Progress |
| 4 | **Maze-Solving Algorithms** — Flood fill, Dijkstra, path smoother | ✅ Complete |
| 5 | **Competition Optimization** — State machine, speed tuning, robustness | 🔲 Pending |

### Software Modules

| Module | File(s) | Status | Notes |
|---|---|:---:|---|
| Maze Data Structures | `maze.h`, `config.h` | ✅ Done | Zero-alloc, embedded-safe |
| Modified Flood Fill | `flood_fill.c/.h` | ✅ Done | BFS + straight-preference tie-break |
| Weighted Dijkstra | `dijkstra_weighted.c/.h` | ✅ Done | 3D state space, binary min-heap |
| Path Smoother | `path_smoother.c/.h` | ✅ Done | Straight merging, rolling turn classification |
| Motion Profile | `motion_profile.c/.h` | ✅ Done | S-curve, arc turns, wheel speed diff |
| Solver Orchestrator | `solver.c/.h` | ✅ Done | Ties all tiers together |
| Desktop Test Harness | `main.c` | ✅ Done | Full algorithm demo on simulated maze |
| PID Controller | — | 🔧 WIP | Needed for motor control integration |
| STM32 Main Firmware | — | 🔲 Pending | Final firmware integrating all modules |

### Hardware Testing

| Component | Test Sketch | Status | Notes |
|---|---|:---:|---|
| Motor Basic Control | `Motors.ino` | ✅ Tested | Forward/reverse/stop via TB6612FNG |
| Motor Advanced | `Motors (Advanced).ino` | ✅ Tested | Extended motor control |
| Quadrature Encoders | `Motors_With_Encoders.ino` | ✅ Tested | Register-level TIM2/TIM3, 0.178 mm/count |
| VL53L0X ToF Sensor | `VL53L0X.ino` | ✅ Tested | Register-level I2C, single-shot ranging |
| MPU6050 IMU | `MPU6050.ino` | ✅ Tested | Calibration, complementary filter, yaw |

---

## 🚀 Quick Start

### Build & Run the Algorithm Test Suite (Desktop)

```bash
# Compile the desktop test harness
cd src/
gcc -o maze_test main.c flood_fill.c dijkstra_weighted.c path_smoother.c solver.c motion_profile.c -lm

# Run the full algorithm demo
./maze_test
```

This runs a complete simulation on a hardcoded 16×16 competition-style maze:
1. **Flood Fill** — Computes BFS distance heatmap
2. **BFS Path** — Traces shortest distance path
3. **Dijkstra Path** — Computes time-optimal path with turn penalties
4. **Path Smoother** — Generates optimized motion commands
5. **Search Simulation** — Simulates exploring an unknown maze
6. **Motion Profiles** — Demos S-curve and rolling turn calculations
7. **Comparison** — Side-by-side BFS vs Dijkstra metrics

### Flash Hardware Test Sketches (Arduino IDE)

1. Install [STM32duino board package](https://github.com/stm32duino/Arduino_Core_STM32)
2. Select board: **Generic STM32F4 → BlackPill F411CE**
3. Open any sketch from `Testing Codes/` and upload

---

## 🗓️ Competition Timeline

| Date | Milestone |
|---|---|
| June 20, 2026 | Registrations Open |
| June 30, 2026 | Registrations Close |
| July 1, 2026 | Induction Session |
| July 18, 2026 | Workshop 1: STM32 Programming |
| August 1, 2026 | Workshop 2: Sensors & Integration |
| August 10, 2026 | Competition Guidelines Released |
| August 14–15, 2026 | Free Practice Runs |
| **August 22, 2026** | **🏁 Final Competition Day** |

---

## 📚 Documentation

| Document | Description |
|---|---|
| [`src/worksheet.md`](src/worksheet.md) | Algorithm deep-dive with math, code breakdowns, and architecture flowcharts |
| [`plan/00_INDEX.md`](plan/00_INDEX.md) | Master build guide index with time estimates |
| [`bin/00-README.md`](bin/00-README.md) | 5-phase learning curriculum from beginner to expert |
| [`plan/MicroMaze_3_Delegate_Booklet.md`](plan/MicroMaze_3_Delegate_Booklet.md) | Official competition rules & requirements |

---

## 🧰 Tuning Parameters

All competition-critical parameters are centralized in [`src/config.h`](src/config.h):

```c
/* Motion Speeds */
SEARCH_MAX_SPEED_MM_S    300.0f    // Conservative for sensor accuracy
FAST_MAX_SPEED_MM_S      800.0f    // Aggressive for speed runs

/* Dijkstra Cost Model */
COST_STRAIGHT             10       // Base cost per cell
COST_TURN_90              12       // Penalty for 90° turn
COST_TURN_180             30       // Penalty for 180° turn

/* S-Curve Profile */
JERK_LIMIT_MM_S3        8000.0f    // Higher = snappier, lower = smoother
ENABLE_S_CURVE             1       // Set to 0 for basic trapezoidal

/* Turn Geometry */
TURN_RADIUS_90_MM        45.0f     // Rolling turn radius (90°)
MAX_TURN_SPEED_MM_S     300.0f     // Linear speed during arc turns
```

---

## 🤝 Team

Built by UOM Efac students for the **MicroMaze 3** inter-university competition.

---

## 📄 License

This project is developed for educational and competition purposes.

---

<p align="center">
  <strong>🏁 SOLVING FOR SPEED — MICROMAZE 3 🏁</strong>
</p>
