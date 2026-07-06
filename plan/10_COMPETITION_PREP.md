# 10 — Competition Preparation
## IEEE/APEC Micromouse Competition Readiness Guide

---

## Competition Rules Summary (IEEE Standard)

### Maze Specifications:
| Parameter | Value |
|-----------|-------|
| Maze size | 16×16 cells |
| Cell size | 18cm × 18cm |
| Wall height | 5cm |
| Wall thickness | 1.2cm |
| Floor | Smooth, flat, light-colored |
| Start position | Corner cell (0,0) |
| Goal | Center 4 cells (7,7), (7,8), (8,7), (8,8) |

### Robot Requirements:
| Requirement | Limit |
|-------------|-------|
| Maximum footprint | 25cm × 25cm |
| Maximum height | No limit (but lower is better) |
| Weight | No limit (but lighter is better) |
| Autonomous | Must be fully autonomous — no remote control |
| Power | Self-contained battery, no external power |

### Run Rules:
- Typically **3–5 runs** allowed
- **Best time** counts (not average)
- Time starts when robot crosses start line
- Time stops when robot enters any goal cell
- **Touching the robot** during a run = disqualification for that run
- Robot must not damage the maze

---

## Pre-Competition Hardware Checklist

### One Week Before:

- [ ] All soldering joints inspected for cold joints (reflow if dull/gray)
- [ ] Battery fully charged and holds charge for >30 min
- [ ] Spare battery prepared and charged
- [ ] All screws thread-locked (small drop of nail polish on screw threads)
- [ ] Wheels checked for wear — replace if tread is smooth
- [ ] Sensor lenses cleaned with microfiber cloth
- [ ] Motor shaft couplings tight (no wheel slip)
- [ ] Ball caster/skid clean and free-moving
- [ ] USB cable and ST-Link packed for emergency reflashing
- [ ] Laptop with Arduino IDE + all libraries configured

### Day Before:

- [ ] Full maze run test on practice maze
- [ ] Battery freshly charged (store at 3.8V/cell if not using immediately)
- [ ] Robot weight confirmed < 150g
- [ ] Robot fits in 25×25cm box
- [ ] PID gains verified and saved
- [ ] Speed run tested at 3 different speed levels
- [ ] Sensor calibration offsets recorded

### At the Competition:

- [ ] Power on and verify sensor readings look reasonable
- [ ] Run gyro calibration (keep still for 2 seconds)
- [ ] Check battery voltage > 7.8V (fully charged)
- [ ] Clear any saved maze from EEPROM (fresh exploration)
- [ ] Verify start button works
- [ ] Do a quick 2-cell test run in practice area (if available)

---

## Competition Strategy

### Run Allocation (5 runs typical):

| Run | Strategy | Speed | Purpose |
|-----|----------|-------|---------|
| 1 | **Explore + slow speed run** | Low (150 PWM) | Map the maze, get a safe time on the board |
| 2 | **Speed run from saved maze** | Medium (180 PWM) | Improve time with known path |
| 3 | **Fast speed run** | High (220 PWM) | Push for best time |
| 4 | **Maximum speed** | Max (240+ PWM) | Go for gold (risk of crash) |
| 5 | **Safety run** | Medium | Only if runs 3-4 crashed — get a clean time |

### Key Tactics:

1. **Always complete Run 1** — a slow, clean exploration + speed run is worth more than crashing at high speed
2. **Save maze to EEPROM** after Run 1 — subsequent runs can skip exploration
3. **Incremental speed** — increase speed by 10-15% per run, not 2×
4. **Know your robot's limits** — test maximum speed before competition day
5. **Battery management** — check voltage between runs, swap if below 7.4V

---

## Practice Maze Construction

### Materials:
- **Cardboard** or **foam board** (5mm thickness, 5cm tall strips)
- **Ruler and pencil** for marking grid
- **Hot glue gun** for attaching walls
- **Flat surface** (plywood, table, or smooth floor)

### Building a Practice Maze:

```
Minimum useful practice maze: 4×4 cells

Materials for 4×4 maze:
- Base board: 72cm × 72cm (4 × 18cm)
- Wall strips: 18cm long × 5cm tall
- Quantity: ~30 wall strips (enough for various configurations)

Steps:
1. Mark 18cm grid lines on base board
2. Cut wall strips: 18cm × 5cm × 0.5cm
3. Arrange walls in different patterns for testing
4. Use removable adhesive (putty/tape) so you can reconfigure
```

### Recommended Test Patterns:

```
Pattern 1: Straight corridor          Pattern 2: Turns
┌───┬───┬───┬───┐                    ┌───┬───┬───┬───┐
│   │   │   │ G │                    │       │       │
├   ┤   ┤   ┤   │                    │   ┌───┘   ┌───┤
│   │   │   │   │                    │   │       │ G │
├   ┤   ┤   ┤   │                    │   │   ┌───┤   │
│   │   │   │   │                    │   │   │       │
├   ┤   ┤   ┤   │                    │   └───┘   ┌───┤
│ S │   │   │   │                    │ S         │   │
└───┴───┴───┴───┘                    └───────────┴───┘
```

---

## Common Competition Pitfalls

| Pitfall | Prevention |
|---------|-----------|
| Battery dies mid-run | Check voltage before every run, carry spare |
| Robot crashes on first cell | Test in practice area first |
| Sensor misreads due to different wall color | Calibrate with actual competition walls (usually white) |
| Robot gets stuck in a loop | Ensure flood fill handles all edge cases — test with multiple maze configs |
| Floor surface is different (grip) | Bring spare wheels with different rubber hardness |
| I2C crash during run | Watchdog timer resets robot — at least robot doesn't freeze |
| Competition maze has more complex paths | Test with random maze generators online |
| Nerves cause misconfiguration | Use a pre-flight checklist (see above) |

---

## Score Optimization

Most competitions score based on:
```
Score = Exploration Time + Best Speed Run Time × Multiplier
```

To minimize score:
1. **Fast exploration** — don't explore the entire maze, just find the goal
2. **Efficient return** — straight-line return to start
3. **Multiple speed runs** — best time counts
4. **Path optimization** — shortest distance, not shortest cell count

> [!TIP]
> Some competitions give bonus points for:
> - Speed runs under a time threshold
> - Exploring the entire maze
> - Creative design or presentation

---

## Emergency Kit

Pack these for competition day:

```
[ ] Spare battery (charged)
[ ] ST-Link V2 programmer
[ ] USB cable (Micro-B or USB-C, match your board)
[ ] Laptop with Arduino IDE + all libraries
[ ] Small screwdriver set
[ ] Tweezers
[ ] Hot glue gun + glue sticks
[ ] Electrical tape
[ ] Spare wheels
[ ] Multimeter
[ ] Spare VL53L0X sensor (they break!)
[ ] Zip ties and double-sided tape
[ ] Cleaning cloth for sensor lenses
[ ] Printed copy of your pin diagram (03_PIN_DIAGRAM_WIRING.md)
```
