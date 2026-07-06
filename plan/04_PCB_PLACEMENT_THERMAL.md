# 04 — PCB Layout, Physical Placement & Thermal Management

---

## Robot Physical Dimensions

### Target Size (IEEE Micromouse rules)
- Robot must fit in **25cm × 25cm** footprint (half-size maze: 12cm × 12cm)
- **Recommended robot size:** 90mm × 90mm × 60mm (W × L × H)
- Lower = better (lower center of gravity)
- Wheel track: ~70mm center-to-center

---

## Layer/Stack Layout (Bottom to Top)

```
────────────────────────
[Top]     Sensors (VL53L0X pointing outward at correct angles)
          + STM32 Board
          + MPU6050 (flat on board, centered)
────────────────────────
[Middle]  TB6612FNG Motor Driver
          + Voltage Regulators (buck + LDO)
          + Decoupling capacitors
────────────────────────
[Bottom]  Battery (2× LiPo flat cells)
          + Motors (N20 at rear, or differential drive center)
────────────────────────
      WHEELS (sides)
```

**Why this stack order?**
- Battery lowest → low center of gravity, less tipping
- Sensors top → clear line of sight to walls
- Driver middle → short motor wires, away from sensors
- MPU6050 center → accurate rotation measurement (at robot's pivot point)

---

## PCB Layer Recommendations

### If designing custom PCB:
```
Layer 1 (Top copper):    Signal traces (I2C, GPIO, PWM, encoder)
Layer 2 (Ground plane):  Solid GND pour — critical for noise reduction
Layer 3 (Power plane):   3.3V and 5V pours (if 4-layer board)
Layer 4 (Bottom copper): Motor power traces (thick: 1mm+)
```

### For 2-layer PCB (minimum viable):
- Top: Signals + 3.3V
- Bottom: GND plane + 7.4V/5V power rails

### Recommended PCB Design Tools:

| Tool | Cost | Best For |
|------|------|----------|
| **KiCad 8** | Free, open-source | Full-featured, industry standard |
| **EasyEDA** | Free, web-based | Quick designs, JLCPCB integration |
| **Altium CircuitMaker** | Free | Team projects, cloud-based |

> [!TIP]
> EasyEDA has a built-in **JLCPCB parts library** — every component has a footprint and 3D model ready. Fastest path from schematic to fabricated PCB.

---

## Component Placement Rules

### STM32 Board
- Mount on center-top using 2.5mm brass standoffs (4–6mm height)
- Ensure USB port is accessible from rear for reflashing
- Keep away from motor driver (EM noise)

### MPU6050
- **MUST be at the robot's center of rotation** (between wheels axle line)
- Mount flat — even 1° tilt causes constant Z-drift
- No vibrating components directly below it
- Isolation: mount with soft foam tape, not rigid standoffs
  - **Recommended:** 2mm EVA foam pad + 3M VHB double-sided tape
  - Cut foam to exact MPU6050 breakout board size
  - This absorbs motor vibration that causes gyro noise
- Keep away from motors (magnetic field interference)
- Keep 10mm+ away from buck converter (EMI from switching inductor)

### VL53L0X Sensors
```
Mounting method for angled sensors:
- 3D-printed or laser-cut angle bracket
- Or: angle PCB footprint at 45°/90° during PCB design
- Sensor face must be flush with chassis edge or slightly recessed
- Cover lens opening — dust causes false readings
```

**Height:** Mount sensors at half the maze-wall height:
- Maze wall = 50mm tall
- Mount sensor at ~25–35mm from floor
- Angle: 0° (horizontal), pointing straight at wall face

### TB6612FNG Motor Driver
- Keep close to motors — shorter motor wires = less inductance noise
- Place 100µF caps on VM pin directly on pads
- Keep PWM signal traces away from I2C lines
- Good airflow — this chip gets warm at 1A+

### Battery
- Center of robot footprint or slightly rear
- Use velcro strap or snap-fit holder (never hot glue alone)
- Leave connector accessible (XT30 or JST connector)
- Keep flat (LiPo cells must never bend)

### Voltage Regulators
- Buck converter (MP1584): Place with inductor away from I2C lines
  - Inductors radiate EMI — keep 10mm+ from MPU6050 and I2C bus
- AMS1117 LDO: Gets warm (up to 40°C) at full load
  - Minimum 5mm clearance from other components
  - Ensure air can flow over it

### Motor Placement — Front vs Rear Drive

| Config | Pros | Cons |
|--------|------|------|
| **Rear drive + front caster** | Battery weight over drive wheels = better traction | Caster can catch on maze floor joints |
| **Front drive + rear caster** | Sensors closer to walls (more accurate) | Less traction (battery weight on caster) |
| **Center drive + front caster** | Best balance, shortest turning radius | More complex chassis design |

> **Recommended: Rear drive** — simpler to build, and placing the heavy battery over the drive wheels maximizes traction.

---

## Ground Plane & EMI Strategy

> [!IMPORTANT]
> Motor PWM switching is the #1 source of electrical noise in a micromouse. Poor grounding = sensors crash when motors run.

### Rules:
1. **Solid ground plane** on one PCB layer — never break it with signal traces
2. **Star ground** for power: all grounds return to one point near battery connector
3. **Keep analog/sensor traces away from motor traces** — minimum 5mm gap
4. **Separate ground regions** (if possible): digital ground, analog ground, motor ground — connected at one star point
5. **Ferrite beads** on motor power lines (optional but effective at blocking high-frequency noise)

### I2C Noise Immunity:
```
I2C traces:
- Route as differential pair (SDA and SCL parallel, close together)
- Keep traces under 50mm total length
- If using wires (not PCB traces): twist SDA+SCL pair together
- 100nF cap on each sensor VCC pin, as close to sensor as possible
```

---

## Thermal Management

### Heat Sources & Expected Temperatures

| Component | Expected Temp | Concern Level |
|-----------|---------------|---------------|
| TB6612FNG | 40–70°C | ⚠️ Medium — heat spreads to PCB |
| AMS1117 LDO | 35–55°C | ⚠️ Medium at high current |
| MP1584 Buck | 35–50°C | 🟢 Low — efficient converter |
| STM32 MCU | 30–45°C | 🟢 Low |
| N20 Motors | 35–60°C | ⚠️ Stall = very hot fast |

### Rules:
1. **TB6612FNG:** Copper pour under chip (exposed pad must solder to PCB thermal pad). If hot at sustained 1A, add small heatsink or increase copper area.
2. **Motors:** Never stall motors for more than 2 seconds — they will overheat. Always have stall detection in firmware.
3. **LiPo Battery:** Never let cell voltage drop below 3.0V/cell (6.0V total). Thermal runaway risk if overdischarged then charged.
4. **LDO Regulator:** If powering 5× VL53L0X from 3.3V LDO, calculate: 5 × 19mA = 95mA. Power dissipated = (5V–3.3V) × 0.095A = **0.16W** → manageable, but check LDO touch temp.

### Thermal Relief:
- Avoid enclosing robot in sealed box — some airflow needed
- If using tight enclosure, add 2×2mm vent holes near driver and regulator
- During long testing sessions: place on open surface, not foam

---

## Wiring Routing Rules

```
KEEP APART:                          KEEP TOGETHER:
Motor wires  ───────────────────    Motor + (AO1/AO2) wires — twisted pair
Power supply wires                  I2C lines (SDA+SCL) — parallel, short
                                    Encoder wires — twisted pair if >10cm

AVOID:
- I2C traces next to PWM traces
- Encoder signal wires next to motor wires
- Sensor VDD lines near high-current paths
- Long wire loops (act as antennas for noise)
```

### Wire gauge guide:

| Connection | Gauge |
|------------|-------|
| Battery to driver (VM) | 24 AWG min, 22 AWG better |
| Motor outputs | 24 AWG |
| Logic signals (PWM, I2C) | 28–30 AWG |
| Sensor power | 28 AWG |

---

## 3D Model / Footprint Sketch

```
Top View (90mm × 90mm):

┌──────────────────────────┐
│  [VL-L45]  [VL-F]  [VL-R45]  │  ← Sensors at front edge
│                          │
│  ┌──────────────────┐   │
│  │   STM32F411CEU6   │   │
│  └──────────────────┘   │
│  ┌──────┐  ┌────────┐   │
│  │MPU60 │  │TB6612  │   │
│  └──────┘  └────────┘   │
│  [L-Motor]    [R-Motor]  │  ← Motors at sides/rear
│         [BATTERY]        │
└──────────────────────────┘
        ↑        ↑
    Left Wheel   Right Wheel
```

---

## Pre-Run Physical Checklist

Before every maze run:
- [ ] Wheels tight on motor shaft (no slip)
- [ ] All sensor brackets secure
- [ ] Battery fully charged (8.2–8.4V for 2S LiPo)
- [ ] Battery strapped down firmly
- [ ] No loose wires that could snag on maze walls
- [ ] MPU6050 mounting hasn't shifted
- [ ] All screws tight (vibration loosens them)
