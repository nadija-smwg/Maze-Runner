# 02 — Additional Components You MUST Have
## Parts Missing from Your Current List

---

## 🔴 Critical Missing Parts

### 1. Voltage Regulators (REQUIRED)
Your 7.4V battery cannot power STM32 or sensors directly.

| Part | Converts | Supplies | Why |
|------|----------|----------|-----|
| **MP1584 or LM2596 Buck Module** | 7.4V → 5V | STM32 VIN | Efficient step-down |
| **AMS1117-3.3 LDO** | 5V → 3.3V | VL53L0X, MPU6050 | Low dropout, 800mA |

> [!TIP]
> Alternatively: Use a single **MP2307** or **XL4005** buck converter set to 5V, and use STM32's onboard 3.3V regulator for sensors (max ~150mA — check if enough for 5 VL53L0X + MPU6050).

**Total 3.3V current needed:**
- 5× VL53L0X: 5 × 19mA = 95mA
- 1× MPU6050: 3.9mA
- **Total: ~100mA** → STM32 onboard 3.3V regulator is okay if < 150mA limit

---

### 2. Decoupling Capacitors (REQUIRED)
Without these, your motors will cause noise that crashes sensors.

| Value | Qty | Placement |
|-------|-----|-----------|
| 100µF electrolytic | 2 | Across motor power supply (VM pin of TB6612FNG) |
| 100nF ceramic (0.1µF) | 6 | Across each IC's VCC/GND pins |
| 10µF electrolytic | 2 | Across 5V and 3.3V regulator outputs |

---

### 3. Wheels + Chassis (REQUIRED)
- **Wheels:** 30–40mm diameter rubber wheels for N20 motors (check your motor shaft = 3mm D-shaft)
- **Chassis:** Custom PCB chassis OR laser-cut acrylic 100×100mm
- **Motor mounts:** Brass standoffs or N20 motor brackets

---

### 4. Ball Caster (REQUIRED)
- **10mm metal or nylon ball caster** — provides 3rd contact point for stability
- Without this, your robot will tip forward/backward
- Alternative: PTFE (Teflon) skid pad for lighter weight

---

### 5. Connectors (REQUIRED)
- **XT30 connector pair** — for battery connection (rated 30A, compact)
- **JST-PH 2.0mm connectors** — for motor and sensor harnesses (easy disconnect)
- **2.54mm pin headers + female sockets** — for STM32 breadboard/PCB mounting

> [!TIP]
> Use connectors everywhere — never solder wires directly. This lets you swap components without desoldering.

---

### 6. XSHUT Control Resistors (REQUIRED for VL53L0X)
- 5× **10kΩ resistors** as pull-ups on XSHUT lines (or use internal STM32 pull-ups)

---

### 7. ST-Link V2 Programmer (REQUIRED)
- Arduino IDE cannot upload to STM32 via USB without a bootloader
- **ST-Link V2** (~$2 clone) is essential for flashing
- Or use: USB-to-Serial + STM32 BOOT0 bootloader method

---

### 8. Logic Level Shifter (SOMETIMES NEEDED)
- VL53L0X is 2.8V–3.3V I2C
- STM32 I2C is 3.3V
- **Usually compatible directly** — but if sensors glitch, add a level shifter
- Recommended: **BSS138-based bidirectional I2C level shifter**

---

## 🟡 Strongly Recommended Parts

### 9. LiPo Battery Protection/BMS
- 2S BMS module with overcharge, overdischarge, and short circuit protection
- Protects your cells from damage
- Includes balance charging port

### 10. LiPo Charger
- **TP4056-based 2S charger** or **B3AC compact balance charger**
- Never charge LiPo without a proper charger — fire hazard!

### 11. Power Switch + LED Indicator
- SPDT or DPDT toggle switch (2A rated)
- Red LED + 330Ω resistor for power indicator
- Green LED + 330Ω for status/maze-running indicator

### 12. Buttons
- **2× tactile push buttons**:
  - Button 1: Start/stop maze run
  - Button 2: Reset / return to start

### 13. Buzzer (Optional but very useful)
- Passive buzzer for audio feedback
- Useful for: startup confirmation, wall detection debugging, maze complete signal

### 14. OLED Display (Optional)
- **0.96" SSD1306 I2C OLED**
- Shows: cell position, heading, sensor values during debug
- Very useful during development

---

## 🟢 Nice to Have

### 15. Logic Analyzer
- **8-channel logic analyzer** (~$5 clone)
- Essential for debugging I2C, encoder signals, PWM
- Works with PulseView (free software)

### 16. Multimeter
- For voltage checking, continuity testing

### 17. Hot Glue / Kapton Tape
- Secure sensors at correct angles
- Insulate bare solder joints

### 18. HC-05 Bluetooth Module (for wireless debugging)
- Stream sensor data and PID values to laptop in real-time
- Remotely start/stop runs
- ~$2–3 cost, massive time savings during tuning

---

## 📦 Full Shopping List Summary

```
CRITICAL:
[ ] MP1584 buck converter (5V output)          Search: "MP1584 adjustable buck converter"
[ ] AMS1117-3.3 LDO regulator                  Search: "AMS1117 3.3V module"
[ ] 100µF capacitors ×4                        Search: "100uF 16V electrolytic capacitor"
[ ] 100nF capacitors ×10                       Search: "100nF ceramic capacitor 0.1uF"
[ ] Rubber wheels 34mm ×2 (fits 3mm D-shaft)   Search: "N20 motor rubber wheel 34mm"
[ ] N20 motor brackets ×2                      Search: "N20 micro motor bracket mount"
[ ] Ball caster 10mm ×1                        Search: "10mm metal ball caster module"
[ ] ST-Link V2 programmer                      Search: "ST-Link V2 STM32 programmer clone"
[ ] 10kΩ resistors ×10                         Search: "10K ohm resistor 1/4W"
[ ] XT30 connector pair ×2                     Search: "XT30 male female connector"
[ ] JST-PH 2.0mm connectors ×10               Search: "JST PH 2.0 connector kit"

STRONGLY RECOMMENDED:
[ ] 2S LiPo BMS protection board               Search: "2S 7.4V BMS protection board"
[ ] 2S LiPo balance charger                    Search: "B3AC 2S 3S LiPo balance charger"
[ ] Power switch (2A)                          Search: "SPDT toggle switch 2A"
[ ] Tactile push buttons ×2                    Search: "6x6mm tactile push button"
[ ] Status LEDs ×2 + 330Ω resistors            Search: "3mm LED assortment + 330 ohm"

OPTIONAL BUT HELPFUL:
[ ] SSD1306 OLED 0.96"                         Search: "SSD1306 I2C OLED 0.96 inch"
[ ] Passive buzzer                             Search: "passive buzzer module 3.3V"
[ ] USB Logic Analyzer                         Search: "8 channel USB logic analyzer"
[ ] HC-05 Bluetooth module                     Search: "HC-05 Bluetooth serial module"
```

---

## 🏭 PCB Fabrication (When Ready)

When you're done prototyping on breadboard and ready for a proper PCB:

| Service | Min Order | Cost | Turnaround |
|---------|-----------|------|------------|
| **JLCPCB** | 5 boards | $2–5 + $3 shipping | 5–7 days |
| **PCBWay** | 5 boards | $5–10 + $5 shipping | 5–7 days |
| **OSH Park** | 3 boards | $5/sq inch | 7–14 days |

**Recommended workflow:**
1. Design PCB in **KiCad** (free) or **EasyEDA** (free, web-based)
2. Export Gerber files
3. Upload to JLCPCB → order 5 boards (you'll want spares)
4. Optional: add SMD assembly service for $8–15 extra

---

## 🖨️ 3D Printed Parts

If you have access to a 3D printer (or use an online service like JLCPCB 3D printing):

| Part | Material | Purpose |
|------|----------|---------|
| Sensor brackets (45°/90°) | PLA | Mount VL53L0X at correct angles |
| Motor mounts | PLA/PETG | Secure N20 motors to chassis |
| Battery holder | PLA | Snap-fit holder for LiPo cells |
| Wheel adapter | PLA | If wheels don't fit motor shaft |
| Full chassis | PLA/PETG | Alternative to PCB-as-chassis |

> [!TIP]
> Design in **Fusion 360** (free for personal use) or **TinkerCAD** (web-based, simpler). Export as STL.

---

## 💰 Estimated Total Cost (Components Only)

| Category | Approx Cost (USD) |
|----------|-------------------|
| Your existing parts | ~$25 |
| Critical additions | ~$15 |
| Strongly recommended | ~$10 |
| Optional | ~$8 |
| PCB fabrication (5 boards) | ~$5 |
| **Total** | **~$63** |
