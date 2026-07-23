# 🐭 Micromouse Project — STM32 + Arduino IDE
## Complete Build & Programming Guide

> **Version:** 1.1 | **Last Updated:** 2026-06-14 | **Target:** IEEE/APEC Micromouse Competition

---

## 📁 Document Index

| File | Contents | Est. Time |
|------|----------|-----------|
| `01_HARDWARE_OVERVIEW.md` | Component roles, specs, what each part does | 30 min read |
| `02_ADDITIONAL_COMPONENTS.md` | Missing parts you MUST buy | 15 min read |
| `03_PIN_DIAGRAM_WIRING.md` | Full pin mapping, wiring tables, connection diagrams | 1–2 hours |
| `04_PCB_PLACEMENT_THERMAL.md` | Physical layout, heat management, placement rules | 1 hour |
| `05_TESTING_STAGES.md` | Step-by-step testing with code for each stage | 3–5 hours |
| `06_MAIN_LOGIC_CODE.md` | Full maze-solving logic, flood fill, PID, final code | 2–3 hours |
| `07_TUNING_CALIBRATION.md` | PID tuning, sensor calibration, motor tuning | 2–4 hours |
| `08_TROUBLESHOOTING.md` | Common problems and fixes | Reference |
| `09_SPEED_RUN.md` | Speed run strategy, path optimization, velocity profiling | 2–3 hours |
| `10_COMPETITION_PREP.md` | Competition rules, checklists, practice maze | 1 hour |
| `11_DATA_LOGGING.md` | Telemetry, Bluetooth, Python visualization | 1–2 hours |

---

## 🧠 Quick Summary

**Your Robot Stack:**
```
STM32F401CCU6 (BlackPill)
├── Sensors:    3–5× VL53L0X (ToF distance)  +  MPU6050 (IMU)
├── Motors:     2× N20 300RPM with encoders
├── Driver:     TB6612FNG dual motor driver
├── Power:      2× 3.7V LiPo (7.4V total)
└── IDE:        Arduino IDE with STM32duino core
```

**Maze:** Standard APEC/IEEE micromouse maze = 16×16 cells, each 18×18 cm with 12mm walls

---

## 🚀 Build Order (Recommended)

| Step | Task | Document | Est. Duration |
|------|------|----------|---------------|
| 1 | Buy missing parts | `02_ADDITIONAL_COMPONENTS.md` | 1–2 weeks (shipping) |
| 2 | Wire on breadboard | `03_PIN_DIAGRAM_WIRING.md` | 2–3 hours |
| 3 | Test each component | `05_TESTING_STAGES.md` (do in order!) | 3–5 hours |
| 4 | Solder final PCB | `04_PCB_PLACEMENT_THERMAL.md` | 3–4 hours |
| 5 | Flash main code | `06_MAIN_LOGIC_CODE.md` | 1–2 hours |
| 6 | Tune and calibrate | `07_TUNING_CALIBRATION.md` | 2–4 hours |
| 7 | Implement speed run | `09_SPEED_RUN.md` | 2–3 hours |
| 8 | Competition prep | `10_COMPETITION_PREP.md` | 1 hour |

**Total estimated build time:** 15–25 hours (spread over 2–4 weekends)

---

## 🔗 External References

| Resource | Link |
|----------|------|
| IEEE Micromouse Rules | https://ieee.org/micromouse |
| STM32duino Board Package | https://github.com/stm32duino/Arduino_Core_STM32 |
| STM32duino Wiki | https://github.com/stm32duino/wiki/wiki |
| VL53L0X Pololu Library | https://github.com/pololu/vl53l0x-arduino |
| MPU6050 Library | https://github.com/ElectronicCats/mpu6050 |
| Micromouse Online Simulator | https://micromouseonline.com |
| Green Ye's Micromouse Blog | https://greenye.net |
