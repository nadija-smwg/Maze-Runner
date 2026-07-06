# 08 — Troubleshooting Guide

---

## Upload / Programming Issues

### "Cannot upload to STM32"
```
Problem: Arduino IDE can't find the board
Fix:
  1. Install STM32duino board package:
     File → Preferences → Board Manager URLs:
     https://github.com/stm32duino/BoardManagerFiles/raw/main/package_stmicroelectronics_index.json
  2. Tools → Board → STM32 Boards → Select correct board
  3. Tools → Upload Method → STLink (if using ST-Link V2)
  4. Check ST-Link V2 wiring: SWDIO, SWCLK, GND, 3.3V
```

### "STM32 board keeps resetting"
```
Fix: Check power supply is stable. Add 100µF cap on 3.3V rail.
     Also check BOOT0 pin — must be LOW (GND) for normal run mode.
```

### "Wire.begin() causes I2C lockup"
```
Fix for STM32 with STM32duino:
  Wire.begin(PB9, PB8);  // Must specify SDA, SCL for STM32
  Wire.setClock(400000);
  
  If still locking: add timeout recovery:
  Wire.setWireTimeout(3000, true);  // 3ms timeout, auto-reset
```

---

## Sensor Issues

### "VL53L0X not found" / I2C scan shows nothing
```
Checklist:
[ ] SDA and SCL wired to correct STM32 pins (PB9=SDA, PB8=SCL)?
[ ] 4.7kΩ pull-up resistors on SDA and SCL to 3.3V?
[ ] Sensor powered with 3.3V (not 5V)?
[ ] Wire.begin(PB9, PB8) called before sensor.init()?
[ ] XSHUT pin HIGH before calling init()?

I2C Scanner code:
  for (byte addr = 8; addr < 120; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("Found at 0x");
      Serial.println(addr, HEX);
    }
  }
```

### "Only first VL53L0X detected, others missing"
```
Problem: All sensors start at 0x29 — address collision
Fix: Follow XSHUT procedure in Stage 6 code exactly.
     Verify each XSHUT pin individually with multimeter.
     Check that you're calling sensor.setAddress() before
     enabling the next sensor.
```

### "VL53L0X reads 65535 or 8190 constantly"
```
65535 = timeout (no reading within time budget)
Fix:
  - Object too far (>1.2m) or no object in view
  - Increase timing budget: sensor.setMeasurementTimingBudget(33000)
  - Check sensor is not pointing at absorptive black surface
  - Check sensor lens is clean and unobstructed
```

### "MPU6050 yaw drifts rapidly"
```
Normal drift: < 1°/minute when still
Excessive drift causes:
  - Motor vibration transmitted to sensor
  - Mount MPU6050 on foam/rubber pad
  - Calibrate gyro bias at startup (see Stage 5)
  - Temperature change — recalibrate after warmup
  - Check AD0 pin — must be stable (not floating)
```

---

## Motor Issues

### "Motors don't move"
```
Checklist:
[ ] STBY pin HIGH?
[ ] Motor power (VM) connected to 7.4V?
[ ] Logic power (VCC) connected to 3.3V?
[ ] analogWrite() range — STM32 may default to 8-bit (0-255)
    Confirm with: analogWriteResolution(8); in setup()
[ ] PWM pin is actually a PWM-capable pin on your STM32?
```

### "One motor runs, other doesn't"
```
Test each motor channel independently with Stage 3 code.
Check BIN1/BIN2 wiring to correct TB6612FNG pins.
Check motor wires fully inserted into motor pads.
```

### "Motors stutter or vibrate instead of spinning smoothly"
```
Fix:
  - Increase PWM frequency (STM32 can do 20kHz):
    analogWriteFrequency(PWMA, 20000);  // (if supported by STM32duino version)
  - Check power supply — motor noise causing voltage drops
  - Add 100µF cap on VM pin of TB6612FNG
  - Check encoder ISR — if ISR is too slow, it blocks motor updates
```

### "Robot curves when it should go straight"
```
Causes:
  a) Different wheel diameters → measure and update code
  b) Different motor characteristics → PID will correct this
  c) One encoder giving wrong count → test encoders in Stage 4
  d) Straight-line correction gain too low → increase Kp_straight
  e) MPU6050 not centered on robot → reposition
```

---

## Encoder Issues

### "Encoder count stays at 0"
```
Fix:
  - Check encoder wires connected to correct STM32 interrupt pins
  - STM32 interrupt pins — most pins support interrupts on STM32F4
  - Check encoder VCC (some need 5V, check datasheet)
  - If 5V encoder, add voltage divider before STM32 pin
  - Test with: digitalRead(ENC_LA) → toggle manually
```

### "Encoder counts too high or inconsistent"
```
Fix:
  - Change from RISING to CHANGE interrupt to count all edges (×2 counts)
    or use RISING only and divide expected CPR by 2
  - Check for encoder wire bouncing → add 100nF cap between each encoder
    pin and GND (near the encoder connector)
```

---

## Power Issues

### "Robot resets when motors start"
```
Problem: Voltage sag on power supply when motors draw current
Fix:
  - Add 470µF–1000µF electrolytic cap on 7.4V rail (near TB6612FNG)
  - Add 100µF on 5V rail
  - Check buck converter current rating ≥ 1A
  - Separate motor power and logic power with LC filter or ferrite bead
```

### "Sensors reset or give garbage when motors run"
```
Problem: Motor PWM EMI coupling into I2C bus
Fix:
  - Route I2C traces/wires away from motor/PWM traces
  - Reduce PWM frequency to 1kHz (less interference) or increase to 20kHz (above I2C)
  - Add 100nF cap on each sensor VCC pin
  - Twist I2C wire pair together
```

### "Battery drains in <5 minutes"
```
Check:
  - Battery capacity (mAh) — use 500mAh minimum
  - Motor stall — if wheels blocked, current skyrockets
  - Check current draw: multimeter in series at startup
    Normal idle: ~200mA
    Normal running: ~400–600mA
    Stalled: >1A → add stall detection
```

---

## Algorithm Issues

### "Robot goes in circles / doesn't reach goal"
```
Debug flood fill:
  Print floodValues array to Serial:
  for (int y = MAZE_SIZE-1; y >= 0; y--) {
    for (int x = 0; x < MAZE_SIZE; x++) {
      Serial.print(floodValues[x][y]); Serial.print("\t");
    }
    Serial.println();
  }
  Check: does value decrease toward goal cells?
```

### "Robot maps walls incorrectly"
```
Debug:
  - Print walls[][] array after each cell visit
  - Check sensor → absolute direction conversion
  - Verify robotHeading updates correctly after turns
  - Verify dx/dy arrays match your heading convention
```

### "Robot tries to go through walls"
```
Fix:
  - Wall thresholds too high (reads "no wall" when wall is present)
  - Lower WALL_PRESENT_THRESHOLD or increase sensor gain
  - Check sensor physical angle — must point at wall, not floor
  - Add extra confirmation: read sensor 3× and take minimum
```

---

## Quick Debug Commands (Serial Monitor)

Add these to your code for easy debugging:
```cpp
void printMazeState() {
  Serial.print("Pos: (");
  Serial.print(robotX); Serial.print(",");
  Serial.print(robotY); Serial.print(") Heading: ");
  const char* dirs[] = {"N","E","S","W"};
  Serial.println(dirs[robotHeading]);
  
  SensorData s = readSensors();
  Serial.print("Sensors F/L/R: ");
  Serial.print(s.front); Serial.print("/");
  Serial.print(s.left); Serial.print("/");
  Serial.println(s.right);
  
  Serial.print("Flood value here: ");
  Serial.println(floodValues[robotX][robotY]);
  
  Serial.print("Battery: ");
  Serial.print(readBatteryVoltage(), 2);
  Serial.println("V");
}
```

---

## I2C Bus Recovery

### "I2C bus locked / SDA stuck low"
```
Problem: A sensor crashed mid-transaction and is holding SDA low.
This prevents all I2C communication.

Automatic recovery code:
```

```cpp
void i2cRecover() {
  // Temporarily release I2C
  Wire.end();
  
  // Manually toggle SCL 9 times to free stuck SDA
  pinMode(PB8, OUTPUT);  // SCL pin
  pinMode(PB9, INPUT);   // SDA pin — check state
  
  for (int i = 0; i < 9; i++) {
    digitalWrite(PB8, LOW);
    delayMicroseconds(5);
    digitalWrite(PB8, HIGH);
    delayMicroseconds(5);
    
    // Check if SDA is released
    if (digitalRead(PB9) == HIGH) {
      Serial.print("I2C recovered after ");
      Serial.print(i + 1);
      Serial.println(" clock pulses.");
      break;
    }
  }
  
  // Generate STOP condition
  pinMode(PB9, OUTPUT);
  digitalWrite(PB9, LOW);
  delayMicroseconds(5);
  digitalWrite(PB8, HIGH);
  delayMicroseconds(5);
  digitalWrite(PB9, HIGH);
  delayMicroseconds(5);
  
  // Re-initialize I2C
  Wire.begin(PB9, PB8);
  Wire.setClock(400000);
}

// Call this if a sensor read returns 65535 multiple times in a row:
// if (consecutiveTimeouts > 3) i2cRecover();
```

### Prevention tips:
- Set I2C timeout: `Wire.setWireTimeout(3000, true);` (3ms timeout, auto-reset)
- Use hardware watchdog (see `06_MAIN_LOGIC_CODE.md`)
- Add 100nF bypass cap on every sensor VCC pin

---

## Robot Tipping / Flipping Over

### "Robot tips forward during braking"
```
Problem: Center of gravity is too high or too far forward.

Fix:
  1. Move battery to the rear — it's the heaviest component
  2. Lower overall height — keep under 60mm
  3. Reduce deceleration rate in moveForward():
     - Change: int speed = (progress > 0.8) ? 80 : baseSpeed;
     - To:     int speed = baseSpeed * (1.0 - progress * 0.7);
       This creates a linear deceleration instead of sudden drop.
  4. Add ball caster with appropriate height matching
  5. Consider wider wheelbase (>70mm) for more stability
```

### "Robot wheelies during acceleration"
```
Fix:
  - Reduce acceleration rate (don't jump from 0 to 200 PWM)
  - Implement trapezoidal velocity profile (ramp up speed gradually)
  - Move battery forward slightly to balance weight
  - Lower center of gravity
```

### Center of gravity check:
```
1. Balance robot on a pencil placed widthwise
2. The balance point should be near the wheel axle line
3. If it tips forward → move battery back
4. If it tips backward → move battery forward
```

---

## STM32 Crash / HardFault Debugging

### "Robot suddenly stops and LED freezes"
```
Problem: STM32 entered a HardFault — usually caused by:
  - Null pointer dereference
  - Stack overflow (too many local variables in recursive functions)
  - Division by zero
  - Invalid memory access
```

### HardFault Handler (add to your code):
```cpp
// This replaces the default HardFault handler with one that gives debug info

extern "C" void HardFault_Handler(void) {
  // Read fault status registers
  volatile uint32_t hfsr = SCB->HFSR;
  volatile uint32_t cfsr = SCB->CFSR;
  volatile uint32_t mmfar = SCB->MMFAR;
  volatile uint32_t bfar = SCB->BFAR;
  
  // Try to output debug info (may not work if UART is broken)
  Serial.println("\n\n!!! HARD FAULT !!!");
  Serial.print("HFSR:  0x"); Serial.println(hfsr, HEX);
  Serial.print("CFSR:  0x"); Serial.println(cfsr, HEX);
  Serial.print("MMFAR: 0x"); Serial.println(mmfar, HEX);
  Serial.print("BFAR:  0x"); Serial.println(bfar, HEX);
  Serial.flush();
  
  // Blink LED rapidly to signal crash
  pinMode(PC13, OUTPUT);
  while (1) {
    digitalWrite(PC13, LOW);
    for (volatile int i = 0; i < 100000; i++);
    digitalWrite(PC13, HIGH);
    for (volatile int i = 0; i < 100000; i++);
  }
}
```

### Common causes and fixes:

| CFSR Bit | Meaning | Likely Cause | Fix |
|----------|---------|-------------|-----|
| Bit 25 (DIVBYZERO) | Division by zero | `dt = 0` in PID calculation | Add `if (dt < 0.001) return;` guard |
| Bit 16 (UNDEFINSTR) | Undefined instruction | Corrupt flash or bad jump | Re-flash firmware |
| Bit 1 (DACCVIOL) | Data access violation | Bad pointer or array overflow | Check array bounds |
| Bit 0 (IACCVIOL) | Instruction access violation | Stack overflow | Reduce local variables, avoid deep recursion |

### Stack overflow prevention:
```
- Don't use large local arrays in functions called from ISRs
- The flood fill BFS uses std::queue which allocates on heap —
  ensure heap is large enough (default 8KB on STM32F411 is usually fine)
- Monitor stack usage: add canary values at the bottom of stack
```

---

## Emergency Recovery Procedure

If your robot is stuck in a bad state and you can't upload new code:

```
1. Hold BOOT0 button while pressing RESET → enters DFU mode
2. In Arduino IDE: Tools → Upload Method → STM32CubeProgrammer (DFU)
3. Flash a simple blink sketch to verify MCU is alive
4. Then re-flash your micromouse code

Alternative: Use ST-Link to force-flash — always works regardless of firmware state.
```
