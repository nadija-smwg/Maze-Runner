# 05 — Testing Stages with Code
## Test Each System Independently Before Integration

---

> [!WARNING]
> **Testing Order — NEVER skip steps**
```mermaid
flowchart TD
    A[Stage 1: Power & Voltage] --> B[Stage 2: STM32 Blink]
    B --> C[Stage 3: Motor Driver]
    C --> D[Stage 4: Encoders]
    D --> I2C[Stage 4.5: I2C Bus Scan]
    I2C --> E[Stage 5: MPU6050 IMU]
    E --> F[Stage 6: VL53L0X Sensors]
    F --> BATT[Stage 6.5: Battery Monitor]
    BATT --> G[Stage 7: PID Control]
    G --> H[Stage 8: Straight Line]
    H --> I[Stage 9: 90° Turn]
    I --> J((Stage 10: Full Integration))
    
    classDef critical fill:#f9f,stroke:#333,stroke-width:2px;
    class J critical;
```

---

## Stage 1 — Power & Voltage Check

**Before connecting anything:**
1. Connect battery through switch to buck converter input
2. Measure buck converter output → must be **5.0V ± 0.1V**
3. Connect 5V to LDO input, measure output → must be **3.3V ± 0.05V**
4. Only then connect STM32

No code needed — just a multimeter.

**✅ PASS:** 5V reads 4.9–5.1V, 3.3V reads 3.25–3.35V | **❌ FAIL:** Voltage out of range or fluctuating

---

## Stage 2 — STM32 Blink Test

**Purpose:** Confirm STM32 is alive and Arduino IDE can upload.

```cpp
// STAGE 2: Basic blink test
// Board: Generic STM32F4xx (STM32F401CCU6 BlackPill)
// Upload via: STLink

void setup() {
  pinMode(PC13, OUTPUT);  // Built-in LED on STM32F401CCU6
  Serial.begin(115200);
  Serial.println("STM32 Alive!");
}

void loop() {
  digitalWrite(PC13, LOW);   // LED ON (active low on STM32F401CCU6)
  delay(500);
  digitalWrite(PC13, HIGH);  // LED OFF
  delay(500);
}
```

**Expected:** LED blinks every 0.5s, serial prints "STM32 Alive!"

**✅ PASS:** LED blinks, serial output visible | **❌ FAIL:** No blink or no serial output

---

## Stage 3 — Motor Driver Test (No Encoders)

**Purpose:** Confirm TB6612FNG wiring and motor direction.

```cpp
// STAGE 3: Motor driver test
// Tests: Forward, Reverse, Brake, both motors independently
// Pin mapping matches actual tested hardware (TIM1 PWM at 20kHz)

#define PWMA  PA8     // TIM1 CH1
#define AIN1  PB12
#define AIN2  PB13

#define PWMB  PA9     // TIM1 CH2
#define BIN1  PB15
#define BIN2  PA10

#define STBY  PB14

void setMotorA(int speed) {
  // speed: -255 to +255
  if (speed > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, speed);
  } else if (speed < 0) {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, -speed);
  } else {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, HIGH);  // Brake
    analogWrite(PWMA, 0);
  }
}

void setMotorB(int speed) {
  if (speed > 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, speed);
  } else if (speed < 0) {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, -speed);
  } else {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, 0);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  digitalWrite(STBY, HIGH);  // Enable driver
  Serial.println("Motor Test Start");
}

void loop() {
  Serial.println("Both Forward");
  setMotorA(150);
  setMotorB(150);
  delay(2000);

  Serial.println("Both Stop");
  setMotorA(0);
  setMotorB(0);
  delay(1000);

  Serial.println("Both Reverse");
  setMotorA(-150);
  setMotorB(-150);
  delay(2000);

  Serial.println("Brake");
  setMotorA(0);
  setMotorB(0);
  delay(2000);
}
```

**Expected:** Motors spin forward, stop, reverse in sequence.

**✅ PASS:** Both motors respond to commands, direction matches code | **❌ FAIL:** Motor doesn't spin, wrong direction, or stutters

**If motor runs backward when it should go forward:** Swap AO1/AO2 wires (or change `AIN1/AIN2` logic in code).

---

## Stage 4 — Encoder Test

**Purpose:** Confirm quadrature encoder reading and direction detection.

> [!NOTE]
> This project uses **hardware encoder mode** (TIM2/TIM3 Encoder Mode 3) instead of software EXTI interrupts.
> This is superior: zero CPU overhead, 4× resolution, no missed pulses at high RPM.
> Left encoder uses TIM2 (PA0/PA1, 32-bit). Right encoder uses TIM3 (PA6/PA7, 16-bit).

```cpp
// STAGE 4: Encoder reading test
// Uses TIM2/TIM3 hardware encoder mode (register-level setup)

#include <Arduino.h>

void setupLeftEncoder() {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

  // PA0 PA1 Alternate Function (TIM2 CH1 and CH2)
  GPIOA->MODER &= ~(3 << (0 * 2));
  GPIOA->MODER &= ~(3 << (1 * 2));
  GPIOA->MODER |= (2 << (0 * 2));
  GPIOA->MODER |= (2 << (1 * 2));

  // AF1 for TIM2
  GPIOA->AFR[0] &= ~(0xF << 0);
  GPIOA->AFR[0] &= ~(0xF << 4);
  GPIOA->AFR[0] |= (1 << 0);
  GPIOA->AFR[0] |= (1 << 4);

  // Reset and configure timer
  TIM2->CR1 = 0; TIM2->CR2 = 0; TIM2->SMCR = 0;
  TIM2->CCMR1 = 0; TIM2->CCER = 0;
  TIM2->PSC = 0;
  TIM2->ARR = 0xFFFFFFFF; // TIM2 is 32-bit

  // Encoder Mode 3 (count on both edges of both channels)
  TIM2->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
  TIM2->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
  TIM2->CCMR1 |= (3 << 4) | (3 << 12); // Input filter

  TIM2->CNT = 0;
  TIM2->CR1 |= TIM_CR1_CEN;
}

void setupRightEncoder() {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

  // PA6 PA7 Alternate Function (TIM3 CH1 and CH2)
  GPIOA->MODER &= ~(3 << (6 * 2));
  GPIOA->MODER &= ~(3 << (7 * 2));
  GPIOA->MODER |= (2 << (6 * 2));
  GPIOA->MODER |= (2 << (7 * 2));

  // AF2 for TIM3
  GPIOA->AFR[0] &= ~(0xF << (6 * 4));
  GPIOA->AFR[0] &= ~(0xF << (7 * 4));
  GPIOA->AFR[0] |= (2 << (6 * 4));
  GPIOA->AFR[0] |= (2 << (7 * 4));

  // Reset and configure timer
  TIM3->CR1 = 0; TIM3->CR2 = 0; TIM3->SMCR = 0;
  TIM3->CCMR1 = 0; TIM3->CCER = 0;
  TIM3->PSC = 0;
  TIM3->ARR = 0xFFFF; // TIM3 is 16-bit

  // Encoder Mode 3
  TIM3->SMCR |= TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
  TIM3->CCMR1 |= TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
  TIM3->CCMR1 |= (3 << 4) | (3 << 12); // Input filter

  TIM3->CNT = 0;
  TIM3->CR1 |= TIM_CR1_CEN;
}

int32_t getLeftCount() {
  return (int32_t)TIM2->CNT;
}

int32_t getRightCount() {
  return (int32_t)(int16_t)TIM3->CNT; // 16-bit sign extension
}

void setup() {
  Serial.begin(115200);
  setupLeftEncoder();
  setupRightEncoder();
  Serial.println("Encoder Test — spin wheels by hand");
}

void loop() {
  Serial.print("Left: ");
  Serial.print(getLeftCount());
  Serial.print("  Right: ");
  Serial.println(getRightCount());
  delay(100);
}
```

**Expected:**
- Spinning left wheel forward → count increases
- Spinning left wheel backward → count decreases
- Same for right
- If direction is wrong: swap encoder A/B wires physically

**Pulses per revolution:**
- Manually spin wheel exactly once → note encoder count
- This is your **CPR (counts per revolution)**
- With hardware encoder mode 3, you get 4× the base PPR
- Record this value for odometry calculations

**✅ PASS:** Forward spin increases count, backward decreases. Consistent CPR. | **❌ FAIL:** Count stays at 0, or inconsistent

---

## Stage 4.5 — I2C Bus Scan

**Purpose:** Verify all I2C devices are detected before testing them individually.

```cpp
// STAGE 4.5: I2C bus scanner
// Detects all devices on the I2C bus

#include <Wire.h>

void setup() {
  Serial.begin(115200);
  Wire.begin(PB9, PB8);  // SDA, SCL for STM32
  Wire.setClock(400000);
  Serial.println("I2C Bus Scanner");
  Serial.println("Scanning...");
}

void loop() {
  int devices = 0;
  
  for (byte addr = 8; addr < 120; addr++) {
    Wire.beginTransmission(addr);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("Device found at 0x");
      if (addr < 16) Serial.print("0");
      Serial.print(addr, HEX);
      
      // Identify known devices
      if (addr == 0x68) Serial.print(" ← MPU6050");
      if (addr == 0x69) Serial.print(" ← MPU6050 (AD0=HIGH)");
      if (addr == 0x29) Serial.print(" ← VL53L0X (default)");
      if (addr >= 0x30 && addr <= 0x35) {
        Serial.print(" ← VL53L0X #");
        Serial.print(addr - 0x2F);
      }
      if (addr == 0x3C || addr == 0x3D) Serial.print(" ← SSD1306 OLED");
      
      Serial.println();
      devices++;
    }
  }
  
  Serial.print("\nTotal devices found: ");
  Serial.println(devices);
  Serial.println("---");
  delay(5000);
}
```

**Expected:**
- MPU6050 at 0x68 (with AD0=GND) or 0x69 (AD0=HIGH)
- VL53L0X at 0x29 (before address assignment) or assigned addresses

**✅ PASS:** All expected devices detected | **❌ FAIL:** Missing devices — check wiring, pull-ups, and power

---

## Stage 5 — MPU6050 Test

**Install library first:** Tools → Manage Libraries → search "MPU6050" by Electronic Cats or Jeff Rowberg

```cpp
// STAGE 5: MPU6050 gyroscope test
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

float yaw = 0;
unsigned long lastTime = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(PB9, PB8);  // SDA, SCL for STM32
  Wire.setClock(400000); // 400kHz Fast I2C
  
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 NOT FOUND! Check wiring.");
    while (1);
  }
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
  
  Serial.println("MPU6050 OK. Calibrating — keep still for 3 seconds...");
  delay(3000);
  Serial.println("Done. Rotate robot and watch yaw.");
  lastTime = millis();
}

void loop() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  float gyroZ = gz / 65.5;  // Convert to degrees/sec (65.5 LSB per deg/s at 500dps range)
  // Note: at 250dps range, divide by 131.0 instead
  
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;
  
  yaw += gyroZ * dt;
  
  Serial.print("Yaw: ");
  Serial.print(yaw, 2);
  Serial.println("°");
  
  delay(20);  // 50Hz update
}
```

**Expected:**
- Robot still: yaw stays near 0 (±0.5° drift per minute acceptable)
- Robot rotated 90° right: yaw shows ~90
- Robot rotated 90° left: yaw shows ~-90

**✅ PASS:** Yaw drift < 1°/min when still, and reads ±5° of expected when rotated | **❌ FAIL:** Rapid drift or no reading

**If MPU6050 not found:** Check I2C address — AD0 pin LOW = 0x68, AD0 HIGH = 0x69. Check pull-ups.

---

## Stage 6 — VL53L0X Sensor Test

**Install library:** Library Manager → "VL53L0X" by Pololu

```cpp
// STAGE 6: VL53L0X sensor test with address assignment
#include <Wire.h>
#include <VL53L0X.h>

// For 3-sensor config
#define XSHUT_1 PC13   // Front
#define XSHUT_2 PC14   // Left  
#define XSHUT_3 PC15   // Right

#define ADDR_1 0x30
#define ADDR_2 0x31
#define ADDR_3 0x32

VL53L0X sensor1, sensor2, sensor3;

void assignSensorAddresses() {
  // Shut all down first
  pinMode(XSHUT_1, OUTPUT);
  pinMode(XSHUT_2, OUTPUT);
  pinMode(XSHUT_3, OUTPUT);
  digitalWrite(XSHUT_1, LOW);
  digitalWrite(XSHUT_2, LOW);
  digitalWrite(XSHUT_3, LOW);
  delay(10);
  
  // Enable sensor 1, assign address
  digitalWrite(XSHUT_1, HIGH);
  delay(10);
  sensor1.init();
  sensor1.setAddress(ADDR_1);
  Serial.println("Sensor 1 (Front) initialized at 0x30");
  
  // Enable sensor 2
  digitalWrite(XSHUT_2, HIGH);
  delay(10);
  sensor2.init();
  sensor2.setAddress(ADDR_2);
  Serial.println("Sensor 2 (Left) initialized at 0x31");
  
  // Enable sensor 3
  digitalWrite(XSHUT_3, HIGH);
  delay(10);
  sensor3.init();
  sensor3.setAddress(ADDR_3);
  Serial.println("Sensor 3 (Right) initialized at 0x32");
}

void setup() {
  Serial.begin(115200);
  Wire.begin(PB9, PB8);
  Wire.setClock(400000);
  
  assignSensorAddresses();
  
  // Set to high speed mode (shorter range but faster)
  sensor1.setMeasurementTimingBudget(20000);  // 20ms per reading
  sensor2.setMeasurementTimingBudget(20000);
  sensor3.setMeasurementTimingBudget(20000);
  
  sensor1.startContinuous();
  sensor2.startContinuous();
  sensor3.startContinuous();
  
  Serial.println("Sensors ready. Move objects in front of them.");
}

void loop() {
  int d1 = sensor1.readRangeContinuousMillimeters();
  int d2 = sensor2.readRangeContinuousMillimeters();
  int d3 = sensor3.readRangeContinuousMillimeters();
  
  Serial.print("Front: "); Serial.print(d1); Serial.print("mm  ");
  Serial.print("Left: ");  Serial.print(d2); Serial.print("mm  ");
  Serial.print("Right: "); Serial.print(d3); Serial.println("mm");
  
  // 65535 = out of range / timeout
  delay(50);
}
```

**Expected:**
- Open air: ~1200mm or 65535 (timeout)
- Hand 200mm away: ~200mm reading
- Each sensor responds independently

**✅ PASS:** All 3 sensors respond, readings match real distance ±10mm | **❌ FAIL:** 65535 constantly, or sensor not initializing

---

## Stage 6.5 — Battery Voltage Monitor Test

**Purpose:** Read battery voltage via ADC for low-voltage protection.

**Wiring:** Create a voltage divider: Battery+ → 20kΩ → PA5 → 10kΩ → GND
(This divides 8.4V max down to 2.8V — safe for 3.3V ADC)

```cpp
// STAGE 6.5: Battery voltage monitor

#define VBAT_PIN PA5
#define DIVIDER_RATIO 3.0    // (20k + 10k) / 10k = 3.0
#define ADC_REF 3.3
#define ADC_MAX 4095.0       // 12-bit ADC

void setup() {
  Serial.begin(115200);
  pinMode(VBAT_PIN, INPUT_ANALOG);
  Serial.println("Battery Monitor Test");
}

void loop() {
  // Average 10 readings for stability
  long sum = 0;
  for (int i = 0; i < 10; i++) {
    sum += analogRead(VBAT_PIN);
    delay(2);
  }
  float adcAvg = sum / 10.0;
  
  float voltage = (adcAvg / ADC_MAX) * ADC_REF * DIVIDER_RATIO;
  float cellVoltage = voltage / 2.0;  // 2S pack
  
  Serial.print("Battery: ");
  Serial.print(voltage, 2);
  Serial.print("V  Cell: ");
  Serial.print(cellVoltage, 2);
  Serial.print("V  ");
  
  if (cellVoltage > 3.7) Serial.println("[GOOD]");
  else if (cellVoltage > 3.5) Serial.println("[OK]");
  else if (cellVoltage > 3.2) Serial.println("[LOW - charge soon]");
  else Serial.println("[CRITICAL - STOP IMMEDIATELY!]");
  
  delay(1000);
}
```

**Expected:**
- Fully charged: ~8.2–8.4V (4.1–4.2V/cell)
- Normal use: 7.0–7.8V (3.5–3.9V/cell)

**✅ PASS:** Voltage reading matches multimeter ±0.1V | **❌ FAIL:** Reading is 0 or wildly inaccurate

---

## Stage 7 — PID Motor Speed Control

**Purpose:** Make motors run at target RPM accurately.

```cpp
// STAGE 7: PID speed control test
// Uses hardware encoder mode (TIM2/TIM3) and motor driver from Stages 3 & 4
// Pin mapping matches actual tested hardware

// Include motor driver setup from Stage 3 (PA8/PA9, PB12/PB13/PB15/PA10, PB14)
// Include encoder setup from Stage 4 (TIM2: PA0/PA1, TIM3: PA6/PA7)
// Then add:

#define COUNTS_PER_REV 600.0    // Counts per revolution (measure in stage 4!)
#define MM_PER_COUNT   0.178f   // Calibrated mm per encoder count
#define SAMPLE_TIME    0.01f    // 10ms PID loop

// PID gains — tune these (start low!)
float Kp = 1.8, Ki = 0.8, Kd = 0.02;

float targetLeftSpeed = 300.0f;   // mm/s
float targetRightSpeed = 300.0f;  // mm/s
float leftSpeed = 0, rightSpeed = 0;
float leftPWM = 0, rightPWM = 0;

int32_t lastLeftCount = 0;
int16_t lastRightCount = 0;
unsigned long lastPID = 0;

struct PIDController {
  float kp, ki, kd;
  float error, previousError, integral, derivative;
};

PIDController leftPID, rightPID;

void setupPID() {
  leftPID.kp = Kp;  leftPID.ki = Ki;  leftPID.kd = Kd;
  leftPID.error = 0; leftPID.previousError = 0;
  leftPID.integral = 0; leftPID.derivative = 0;
  rightPID = leftPID;
}

float calculatePID(PIDController &pid, float target, float actual) {
  pid.error = target - actual;
  pid.integral += pid.error * SAMPLE_TIME;
  pid.integral = constrain(pid.integral, -500, 500);  // Anti-windup
  pid.derivative = (pid.error - pid.previousError) / SAMPLE_TIME;
  float output = pid.kp * pid.error + pid.ki * pid.integral + pid.kd * pid.derivative;
  pid.previousError = pid.error;
  return output;
}

void updateSpeed() {
  int32_t currentLeft = TIM2->CNT;
  int32_t leftDelta = currentLeft - lastLeftCount;
  lastLeftCount = currentLeft;

  int16_t currentRight = (int16_t)TIM3->CNT;
  int16_t rightDelta = -(currentRight - lastRightCount); // Negate if needed
  lastRightCount = currentRight;

  leftSpeed = (leftDelta * MM_PER_COUNT) / SAMPLE_TIME;
  rightSpeed = (rightDelta * MM_PER_COUNT) / SAMPLE_TIME;
}

void updateMotorPID() {
  updateSpeed();
  leftPWM += calculatePID(leftPID, targetLeftSpeed, leftSpeed);
  rightPWM += calculatePID(rightPID, targetRightSpeed, rightSpeed);
  leftPWM = constrain(leftPWM, 0, 4999);
  rightPWM = constrain(rightPWM, 0, 4999);
  // Apply to TIM1 compare registers directly
  TIM1->CCR1 = (uint16_t)leftPWM;
  TIM1->CCR2 = (uint16_t)rightPWM;
}

void setup() {
  Serial.begin(115200);
  analogWriteResolution(13);  // 0-8191 range for PWM
  setupMotorDriver();  // From Stage 3
  setupLeftEncoder();  // From Stage 4
  setupRightEncoder();
  setupPID();
  delay(2000);

  // Set motors forward
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
}

void loop() {
  if (millis() - lastPID >= 10) {
    lastPID = millis();
    updateMotorPID();

    Serial.print("Target:"); Serial.print(targetLeftSpeed);
    Serial.print(",Left:"); Serial.print(leftSpeed);
    Serial.print(",Right:"); Serial.println(rightSpeed);
  }
}
```

**Expected:** Both motors run at ~150 RPM. PWM values settle after a few seconds.

**✅ PASS:** RPM stabilizes within ±10 of target in <3 seconds | **❌ FAIL:** Oscillating, maxing out, or no response

---

## Stage 8 — Straight Line Test

**Purpose:** Robot drives straight without drifting.

Combine PID speed control with encoder odometry feedback. Set both motors to same target speed.

```cpp
// After Stage 7 is working:
// Add this — corrections to keep robot straight using encoder differential

float straightCorrection(long encL, long encR) {
  float diff = encL - encR;  // If positive, left went further → need to slow left
  return diff * 0.5;          // Correction gain — tune this
}

// In main loop:
float correction = straightCorrection(encoderL, encoderR);
float targetL = TARGET_RPM - correction;
float targetR = TARGET_RPM + correction;
// Feed targetL, targetR into PID instead of fixed TARGET_RPM
```

**Expected:** Robot drives 1 meter in a straight line (<5mm lateral deviation).

**✅ PASS:** <5mm deviation over 1 meter | **❌ FAIL:** Curves or oscillates

---

## Stage 9 — 90° Turn Test

```cpp
// Turn left 90 degrees using encoder counting

void turnLeft90() {
  // Calculate encoder counts for 90° turn
  // Arc length = (PI/2) * wheelbase_radius
  // wheelbase = distance between wheel centers (measure yours, e.g. 70mm)
  float wheelbase = 70.0;   // mm
  float wheelDiam = 34.0;   // mm
  float wheelCirc = PI * wheelDiam;
  float arcLength = (PI / 2.0) * (wheelbase / 2.0);
  int targetCounts = (int)(arcLength / wheelCirc * ENCODER_CPR);
  
  long startL = encoderL;
  long startR = encoderR;
  
  // Left motor backward, right motor forward
  setMotorA(-120);  // Left backward
  setMotorB(120);   // Right forward
  
  while (true) {
    long dL = abs(encoderL - startL);
    long dR = abs(encoderR - startR);
    long avg = (dL + dR) / 2;
    if (avg >= targetCounts) break;
  }
  
  setMotorA(0);
  setMotorB(0);
}
```

**Expected:** Robot turns approximately 90°. Verify with MPU6050 yaw reading.

**✅ PASS:** Turn within ±5° of 90° | **❌ FAIL:** Under/overshooting significantly

After passing this, combine gyro feedback for more accurate turns (see `06_MAIN_LOGIC_CODE.md`).

---

## Stage 10 — Full Integration Test

**Purpose:** Verify all subsystems work together in a controlled mini-maze.

### Test Environment:
- Build a 2×2 or 3×3 mini-maze with cardboard walls (18cm cells, 5cm tall walls)
- Start robot in corner cell

### Integration Test Code:
```cpp
// STAGE 10: Full integration test
// Runs a simplified exploration in a small test maze

// Include ALL code from stages 3-9 and 06_MAIN_LOGIC_CODE.md
// Set MAZE_SIZE to 4 for a small test maze:
// #define MAZE_SIZE 4

// Override goal to cell (1,1) for small maze:
// bool isGoal(int x, int y) { return x == 1 && y == 1; }

void setup() {
  Serial.begin(115200);
  Serial.println("=== FULL INTEGRATION TEST ===");
  
  initMotors();
  initSensors();
  calibrateGyro();  // From 07_TUNING_CALIBRATION
  
  // Initialize small maze
  memset(walls, 0, sizeof(walls));
  memset(visited, 0, sizeof(visited));
  
  // Set border walls for small maze
  for (int i = 0; i < MAZE_SIZE; i++) {
    walls[0][i]           |= 8;  // West
    walls[MAZE_SIZE-1][i] |= 2;  // East
    walls[i][0]           |= 4;  // South
    walls[i][MAZE_SIZE-1] |= 1;  // North
  }
  
  floodFill();
  
  pinMode(BTN_START, INPUT_PULLUP);
  Serial.println("Press START button.");
  while (digitalRead(BTN_START) == HIGH);
  delay(1000);  // 1 second delay after button press
  
  Serial.println("Running!");
}

void loop() {
  // Print state before each step
  printMazeState();  // From 08_TROUBLESHOOTING
  
  // Check battery
  float vbat = readBatteryVoltage();
  if (vbat < 6.4) {
    setMotors(0, 0);
    Serial.println("LOW BATTERY! STOPPING.");
    while(1);
  }
  
  exploreStep();
  
  if (goalReached) {
    setMotors(0, 0);
    Serial.println("=== GOAL REACHED! TEST PASSED! ===");
    // Print final maze map
    for (int y = MAZE_SIZE-1; y >= 0; y--) {
      for (int x = 0; x < MAZE_SIZE; x++) {
        Serial.print(floodValues[x][y]);
        Serial.print("\t");
      }
      Serial.println();
    }
    while(1);  // Stop
  }
}
```

### Integration Checklist:
- [ ] Robot reads sensors without crashing when motors are running
- [ ] Wall detection matches physical walls in test maze
- [ ] Robot navigates at least 3 cells correctly
- [ ] Turns are accurate (±5°)
- [ ] Robot stops at goal cell
- [ ] Battery voltage reads correctly under motor load
- [ ] No I2C lockups during 5-minute continuous run

**✅ PASS:** Robot completes mini-maze without errors | **❌ FAIL:** See `08_TROUBLESHOOTING.md`
