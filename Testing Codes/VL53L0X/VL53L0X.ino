#include <Wire.h>

#define VL53L0X_ADDR   0x29  // 7-bit standard I2C address for VL53L0X
#define XSHUT_PIN      PA4   // Reset Pin

// Function Prototypes
void vl53l0x_writeByte(uint8_t reg, uint8_t value);
uint8_t vl53l0x_readByte(uint8_t reg);
uint16_t vl53l0x_readDistance();

void setup() {
  Serial.begin(115200);
  while (!Serial); // Wait for Serial Monitor to open

  // Configure XSHUT pin to safely reset the sensor
  pinMode(XSHUT_PIN, OUTPUT);
  digitalWrite(XSHUT_PIN, LOW);   // Put sensor in shutdown
  delay(10);
  digitalWrite(XSHUT_PIN, HIGH);  // Wake up sensor
  delay(10);                      // Wait for boot (tBOOT max is 1.2ms)

  // Initialize standard I2C at 100kHz
  Wire.begin();
  Wire.setClock(100000); 

  Serial.println("VL53L0X Register-Level Reader Initialized.");
}

void loop() {
  uint16_t distance_mm = vl53l0x_readDistance();

  Serial.print("Distance: ");
  Serial.print(distance_mm);
  Serial.println(" mm");

  delay(200); // Sampling interval
}

// Low-Level Function to read absolute distance via registers
uint16_t vl53l0x_readDistance() {
  // 1. Trigger a single-shot measurement range execution
  // Write 0x01 to SYSRANGE_START (Register 0x00)
  vl53l0x_writeByte(0x00, 0x01); 

  // 2. Poll for the data-ready flag status bit
  uint8_t ready = 0;
  uint16_t timeout = 1000;
  
  while (((ready & 0x01) == 0) && (timeout > 0)) {
    // Read RESULT_INTERRUPT_STATUS register (0x13)
    ready = vl53l0x_readByte(0x13);
    timeout--;
    delayMicroseconds(10);
  }

  // 3. Read 2 sequential bytes from the distance result register
  // The calculated 16-bit millimeter range value is stored at index 0x1E + 10 (0x24)
  Wire.beginTransmission(VL53L0X_ADDR);
  Wire.write(0x1E + 10); 
  Wire.endTransmission(false); // Send repeated start

  Wire.requestFrom(VL53L0X_ADDR, (uint8_t)2);
  
  uint8_t highByte = 0;
  uint8_t lowByte = 0;
  if (Wire.available() >= 2) {
    highByte = Wire.read();
    lowByte = Wire.read();
  }

  // 4. Clear internal hardware interrupt status flag
  // Write 0x01 to SYSTEM_INTERRUPT_CLEAR (Register 0x0B)
  vl53l0x_writeByte(0x0B, 0x01);

  // Combine bytes (VL53L0X uses Big-Endian byte arrangement)
  uint16_t distance = ((uint16_t)highByte << 8) | lowByte;
  
  return distance;
}

// Helper function to write a byte to a register
void vl53l0x_writeByte(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(VL53L0X_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

// Helper function to read a single byte from a register
uint8_t vl53l0x_readByte(uint8_t reg) {
  uint8_t value = 0;
  Wire.beginTransmission(VL53L0X_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false); // Repeated start
  
  Wire.requestFrom(VL53L0X_ADDR, (uint8_t)1);
  if (Wire.available()) {
    value = Wire.read();
  }
  return value;
}