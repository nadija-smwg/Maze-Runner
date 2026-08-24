#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


// Define XSHUT pins (Updated to match your actual STM32 wiring)
#define PIN_TOF_XSHUT_FRONT         PA4     /**< Front sensor XSHUT               */
#define PIN_TOF_XSHUT_FRONT_LEFT    PB1     /**< Front-left (L-45°) sensor XSHUT  */
#define PIN_TOF_XSHUT_FRONT_RIGHT   PC14    /**< Front-right (R-45°) sensor XSHUT */
#define PIN_TOF_XSHUT_LEFT          PA15    /**< Left (L-90°) sensor XSHUT        */
#define PIN_TOF_XSHUT_RIGHT         PB3     /**< Right (R-90°) sensor XSHUT       */

// Create objects for the sensors (from Pololu VL53L0X library)
VL53L0X sensor1;
VL53L0X sensor2;
VL53L0X sensor3;
VL53L0X sensor4;
VL53L0X sensor5;

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Pololu VL53L0X Continuous Mode Test");

  // Set I2C pins for STM32 before starting Wire
  Wire.setSCL(PB8);
  Wire.setSDA(PB9);
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C for speed!

  // Initialize OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("SSD1306 allocation failed");
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Pololu ToF Test");
    display.display();
  }

  // 1. Disable all sensors by pulling XSHUT LOW
  pinMode(PIN_TOF_XSHUT_FRONT, OUTPUT); digitalWrite(PIN_TOF_XSHUT_FRONT, LOW);
  pinMode(PIN_TOF_XSHUT_FRONT_LEFT, OUTPUT); digitalWrite(PIN_TOF_XSHUT_FRONT_LEFT, LOW);
  pinMode(PIN_TOF_XSHUT_FRONT_RIGHT, OUTPUT); digitalWrite(PIN_TOF_XSHUT_FRONT_RIGHT, LOW);
  pinMode(PIN_TOF_XSHUT_LEFT, OUTPUT); digitalWrite(PIN_TOF_XSHUT_LEFT, LOW);
  pinMode(PIN_TOF_XSHUT_RIGHT, OUTPUT); digitalWrite(PIN_TOF_XSHUT_RIGHT, LOW);
  delay(10);

  // 2. Wake up and initialize Sensor 1 (Front)
  digitalWrite(PIN_TOF_XSHUT_FRONT, HIGH);
  delay(10);
  sensor1.init();
  sensor1.setTimeout(500);
  sensor1.setAddress(0x30); // Assign unique I2C address
  sensor1.startContinuous(); // Start measuring in the background constantly

  // 3. Wake up and initialize Sensor 2 (Front-Left)
  digitalWrite(PIN_TOF_XSHUT_FRONT_LEFT, HIGH);
  delay(10);
  sensor2.init();
  sensor2.setTimeout(500);
  sensor2.setAddress(0x31);
  sensor2.startContinuous();

  // 4. Wake up and initialize Sensor 3 (Front-Right)
  digitalWrite(PIN_TOF_XSHUT_FRONT_RIGHT, HIGH);
  delay(10);
  sensor3.init();
  sensor3.setTimeout(500);
  sensor3.setAddress(0x32);
  sensor3.startContinuous();

  // 5. Wake up and initialize Sensor 4 (Left)
  digitalWrite(PIN_TOF_XSHUT_LEFT, HIGH);
  delay(10);
  sensor4.init();
  sensor4.setTimeout(500);
  sensor4.setAddress(0x33);
  sensor4.startContinuous();

  // 6. Wake up and initialize Sensor 5 (Right)
  digitalWrite(PIN_TOF_XSHUT_RIGHT, HIGH);
  delay(10);
  sensor5.init();
  sensor5.setTimeout(500);
  sensor5.setAddress(0x34);
  sensor5.startContinuous();

  Serial.println("All 5 Sensors Initialized! Starting loop...");
}

void loop() {
  uint32_t start_time = millis();

  // Read the sensors
  // Because they are in continuous mode, the laser fires continuously in the background.
  // This command just grabs the latest finished reading.
  uint16_t d1 = sensor1.readRangeContinuousMillimeters();
  uint16_t d2 = sensor2.readRangeContinuousMillimeters();
  uint16_t d3 = sensor3.readRangeContinuousMillimeters();
  uint16_t d4 = sensor4.readRangeContinuousMillimeters();
  uint16_t d5 = sensor5.readRangeContinuousMillimeters();
  
  uint32_t end_time = millis();

  // Print results
  Serial.print("S1:"); Serial.print(d1);
  Serial.print(" | S2:"); Serial.print(d2);
  Serial.print(" | S3:"); Serial.print(d3);
  Serial.print(" | S4:"); Serial.print(d4);
  Serial.print(" | S5:"); Serial.print(d5);
  
  // Print how long it took to read all 5 sensors
  Serial.print(" || Block Time: ");
  Serial.print(end_time - start_time);
  Serial.println(" ms");

  // Update OLED Display
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Pololu ToF Test");
  
  display.setCursor(0, 15);
  display.print("F:"); display.print(d1);
  display.print(" FL:"); display.print(d2);
  display.print(" FR:"); display.println(d3);
  
  display.setCursor(0, 30);
  display.print("L:"); display.print(d4);
  display.print(" R:"); display.println(d5);

  display.setCursor(0, 50);
  display.print("Block time: ");
  display.print(end_time - start_time);
  display.print(" ms");
  
  display.display();

  delay(50); // Small delay to slow down the serial monitor
}
