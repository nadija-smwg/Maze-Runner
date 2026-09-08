#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// I2C Pins
#define PIN_I2C_SCL PB8
#define PIN_I2C_SDA PB9

// XSHUT Pins
#define PIN_TOF_XSHUT_FRONT       PA4
#define PIN_TOF_XSHUT_FRONT_LEFT  PB1
#define PIN_TOF_XSHUT_FRONT_RIGHT PC14
#define PIN_TOF_XSHUT_LEFT        PA15
#define PIN_TOF_XSHUT_RIGHT       PB3

// New I2C Addresses
#define TOF_ADDR_FRONT       0x30
#define TOF_ADDR_FRONT_LEFT  0x31
#define TOF_ADDR_FRONT_RIGHT 0x32
#define TOF_ADDR_LEFT        0x33
#define TOF_ADDR_RIGHT       0x34

Adafruit_VL53L0X sensor_f  = Adafruit_VL53L0X();
Adafruit_VL53L0X sensor_fl = Adafruit_VL53L0X();
Adafruit_VL53L0X sensor_fr = Adafruit_VL53L0X();
Adafruit_VL53L0X sensor_l  = Adafruit_VL53L0X();
Adafruit_VL53L0X sensor_r  = Adafruit_VL53L0X();

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

void setup() {
  Serial.begin(115200);
  // REMOVED: while (!Serial) - this blocks STM32 boot if no PC is connected!
  
  // Give a small delay for power to stabilize
  delay(100);
  
  Serial.println("Raw ToF Hardware Test Starting...");

  // Initialize I2C
  Wire.setSCL(PIN_I2C_SCL);
  Wire.setSDA(PIN_I2C_SDA);
  Wire.begin();
  Wire.setClock(400000);

  // Initialize OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Booting ToFs...");
    display.display();
  }

  // Set all XSHUT pins to OUTPUT and LOW (reset all sensors)
  pinMode(PIN_TOF_XSHUT_FRONT, OUTPUT);
  pinMode(PIN_TOF_XSHUT_FRONT_LEFT, OUTPUT);
  pinMode(PIN_TOF_XSHUT_FRONT_RIGHT, OUTPUT);
  pinMode(PIN_TOF_XSHUT_LEFT, OUTPUT);
  pinMode(PIN_TOF_XSHUT_RIGHT, OUTPUT);

  digitalWrite(PIN_TOF_XSHUT_FRONT, LOW);
  digitalWrite(PIN_TOF_XSHUT_FRONT_LEFT, LOW);
  digitalWrite(PIN_TOF_XSHUT_FRONT_RIGHT, LOW);
  digitalWrite(PIN_TOF_XSHUT_LEFT, LOW);
  digitalWrite(PIN_TOF_XSHUT_RIGHT, LOW);
  delay(10);

  // Bring up and init FRONT
  digitalWrite(PIN_TOF_XSHUT_FRONT, HIGH);
  delay(10);
  if (!sensor_f.begin(TOF_ADDR_FRONT)) {
    Serial.println("Failed to boot FRONT ToF");
  } else {
    sensor_f.startRangeContinuous();
  }

  // Bring up and init FRONT LEFT
  digitalWrite(PIN_TOF_XSHUT_FRONT_LEFT, HIGH);
  delay(10);
  if (!sensor_fl.begin(TOF_ADDR_FRONT_LEFT)) {
    Serial.println("Failed to boot FRONT_LEFT ToF");
  } else {
    sensor_fl.startRangeContinuous();
  }

  // Bring up and init FRONT RIGHT
  digitalWrite(PIN_TOF_XSHUT_FRONT_RIGHT, HIGH);
  delay(10);
  if (!sensor_fr.begin(TOF_ADDR_FRONT_RIGHT)) {
    Serial.println("Failed to boot FRONT_RIGHT ToF");
  } else {
    sensor_fr.startRangeContinuous();
  }

  // Bring up and init LEFT
  digitalWrite(PIN_TOF_XSHUT_LEFT, HIGH);
  delay(10);
  if (!sensor_l.begin(TOF_ADDR_LEFT)) {
    Serial.println("Failed to boot LEFT ToF");
  } else {
    sensor_l.startRangeContinuous();
  }

  // Bring up and init RIGHT
  digitalWrite(PIN_TOF_XSHUT_RIGHT, HIGH);
  delay(10);
  if (!sensor_r.begin(TOF_ADDR_RIGHT)) {
    Serial.println("Failed to boot RIGHT ToF");
  } else {
    sensor_r.startRangeContinuous();
  }

  Serial.println("All sensors initialized. Reading raw values...");
}

void loop() {
  uint16_t f  = sensor_f.isRangeComplete() ? sensor_f.readRange() : 8190;
  uint16_t fl = sensor_fl.isRangeComplete() ? sensor_fl.readRange() : 8190;
  uint16_t fr = sensor_fr.isRangeComplete() ? sensor_fr.readRange() : 8190;
  uint16_t l  = sensor_l.isRangeComplete() ? sensor_l.readRange() : 8190;
  uint16_t r  = sensor_r.isRangeComplete() ? sensor_r.readRange() : 8190;

  Serial.print("F:"); Serial.print(f);
  Serial.print(" | FL:"); Serial.print(fl);
  Serial.print(" | FR:"); Serial.print(fr);
  Serial.print(" | L:"); Serial.print(l);
  Serial.print(" | R:"); Serial.println(r);

  // Update OLED
  display.clearDisplay();
  display.setCursor(0, 0);
  
  display.println("--- RAW TOF (mm) ---");
  
  display.print("FL: "); display.print(fl);
  display.print("  FR: "); display.println(fr);
  display.println();
  
  display.print("L:  "); display.print(l);
  display.print("  R:  "); display.println(r);
  display.println();
  
  display.print("FRONT: "); display.println(f);
  
  display.display();

  delay(50);
}
