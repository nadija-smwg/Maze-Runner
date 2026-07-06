#define PWMA  PA8
#define AIN1  PB12
#define AIN2  PB13

#define PWMB  PA9
#define BIN1  PB15
#define BIN2  PA10

#define STBY  PB14

void setupMotorDriver()
{
  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);
}

void motorAForward(uint8_t pwm)
{
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, pwm);
}

void motorAReverse(uint8_t pwm)
{
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  analogWrite(PWMA, pwm);
}

void motorBForward(uint8_t pwm)
{
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, pwm);
}

void motorBReverse(uint8_t pwm)
{
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  analogWrite(PWMB, pwm);
}

void motorAStop()
{
  analogWrite(PWMA, 0);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
}

void motorBStop()
{
  analogWrite(PWMB, 0);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  setupMotorDriver();
  setupPWM();
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Forward");
  motorAForward(100);
  motorBForward(100);
  delay(3000);

  Serial.println("Stop");
  motorAStop();
  motorBStop();
  delay(1000);

  Serial.println("Reverse");
  motorAReverse(100);
  motorBReverse(100);
  delay(3000);

  Serial.println("Stop");
  motorAStop();
  motorBStop();
  delay(1000);
}
