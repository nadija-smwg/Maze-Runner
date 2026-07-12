#define PWMA   PA8
#define AIN1   PB12
#define AIN2   PB13

#define PWMB   PA9
#define BIN1   PB15
#define BIN2   PA10

#define STBY   PB14

void setupMotorDriver() {
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);
}

void setupPWM() {
  // Enable clocks
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

  // PA8 and PA9 -> Alternate Function
  GPIOA->MODER &= ~(3 << (8 * 2));
  GPIOA->MODER &= ~(3 << (9 * 2));

  GPIOA->MODER |= (2 << (8 * 2));
  GPIOA->MODER |= (2 << (9 * 2));

  // AF1 = TIM1
  GPIOA->AFR[1] &= ~(0xF << 0);
  GPIOA->AFR[1] &= ~(0xF << 4);

  GPIOA->AFR[1] |= (1 << 0);
  GPIOA->AFR[1] |= (1 << 4);

  // Reset timer
  TIM1->CR1 = 0;
  TIM1->CR2 = 0;
  TIM1->SMCR = 0;
  TIM1->DIER = 0;
  TIM1->CCER = 0;
  TIM1->CCMR1 = 0;
  TIM1->CCMR2 = 0;

  // 20 kHz PWM
  TIM1->PSC = 0;
  TIM1->ARR = 4999;
  TIM1->CNT = 0;

  // Duty cycle = 0%
  TIM1->CCR1 = 0;
  TIM1->CCR2 = 0;

  // PWM Mode 1
  TIM1->CCMR1 |= (6 << 4);
  TIM1->CCMR1 |= (6 << 12);

  // Enable preload
  TIM1->CCMR1 |= TIM_CCMR1_OC1PE;
  TIM1->CCMR1 |= TIM_CCMR1_OC2PE;

  // Enable outputs
  TIM1->CCER |= TIM_CCER_CC1E;
  TIM1->CCER |= TIM_CCER_CC2E;

  // Main output enable
  TIM1->BDTR |= TIM_BDTR_MOE;

  // Update registers
  TIM1->EGR = TIM_EGR_UG;

  // Auto-reload preload
  TIM1->CR1 |= TIM_CR1_ARPE;

  // Start timer
  TIM1->CR1 |= TIM_CR1_CEN;
}

void leftForward() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
}

void leftReverse() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
}

void leftBrake() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
}

void leftCoast() {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
}

void rightForward() {
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
}

void rightReverse() {
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
}

void rightBrake() {
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, HIGH);
}

void rightCoast() {
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}

void setLeftPWM(uint16_t pwm) {
  if (pwm > 4999) {
    pwm = 4999;
  }
  TIM1->CCR1 = pwm;
}

void setRightPWM(uint16_t pwm) {
  if (pwm > 4999) {
    pwm = 4999;
  }
  TIM1->CCR2 = pwm;
}

void setMotorPWM(uint16_t left, uint16_t right) {
  setLeftPWM(left);
  setRightPWM(right);
}

void setLeftMotor(int16_t pwm) {
  if (pwm > 4999) {
    pwm = 4999;
  }

  if (pwm < -4999) {
    pwm = -4999;
  }

  if (pwm > 0) {
    leftForward();
    TIM1->CCR1 = pwm;
  } else if (pwm < 0) {
    leftReverse();
    TIM1->CCR1 = -pwm;
  } else {
    leftBrake();
    TIM1->CCR1 = 0;
  }
}

void setRightMotor(int16_t pwm) {
  if (pwm > 4999) {
    pwm = 4999;
  }

  if (pwm < -4999) {
    pwm = -4999;
  }

  if (pwm > 0) {
    rightForward();
    TIM1->CCR2 = pwm;
  } else if (pwm < 0) {
    rightReverse();
    TIM1->CCR2 = -pwm;
  } else {
    rightBrake();
    TIM1->CCR2 = 0;
  }
}

void robotForward(uint16_t pwm) {
  setLeftMotor(pwm);
  setRightMotor(pwm);
}

void robotReverse(uint16_t pwm) {
  setLeftMotor(-pwm);
  setRightMotor(-pwm);
}

void robotTurnLeft(uint16_t pwm) {
  setLeftMotor(-pwm);
  setRightMotor(pwm);
}

void robotTurnRight(uint16_t pwm) {
  setLeftMotor(pwm);
  setRightMotor(-pwm);
}

void robotStop() {
  setLeftMotor(0);
  setRightMotor(0);
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  setupMotorDriver();
  setupPWM();
  Serial.println("Motor Driver Ready");
}

void loop() {
  Serial.println("Forward");
  robotForward(2500);
  delay(3000);

  robotStop();
  delay(1000);

  Serial.println("Reverse");
  robotReverse(2500);
  delay(3000);

  robotStop();
  delay(1000);

  Serial.println("Turn Left");
  robotTurnLeft(2500);
  delay(2000);

  robotStop();
  delay(1000);

  Serial.println("Turn Right");
  robotTurnRight(2500);
  delay(2000);

  robotStop();
  delay(2000);
}