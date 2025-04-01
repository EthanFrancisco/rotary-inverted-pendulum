#include <LiquidCrystal_I2C.h>
#include <Wire.h>

// Encoder Pins
#define encoderPinA 2  // Encoder Channel A
#define encoderPinB 3  // Encoder Channel B

// Motor control pins
#define motorPin1 10
#define motorPin2 9
#define motorPinE 11

// LCD Setup
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Encoder variables
volatile long encoderTicks = 0;
long prevEncoderTicks = 0;
const float DEG_PER_TICK = 360.0 / 600.0;
float theta = 0.0;
float theta_d = 0.0;

// LQR Gain Values
const float K1 = 34.1385;
const float K2 = 9.9697;

// Time step for control loop
unsigned long lastTime = 0;
const float dt = 0.002;  // 2ms control loop (500Hz)

// Target equilibrium angle (in degrees)
const float theta_target = 180;

// Interrupt Service Routine for Encoder
void encoderISR() {
  if (digitalRead(encoderPinB) == LOW) {
    encoderTicks++;
  } else {
    encoderTicks--;
  }
}

void motorCommand(double PWMcommand) {
  PWMcommand = round(255 * PWMcommand / 12);

  if (PWMcommand > 255) PWMcommand = 255;
  if (PWMcommand < -255) PWMcommand = -255;

  if (PWMcommand >= 0) {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
  } else {
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
  }
  analogWrite(motorPinE, abs(PWMcommand));
}

void setup() {
  Serial.begin(9600);

  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderISR, RISING);

  lcd.init();
  lcd.backlight();

  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPinE, OUTPUT);
  lastTime = millis();
}

void loop() {
  static bool startDelay = true;
  static unsigned long startTime = millis();

  theta = encoderTicks * DEG_PER_TICK;

  if (startDelay) {
    int countdown = 5 - (millis() - startTime) / 1000;
    lcd.setCursor(0, 0);
    lcd.print("Get ready: ");
    lcd.print(countdown);
    lcd.print("s ");
    lcd.setCursor(0, 1);
    lcd.print("Angle: ");
    lcd.print(theta, 2);

    if (millis() - startTime >= 5000) {
      lcd.clear();
      startDelay = false;
    }
    return;
  }

  unsigned long currentTime = millis();
  float timeElapsed = (currentTime - lastTime) / 1000.0;

  theta_d = (encoderTicks - prevEncoderTicks) * DEG_PER_TICK / timeElapsed;

  float u = -(K1 * (theta - theta_target) + K2 * theta_d);

  motorCommand(u);

  prevEncoderTicks = encoderTicks;

  lcd.setCursor(0, 0);
  lcd.print("Angle: ");
  lcd.setCursor(7, 0);
  lcd.print(theta, 2);

  Serial.print("Angle:");
  Serial.print(theta);
  Serial.print(", PWM:");
  Serial.println(u);

  lastTime = currentTime;
  delay(dt * 1000);
}
