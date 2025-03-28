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
volatile long encoderTicks = 0;            // Total encoder ticks
long prevEncoderTicks = 0;                 // Previous encoder ticks for velocity calculation
const float DEG_PER_TICK = 360.0 / 600.0;  // 600 PPR encoder, 0.6° per tick
float theta = 0.0;                         // Pendulum angle in degrees
float theta_d = 0.0;                       // Pendulum angular velocity (degrees/s)

// Control gains (tune these values for your system)
const float kP = 0.953;   // Proportional gain - 0.953
const float kI = 0.0013;  // Integral gain - 0.0013
const float kD = 0.0;    // Derivative gain
const float BIAS = 0.0;  // Bias term

// Time step for control loop
unsigned long lastTime = 0;
const float dt = 0.002;  // 10ms control loop (100Hz)

// Target equilibrium angle (in degrees)
const float theta_target = 180;  // Upright position, 180 degrees

// Variables for storing metrics
float intervals[1000];  // Interval storage (for debugging)
float enc_degs[1000];   // Encoder degrees storage
float move_degs[1000];  // Motor movement degrees

// PID variables
float error_prev = 0;
float integral = 0;

// Function to control the motor with PWM signal
void motorCommand(double PWMcommand) {
  PWMcommand = round(255 * PWMcommand / 12);  // Scale PWM command to [0, 255]

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

// Interrupt Service Routine for Encoder Channel A
void encoderISR() {
  if (digitalRead(encoderPinB) == LOW) {
    encoderTicks++;  // Increment for one direction
  } else {
    encoderTicks--;  // Decrement for the opposite direction
  }
}

void setup() {
  Serial.begin(9600);  // Start serial communication for debugging

  pinMode(encoderPinA, INPUT_PULLUP);  // Set encoder pin A as input with pull-up resistor
  pinMode(encoderPinB, INPUT_PULLUP);  // Set encoder pin B as input with pull-up resistor

  // Attach interrupt to encoder pin A (on rising edge)
  attachInterrupt(digitalPinToInterrupt(encoderPinA), encoderISR, RISING);

  lcd.init();
  lcd.backlight();

  // Set motor control pins as output
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(motorPinE, OUTPUT);
  lastTime = millis();  // Initialize the last time to track control loop timing
}

void loop() {
  static bool startDelay = true;
  static unsigned long startTime = millis();

  // Sensing phase: Calculate pendulum angle (theta) and angular velocity (theta_d)
  theta = encoderTicks * DEG_PER_TICK;  // Convert ticks to degrees

  if (startDelay) {
    // Calculate the remaining countdown time
    int countdown = 5 - (millis() - startTime) / 1000;

    // Display countdown and angle
    lcd.setCursor(0, 0);
    lcd.print("Get ready: ");
    lcd.print(countdown);
    lcd.print("s ");
    lcd.setCursor(0, 1);
    lcd.print("Angle: ");
    lcd.print(theta, 2);

    // Keep checking the time until 5 seconds pass
    if (millis() - startTime >= 5000) {
      lcd.clear();
      startDelay = false;
    }
    return;
  }

  unsigned long currentTime = millis();
  float timeElapsed = (currentTime - lastTime) / 1000.0;  // Time in seconds

  theta_d = (encoderTicks - prevEncoderTicks) * DEG_PER_TICK / timeElapsed;  // Calculate angular velocity

  // Control phase: Apply PD control to balance the pendulum at theta_target
  // Calculate error in angle and angular velocity
  float error = theta_target - theta;                     // Deviation from target angle (180 degrees)
  integral += error * timeElapsed;                        // Integral term
  float derivative = (error - error_prev) / timeElapsed;  // Derivative term

  // Apply PID control law: u = Kp * error + Ki * integral + Kd * derivative + bias
  float u = (kP * error) + (kI * integral) + (kD * derivative) + BIAS;

  // Apply deadzone to prevent jittering near the target angle
  if (abs(error) < 0.6) {
    u = 0;  // No motor command if within deadzone
  }

  // Limit the motor command to a safe range
  if (u > 255) u = 255;
  if (u < -255) u = -255;

  // Send control signal to the motor
  motorCommand(u);  // Apply the control input to the motor

  // Update previous encoder ticks for next iteration
  prevEncoderTicks = encoderTicks;

  // Store data for debugging/metrics
  intervals[encoderTicks] = timeElapsed * 1000;  // Store interval in milliseconds
  enc_degs[encoderTicks] = theta;
  move_degs[encoderTicks] = u;

  // Print relevant data for plotting in Serial Plotter format
  Serial.print("Angle:");
  Serial.print(theta);
  Serial.print(", ");
  Serial.print("PWM:");
  Serial.print(u);
  Serial.println(", ");

  // Display the angle on the LCD
  lcd.setCursor(0, 0);
  lcd.print("Angle: ");
  lcd.setCursor(7, 0);
  lcd.print(theta, 2);

  // Timing: Wait to maintain the control loop at a fixed rate (100Hz)
  lastTime = currentTime;
  delay(dt * 1000);  // Delay for the desired control loop period
}
