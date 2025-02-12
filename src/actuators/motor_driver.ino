// Define motor driver pins for Arduino Nano
const int motor1PWMPin = 3; // Motor 1 PWM pin
const int motor1AIN1Pin = 4; // Motor 1 AIN1 pin
const int motor1AIN2Pin = 5; // Motor 1 AIN2 pin

const int motor2PWMPin = 6; // Motor 2 PWM pin
const int motor2BIN1Pin = 7; // Motor 2 BIN1 pin
const int motor2BIN2Pin = 8; // Motor 2 BIN2 pin

void initMotorDriver1() {
    // Initialize the first motor driver
    pinMode(motor1PWMPin, OUTPUT);
    pinMode(motor1AIN1Pin, OUTPUT);
    pinMode(motor1AIN2Pin, OUTPUT);
}

void initMotorDriver2() {
    // Initialize the second motor driver
    pinMode(motor2PWMPin, OUTPUT);
    pinMode(motor2BIN1Pin, OUTPUT);
    pinMode(motor2BIN2Pin, OUTPUT);
}

void setMotorSpeed1(float speed) {
    // Set the speed for the first motor
    if (speed > 0) {
        digitalWrite(motor1AIN1Pin, HIGH);
        digitalWrite(motor1AIN2Pin, LOW);
    } else {
        digitalWrite(motor1AIN1Pin, LOW);
        digitalWrite(motor1AIN2Pin, HIGH);
        speed = -speed;
    }
    analogWrite(motor1PWMPin, speed);
}

void setMotorSpeed2(float speed) {
    // Set the speed for the second motor
    if (speed > 0) {
        digitalWrite(motor2BIN1Pin, HIGH);
        digitalWrite(motor2BIN2Pin, LOW);
    } else {
        digitalWrite(motor2BIN1Pin, LOW);
        digitalWrite(motor2BIN2Pin, HIGH);
        speed = -speed;
    }
    analogWrite(motor2PWMPin, speed);
}