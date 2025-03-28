#include "controllers/pid_controller.ino"
#include "sensors/encoder.ino"
#include "actuators/motor_driver.ino"

// Define motor and encoder pins for Arduino Nano
const int motor1Pin1 = 3; // Motor 1 positive pin
const int motor1Pin2 = 4; // Motor 1 negative pin
const int motor2Pin1 = 5; // Motor 2 positive pin
const int motor2Pin2 = 6; // Motor 2 negative pin

const int encoder1PinA = 2; // Encoder 1 A phase pin
const int encoder1PinB = 7; // Encoder 1 B phase pin
const int encoder2PinA = 8; // Encoder 2 A phase pin
const int encoder2PinB = 9; // Encoder 2 B phase pin

void setup() {
    // Initialize serial communication
    Serial.begin(9600);

    // Initialize motor pins
    pinMode(motor1Pin1, OUTPUT);
    pinMode(motor1Pin2, OUTPUT);
    pinMode(motor2Pin1, OUTPUT);
    pinMode(motor2Pin2, OUTPUT);

    // Initialize encoder pins
    pinMode(encoder1PinA, INPUT);
    pinMode(encoder1PinB, INPUT);
    pinMode(encoder2PinA, INPUT);
    pinMode(encoder2PinB, INPUT);

    // Initialize sensors and actuators
    initEncoder();
    initMotorDriver1();
    initMotorDriver2();
}

void loop() {
    // Read the current angle of the pendulum
    float angle1 = readEncoder1();
    float angle2 = readEncoder2();

    // Compute the control signal using the PID controller
    float controlSignal1 = computePID(angle1);
    float controlSignal2 = computePID(angle2); // Adjust as needed for the second motor

    // Apply the control signal to the motors
    setMotorSpeed1(controlSignal1);
    setMotorSpeed2(controlSignal2);

    // Add a small delay to avoid overwhelming the microcontroller
    delay(10);
}