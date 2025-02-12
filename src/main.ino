#include "controllers/pid_controller.ino"
#include "sensors/encoder.ino"
#include "actuators/motor_driver.ino"

void setup() {
    // Initialize serial communication
    Serial.begin(9600);

    // Initialize sensors and actuators
    initEncoder();
    initMotorDriver1();
    initMotorDriver2();
}

void loop() {
    // Read the current angle of the pendulum
    float angle = readEncoder();

    // Compute the control signal using the PID controller
    float controlSignal1 = computePID(angle);
    float controlSignal2 = computePID(angle); // Adjust as needed for the second motor

    // Apply the control signal to the motors
    setMotorSpeed1(controlSignal1);
    setMotorSpeed2(controlSignal2);

    // Add a small delay to avoid overwhelming the microcontroller
    delay(10);
}