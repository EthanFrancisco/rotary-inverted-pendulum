#include "controllers/pid_controller.ino"
#include "sensors/encoder.ino"
#include "actuators/motor_driver.ino"

void setup() {
    // Initialize serial communication
    Serial.begin(9600);

    // Initialize sensors and actuators
    initEncoder();
    initMotorDriver();
}

void loop() {
    // Read the current angle of the pendulum
    float angle = readEncoder();

    // Compute the control signal using the PID controller
    float controlSignal = computerPID(angle);

    // Apply the control signal to the motor
    setMotorSpeed(controlSignal);

    // Add a small delay to avoid overwhelming the microcontroller
    delay(10);
}