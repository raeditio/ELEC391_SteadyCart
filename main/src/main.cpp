#include <Arduino.h>
#include "compAngle.h"

#define leftForward D2
#define leftReverse D3
#define rightForward D4
#define rightReverse D5

const int motors[] = {leftForward, rightForward, leftReverse, rightReverse};
const int forward[] = {leftForward, rightForward};
const int reverse[] = {leftReverse, rightReverse};

int task = 0;
int pwm = 0;
// float targetAngle = 0;  // Desired angle (e.g., keep at level 0°)

void setup() {
    Serial.begin(115200);
    for (int motor : motors) {
        pinMode(motor, OUTPUT);
    }
    Serial.println("Motor control initialized");

    // Initialize IMU
    initIMU();
}

void loop() {
    float currentAngle = getCompAngle();  // Get tilt angle

    // Compute PID-controlled motor speed
    int speed = computePID(currentAngle);
    // int speed = 0;

    // Serial.print("Target: ");
    // Serial.print(targetAngle);
    // Serial.print("°, Current: ");
    // Serial.print(currentAngle, 2);
    Serial.print(", Adjusted PWM: ");
    if (currentAngle > 0) Serial.print("-");
    Serial.println(speed);

    // If angle is positive, move forward; if negative, move in reverse
    if (currentAngle < 0) {
        analogWrite(leftForward, speed);
        analogWrite(rightForward, speed);
        analogWrite(leftReverse, 0);
        analogWrite(rightReverse, 0);
    } else if (currentAngle > 0) {
        analogWrite(leftReverse, speed);
        analogWrite(rightReverse, speed);
        analogWrite(leftForward, 0);
        analogWrite(rightForward, 0);
    // } else {
    //     // Stop motors if the angle is near 0
    //     analogWrite(leftForward, 0);
    //     analogWrite(rightForward, 0);
    //     analogWrite(leftReverse, 0);
    //     analogWrite(rightReverse, 0);
    }

    //delay(100);  // Short delay for stability
}
