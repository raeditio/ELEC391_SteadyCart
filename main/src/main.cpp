#include <Arduino.h>
#include <PID_v1.h>
#include "compAngle.h"

#define leftForward D2
#define leftReverse D3
#define rightForward D5
#define rightReverse D4

const int motors[] = {leftForward, rightForward, leftReverse, rightReverse};
const int forward[] = {leftForward, rightForward};
const int reverse[] = {leftReverse, rightReverse};

double setpoint = 1.6;  // Desired angle (0 degrees)
double input, output;
// double Kp = 6;  // Proportional gain
// double Ki = 0.15;  // Integral gain
// double Kd = 0.05; // Derivative gain
double Kp = 3.75; // 7
double Ki = 0.075; // 0.5
double Kd = 0.57;
double gain = 10;
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

unsigned long lastPIDUpdate = 0;
const int interval = 10;  // PID update interval in milliseconds

int speed = 40;  // Motor speed (0-255)
int sign = 0;  // Sign of the angle (1 for forward, -1 for reverse)
double currentAngle = 0;  // Current angle from the IMU
double prevAngle = 0;  // Previous angle for PID calculation

// Function to calculate PWM from RPM using the given polynomial equation
// int rpm2pwm(float rpm) {
//     // return 0.000041559 * pow(rpm, 3) - 0.0347502552 * pow(rpm, 2) + 9.8233911975 * rpm - 860.4124730999;
//     int pwm = 0.000675698364746 * pow(rpm, 2.07915394237297); // Exponential equation
//     return pwm;  // Ensure PWM is within valid range (0-255)
// }

void PIDLoop() {
    unsigned long currentTime = millis();
    if (currentTime - lastPIDUpdate >= interval) {
        lastPIDUpdate = currentTime;

        // Get the current angle from the IMU
        currentAngle = getCompAngle();  // Get tilt angle

        // Determine sign from real angle (negative or positive)
        if (currentAngle > 0) {
            sign = 1;
        } else if (currentAngle < 0) {
            sign = -1;
        } else {
            sign = 0;
        }

        currentAngle = abs(currentAngle);

        // Compute PID output
        input = setpoint - currentAngle;
        myPID.Compute();

        // Adjust motor speed based on PID output
        speed = (int)output;
        speed = constrain(speed * gain, 0, 255);
        Serial.print("Current Angle: ");
        Serial.print(currentAngle);
    }
    Serial.print(", PID Output: ");
}

void motorControl() {
    // Set motor speed and direction based on PID output
    if (sign < 0) {
        analogWrite(leftForward, speed);
        analogWrite(rightForward, speed);
        analogWrite(leftReverse, 0);
        analogWrite(rightReverse, 0);
    } else if (sign > 0) {
        analogWrite(leftForward, 0);
        analogWrite(rightForward, 0);
        analogWrite(leftReverse, speed);
        analogWrite(rightReverse, speed);
    } else {
        // Stop motors if output is zero
        for (int i = 0; i < 4; i++) {
            digitalWrite(motors[i], LOW);
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(100);  // Short delay to allow serial connection to initialize
    Serial.println("Initializing IMU...");
    initIMU();  // Initialize IMU

    // Set motor pins as outputs
    for (int i = 0; i < 4; i++) {
        pinMode(motors[i], OUTPUT);
        digitalWrite(motors[i], LOW);  // Ensure motors are off initially
    }

    myPID.SetMode(AUTOMATIC);  // Set PID mode to automatic
    myPID.SetOutputLimits(-10000, 10000);
}

void loop() {
    PIDLoop();  // Update PID loop
    motorControl();  // Control motors based on PID output

    // Print current angle and speed for debugging
    Serial.print("Speed: ");
    Serial.println(sign * speed);  // Print speed with sign
    delay(100);  // Short delay for readability
}


// void setup() {
//     Serial.begin(115200);
//     Serial.println("RightForward");
//     analogWrite(rightForward, 255);
//     delay(1000);
//     analogWrite(rightForward, 0);
//     delay(1000);
//     Serial.println("LeftForward");
//     analogWrite(leftForward, 255);
//     delay(1000);
//     analogWrite(leftForward, 0);
//     delay(1000);
//     Serial.println("RightReverse");
//     analogWrite(rightReverse, 255);
//     delay(1000);
//     analogWrite(rightReverse, 0);
//     delay(1000);
//     Serial.println("LeftReverse");
//     analogWrite(leftReverse, 255);
//     delay(1000);
//     analogWrite(leftReverse, 0);
//     delay(1000);
// }

// void loop() {
//     // Nothing to do here
// }