#include "accelAngle.h"
#include <Arduino.h>
#include <Wire.h>
#include <Arduino_BMI270_BMM150.h>  // Ensure this library is installed
#include <math.h>

// IMU Variables
float accelAngle = 0;          // Angle calculated using only the accelerometer
float initialAngleOffset = 0;  // Offset to subtract from readings

// PID Variables
float Kp = 642.4256;   // Proportional Gain (Adjust for faster/slower response) (kcrit = 30)
float Ki = 926.2478;  // Integral Gain (Can be set to 0 if not needed)
float Kd = 141.3932;   // Derivative Gain (Smooths sudden changes)
float prevError = 0;
float integral = 0;
unsigned long prevTime = 0;

void initIMU() {
    Serial.begin(115200);
    Serial.println("Initializing IMU...");
    // while (!Serial);

    if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
        while (1);
    }

    Serial.println("IMU Initialized.");

    // Read initial accelerometer data to determine the starting angle
    float ax, ay, az;
    while (!IMU.accelerationAvailable());
    IMU.readAcceleration(ax, ay, az);

    // Compute initial tilt angle
    // initialAngleOffset = -atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI; // Corrected formula for pitch
    // Serial.print("Initial Angle Offset: ");
    // Serial.println(initialAngleOffset);

    prevTime = millis();  // Initialize time for PID
}

float getAccelAngle() {
    if (IMU.accelerationAvailable()) {
        float ax, ay, az;
        IMU.readAcceleration(ax, ay, az);

        // Compute tilt angle directly using the accelerometer
        accelAngle = -atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;

        // Subtract the initial offset to normalize readings
        // accelAngle -= initialAngleOffset;

        // Debug output
        Serial.print("Accelerometer Angle (Offset Applied): ");
        Serial.println(accelAngle);
    }

    return accelAngle;
}

// Function to calculate PWM from RPM using the given polynomial equation
int rpm2pwm(float rpm) {
    return 6e-05 * pow(rpm, 3) - 0.0358 * pow(rpm, 2) + 7.1803 * rpm - 83.148;
}

// PID Controller Function
int computePID(float targetAngle, float currentAngle) {
    unsigned long currentTime = millis();
    float dt = (currentTime - prevTime) / 1000.0;  // Time difference in seconds
    prevTime = currentTime;

    float error = targetAngle - currentAngle;  // Difference between target and current angle
    integral += error * dt;   // Integral term (accumulates small errors)
    float derivative = (error - prevError) / dt;  // Derivative term (smooths response)
    prevError = error;  // Store error for next iteration

    // Compute PID output (desired RPM)
    float desiredRPM = (Kp * error) + (Ki * integral) + (Kd * derivative);

    // Calculate the corresponding PWM value using the polynomial equation
    int pwmValue = rpm2pwm(desiredRPM);

    // Constrain PWM value to valid range (0-255)
    pwmValue = constrain(pwmValue, 0, 255);
    return pwmValue;
}
