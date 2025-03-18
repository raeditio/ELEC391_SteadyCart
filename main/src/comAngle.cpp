#include "compAngle.h"
#include <Arduino.h>
#include <Wire.h>
#include <Arduino_BMI270_BMM150.h>  // Ensure this library is installed
#include <math.h>

// IMU Variables
float accelAngle = 0;          // Angle calculated using only the accelerometer
float gyroAngle;           // Angle calculated using only the gyroscope
float compAngle;           // Combined angle using complementary filter

float Kc = 0.95;  // Weight for accelerometer data (adjust as needed)

// PID Variables
// float Kp = 1.06;   // Proportional Gain (Adjust for faster/slower response)
// float Ki = 1.33;  // Integral Gain (Can be set to 0 if not needed)
// float Kd = 0.87;   // Derivative Gain (Smooths sudden changes)
float Kp = 1160;
float Ki = 4330;
float Kd = 0;
float prevError = 0;
float integral = 0;
unsigned long prevTime = 0;

void initIMU() {
    Serial.begin(115200);
    delay(100);  // Short delay to allow serial connection to initialize
    Serial.println("Initializing IMU...");

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
    accelAngle = atan2(ay, az) * 180.0 / PI;
    gyroAngle = accelAngle;  // Set initial gyroAngle as accelAngle
    compAngle = accelAngle;  // Set initial compAngle as accelAngle

    prevTime = millis();  // Initialize time for PID
}

float getAccelAngle() {
    if (IMU.accelerationAvailable()) {
        float ax, ay, az;
        IMU.readAcceleration(ax, ay, az);

        // Compute tilt angle directly using the accelerometer
        accelAngle = atan2(ay, az) * 180.0 / PI;
    }
    return accelAngle;
}

float getGyroAngle() {
    float prevAngle = gyroAngle;
    if (IMU.gyroscopeAvailable()) {
        // Compute tilt angle using the gyroscope
        float gx, gy, gz;
        IMU.readGyroscope(gx, gy, gz);
        float dt = (millis() - prevTime) / 1000.0;  // Time difference in seconds
        prevTime = millis();
        gyroAngle -= gx * dt;
    }
    return (prevAngle - gyroAngle);
}

float getCompAngle() {
    // Combine accelerometer and gyroscope readings using complementary filter
    compAngle = Kc * (compAngle + getGyroAngle()) + (1 - Kc) * getAccelAngle();
    
    Serial.print("Comp Angle: ");
    Serial.print(compAngle);
    Serial.print(" | ");
    Serial.print("Gyro Angle: ");
    Serial.print(gyroAngle);
    Serial.print(" | ");
    Serial.print("Accel Angle: ");
    Serial.println(accelAngle);
    return compAngle;
}

// Function to calculate PWM from RPM using the given polynomial equation
int rpm2pwm(float rpm) {
    return 0.000041559 * pow(rpm, 3) - 0.0347502552 * pow(rpm, 2) + 9.8233911975 * rpm - 860.4124730999;
}

// PID Controller Function
int computePID(float currentAngle) {
    currentAngle = currentAngle * PI / 180;  // Convert to radians (absolute value)
    unsigned long currentTime = millis();
    float dt = (currentTime - prevTime) / 1000.0;  // Time difference in seconds
    if (dt < 0.001) dt = 0.001;  // Prevent division by zero
    prevTime = currentTime;

    float error = currentAngle;  // Difference between target and current angle
    integral += error * dt;   // Integral term (accumulates small errors)

    float derivative = (error - prevError) / dt;  // Derivative term (smooths response)
    prevError = error;  // Store error for next iteration

    // Compute PID output (Correction Force)
    float Force = (Kp * error) + (Ki * integral) + (Kd * derivative);

    // Print PID values for debugging
    Serial.print("P: ");
    Serial.print(Kp * error);
    Serial.print(" | I: ");
    Serial.print(Ki * integral);
    Serial.print(" | D: ");
    Serial.println(Kd * derivative);

    // Compute required acceleration
    int M = 4;  // Mass of the Cart (kg)
    float acceleration = Force / M;

    // Angular acceleration = Linear acceleration / Radius of the wheel
    // Angular velocity = Angular acceleration * Time
    // Thus, RPM = Angular velocity * 60 / (2 * PI)
    // RPM = (acceleration / radius * t) * 60 / (2 * PI)
    float radius = 0.04;  // Radius of the wheel (m)

    // Calculate the desired RPM based on the acceleration
    float desiredRPM = abs((acceleration / radius * dt) * 60 / (2 * PI));

    // print the desired RPM
    Serial.print("Desired RPM: ");
    Serial.print(desiredRPM);

    // Calculate the corresponding PWM value using the polynomial equation
    int pwmValue = rpm2pwm(desiredRPM);

    // Constrain PWM value to valid range (0-255)
    // pwmValue = constrain(pwmValue, 0, 255);
    return constrain(pwmValue, 0, 255);
}
