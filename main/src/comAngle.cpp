#include "compAngle.h"
#include <Arduino.h>
#include <Wire.h>
#include <Arduino_BMI270_BMM150.h>  // Ensure this library is installed

// IMU Variables
float accelAngle = 0;          // Angle calculated using only the accelerometer
float gyroAngle;           // Angle calculated using only the gyroscope
float compAngle;           // Combined angle using complementary filter

float Kc = 0.6;  // Weight for accelerometer data (adjust as needed)

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
    // float prevAngle = gyroAngle;
    if (IMU.gyroscopeAvailable()) {
        // Compute tilt angle using the gyroscope
        float gx, gy, gz;
        IMU.readGyroscope(gx, gy, gz);
        float dt = (millis() - prevTime) / 1000.0;  // Time difference in seconds
        prevTime = millis();
        gyroAngle -= gx * dt;
    }
    return (gyroAngle);
}

float getCompAngle() {
    // Combine accelerometer and gyroscope readings using complementary filter
    compAngle = Kc * (compAngle + getGyroAngle() * ((millis() - prevTime) / 1000.0)) + (1 - Kc) * getAccelAngle();
    prevTime = millis();  // Update previous time for next calculation

    // Optional: Print angles for debugging
    Serial.print("Comp Angle: ");
    Serial.print(compAngle);
    Serial.print(" | ");
    Serial.print("Gyro Angle: ");
    Serial.print(gyroAngle);
    Serial.print(" | ");
    Serial.print("Accel Angle: ");
    Serial.println(accelAngle);
    
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