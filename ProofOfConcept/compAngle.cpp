#include "CompAngle.h"
#include <Arduino.h>
#include <Wire.h>
#include <Arduino_BMI270_BMM150.h>  // Ensure this library is installed
#include <math.h>

// IMU Variables
float accelAngle = 0;
float gyroAngle = 0;
float compAngle = 0;
float k = 0.95;  // Give more weight to accelerometer to reduce drift
unsigned long previous_time;
float gyro_y_offset = 0;  // Bias correction for Y-axis

void initIMU() {
    Serial.begin(115200);
    Serial.println("Initializing IMU...");
    while (!Serial);

    if (!IMU.begin()) {
        Serial.println("Failed to initialize IMU!");
        while (1);
    }

    Serial.println("Calibrating gyroscope...");
    int samples = 2000;  // Increase samples for better accuracy
    float sum_y = 0;

    // Calculate bias correction for gyroscope
    for (int i = 0; i < samples; i++) {
        float x, y, z;
        while (!IMU.gyroscopeAvailable());
        IMU.readGyroscope(x, y, z);
        sum_y += y;
        delay(1);
    }
    gyro_y_offset = sum_y / samples;  // Compute bias correction
    Serial.print("Gyro Y Offset: ");
    Serial.println(gyro_y_offset, 6);

    previous_time = millis();

    // Initialize complementary filter with accelerometer angle
    float ax, ay, az;
    while (!IMU.accelerationAvailable());
    IMU.readAcceleration(ax, ay, az);
    compAngle = -atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI; // Corrected formula for pitch
}

float getCompAngle() {
    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
        float ax, ay, az, gx, gy, gz;
        IMU.readAcceleration(ax, ay, az);
        IMU.readGyroscope(gx, gy, gz);

        // Compute accelerometer angle for pitch
        accelAngle = -atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;

        // Time difference in seconds
        unsigned long current_time = millis();
        float dt = (current_time - previous_time) / 1000.0;
        previous_time = current_time; // Update time for next loop

        // Apply bias correction and convert gyroscope data to degrees/sec
        gy = (gy - gyro_y_offset) * 180.0 / PI;

        // Apply dead zone to ignore small drift
        if (abs(gy) < 0.05) gy = 0;

        // Integrate gyroscope angle
        gyroAngle += gy * dt;

        // Apply complementary filter
        compAngle = k * (compAngle + gy * dt) + (1 - k) * accelAngle;

        // **Active Gyro Drift Resetting**: If stationary for a while, slowly adjust drift
        static unsigned long last_reset_time = millis();
        if (millis() - last_reset_time > 10000 && abs(gy) < 0.05) { 
            gyro_y_offset = gyro_y_offset * 0.99 + gy * 0.01; // Adapt bias dynamically
            gyroAngle = accelAngle;  // Reset drift accumulation
            last_reset_time = millis();
        }

        // Debug output
        Serial.print("Complementary Angle: ");
        Serial.print(compAngle);
        Serial.print(", Accel Angle: ");
        Serial.print(accelAngle);
        Serial.print(", Gyro Angle: ");
        Serial.println(gyroAngle);
    }

    return compAngle;
}
