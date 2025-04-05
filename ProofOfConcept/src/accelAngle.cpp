#include "accelAngle.h"
#include <Arduino.h>
#include <Wire.h>
#include <Arduino_BMI270_BMM150.h>  // Ensure this library is installed
#include <math.h>

// IMU Variables
float accelAngle = 0;          // Angle calculated using only the accelerometer
float initialAngleOffset = 0;  // Offset to subtract from readings

void initIMU() {
    Serial.begin(115200);
    Serial.println("Initializing IMU...");
    while (!Serial);

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
    initialAngleOffset = -atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI; // Corrected formula for pitch
    Serial.print("Initial Angle Offset: ");
    Serial.println(initialAngleOffset);
}

float getAccelAngle() {
    if (IMU.accelerationAvailable()) {
        float ax, ay, az;
        IMU.readAcceleration(ax, ay, az);

        // Compute tilt angle directly using the accelerometer
        accelAngle = -atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;

        // Subtract the initial offset to normalize readings
        accelAngle -= initialAngleOffset;

        // Debug output
        // Serial.print("Accelerometer Angle (Offset Applied): ");
        // Serial.println(accelAngle);
    }

    return accelAngle;
}