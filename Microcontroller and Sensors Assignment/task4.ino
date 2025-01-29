#include "Arduino_BMI270_BMM150.h"
#include <math.h>

float accelAngle = 0;
float gyroAngle = 0;
float compAngle = 0;
float gyroRate = 0;
float k = 0.98; // Adjust this based on tuning
unsigned long previous_time

void setup() {
  Serial.begin(115200);
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  previous_time = millis();
}

void loop() {
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    float ax, ay, az, gx, gy, gz;
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);

    // Compute accelerometer angle
    accelAngle = atan2(sqrt(ax * ax + ay * ay), az) * 180.0 / PI;

    // Time difference in seconds
    unsigned long previous_time= 0;
    unsigned long current_time = millis();
    float dt = (current_time - previous_time) / 1000.0;

    // Compute gyroscope angle
    gz -= gz * 180.0 / PI;
    gyroAngle += gz * dt;
    previous_time = current_time;

    // Compute complementary filter angle
    compAngle = k * (compAngle + gyroAngle) + (1 - k) * accelAngle;

    // Send data over serial
    Serial.println(compAngle);
    Serial.print(",");
    Serial.print(accelAngle);
    Serial.print(",");
    Serial.print(gyroAngle);
  }
}