#include "Arduino_BMI270_BMM150.h"
#include <math.h>

float accelAngle = 0;
float gyroAngle = 0;
float compAngle = 0;
float k = 0.96; // Lower k slightly to reduce drift
unsigned long previous_time;
float gyro_y_offset = 0; // Bias correction for Y-axis

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.println("Calibrating gyroscope...");
  int samples = 1000;
  float sum_y = 0;

  for (int i = 0; i < samples; i++) {
    float x, y, z;
    while (!IMU.gyroscopeAvailable());
    IMU.readGyroscope(x, y, z);
    sum_y += y;
    delay(2);
  }

  gyro_y_offset = sum_y / samples; // Compute Y-axis bias
  Serial.print("Gyro Y Offset: ");
  Serial.println(gyro_y_offset, 6);

  previous_time = millis();

  // Initialize complementary filter with accelerometer angle
  float ax, ay, az;
  while (!IMU.accelerationAvailable());
  IMU.readAcceleration(ax, ay, az);
  compAngle = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI; // Corrected formula for pitch
}

void loop() {
  if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
    float ax, ay, az, gx, gy, gz;
    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);

    // Compute accelerometer angle for pitch
    accelAngle = atan2(ay, sqrt(ax * ax + az * az)) * 180.0 / PI;

    // Time difference in seconds
    unsigned long current_time = millis();
    float dt = (current_time - previous_time) / 1000.0;
    previous_time = current_time; // Update time for next loop

    // Compute gyroscope angle (use Y-axis and apply bias correction)
    gy = (gy - gyro_y_offset) * 180.0 / PI; // Convert to degrees/sec
    gyroAngle += gy * dt; // Integrate angular velocity

    // Dynamic gyro drift correction when stationary
    if (abs(gy) < 0.05) { 
        gyro_y_offset = 0.99 * gyro_y_offset + 0.01 * gy; // Slow bias adaptation
    }

    // Compute complementary filter angle
    compAngle = k * (compAngle + gy * dt) + (1 - k) * accelAngle;

    // Periodic drift reset (every 30 seconds)
    static unsigned long last_correction_time = 0;
    if (millis() - last_correction_time > 30000) { 
        gyroAngle = accelAngle; // Reset drift accumulation
        last_correction_time = millis();
    }

    // Send data over serial
    Serial.print("Complementary Angle: ");
    Serial.print(compAngle);
    Serial.print(", Accel Angle: ");
    Serial.print(accelAngle);
    Serial.print(", Gyro Angle: ");
    Serial.println(gyroAngle);
  }
}
