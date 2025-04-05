#include "Arduino_BMI270_BMM150.h"
#include <math.h>

unsigned long previous_time;
float pitch_angle = 0;
float gyro_y_offset = 0;
float alpha = 0.98; // Complementary filter coefficient

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Started");

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

  gyro_y_offset = sum_y / samples;
  Serial.print("Gyro Y Offset: ");
  Serial.println(gyro_y_offset, 6);

  Serial.println("X\tY\tZ\tPitch Angle");
  previous_time = micros();
}

void loop() {
  float x, y, z, ax, ay, az;

  if (IMU.gyroscopeAvailable() && IMU.accelerationAvailable()) {
    IMU.readGyroscope(x, y, z);
    IMU.readAcceleration(ax, ay, az);

    y = (y - gyro_y_offset) * 180.0 / PI; // Convert to degrees/sec and remove offset

    unsigned long current_time = micros();
    float delta_T = (current_time - previous_time) / 1000000.0; // Convert to seconds
    previous_time = current_time;

    // Gyroscope integration
    float gyro_angle = pitch_angle + (y * delta_T);

    // Accelerometer-based angle estimation
    float accel_angle = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI;

    // Complementary filter: Combine gyroscope and accelerometer readings
    pitch_angle = alpha * gyro_angle + (1 - alpha) * accel_angle;

    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.print(z);
    Serial.print('\t');
    Serial.println(pitch_angle);
  }
}