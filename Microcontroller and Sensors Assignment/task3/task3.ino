#include "Arduino_BMI270_BMM150.h"
#include <math.h>

unsigned long previous_time;
float angle = 0;
float gyro_z_offset = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  
  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");

  Serial.println("Calibrating gyroscope...");
  int samples = 500;
  float sum_z = 0;

  for (int i = 0; i < samples; i++) {
    float x, y, z;
    while (!IMU.gyroscopeAvailable());  // Wait for new data
    IMU.readGyroscope(x, y, z);
    sum_z += z;
    delay(5);
  }
  
  gyro_z_offset = sum_z / samples; // Calculate offset
  Serial.print("Gyro Z Offset: ");
  Serial.println(gyro_z_offset, 6);

  Serial.println("Gyroscope in degrees/second");
  Serial.println("X\tY\tZ\tAngle");

  previous_time = micros(); // Use micros() for better precision
}

void loop() {
  float x, y, z;

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x, y, z);

    z = (z - gyro_z_offset) * 180.0 / PI; // Convert to degrees/sec and remove offset

    unsigned long current_time = micros();
    float delta_T = (current_time - previous_time) / 1000000.0; // Convert to seconds
    previous_time = current_time;

    angle += z * delta_T; // Integrate the angular velocity

    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.print(z);
    Serial.print('\t');
    Serial.println(angle);
  }
}