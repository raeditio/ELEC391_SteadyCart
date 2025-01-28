/*
  Arduino BMI270 - Simple Gyroscope

  This example reads the gyroscope values from the BMI270
  sensor and continuously prints them to the Serial Monitor
  or Serial Plotter.

  The circuit:
  - Arduino Nano 33 BLE Sense Rev2

  created 10 Jul 2019
  by Riccardo Rizzo

  This example code is in the public domain.
*/

#include "Arduino_BMI270_BMM150.h"
#include <math.h>

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  Serial.println();
  Serial.println("Gyroscope in degrees/second");
  Serial.println("X\tY\tZ\tAngle");
}

void loop() {

  float x, y, z;
  float previous_z = 0.0;
  float angle = 0.0;
  unsigned long previous_time = 0;
  unsigned long current_time = millis();
  float delta_T = (current_time -previous_time)/1000.0;

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x, y, z);

    z = z * 180.0 / PI;
    angle += z * delta_T;
    previous_time = current_time;

    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.println(z);
    Serial.print('\t');
    Serial.println(angle);
  }
}
