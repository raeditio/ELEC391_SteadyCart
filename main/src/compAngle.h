#ifndef ACCEL_ANGLE_H
#define ACCEL_ANGLE_H

void initIMU();         // Initializes the IMU
float getCompAngle();  // Returns the complementary tilt angle
int computePID(float currentAngle);       // Computes the PID output
#endif