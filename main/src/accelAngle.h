#ifndef ACCEL_ANGLE_H
#define ACCEL_ANGLE_H

void initIMU();         // Initializes the IMU
float getAccelAngle();  // Returns the tilt angle using only the accelerometer
int computePID(float currentAngle);       // Computes the PID output
#endif