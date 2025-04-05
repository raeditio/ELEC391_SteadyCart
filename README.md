# ELEC391 SteadyCart
ELEC 391 Project Repository

This project intends to design a self-balancing dual wheel cart. An Arduino Nano 33 BLE is used to measure and control the movement of the cart.
The data from the IMU embedded inside the MCU is used to promptly receive real-time data of the cart's acceleration, which then is used as feedback for the motor controller.
The design of this device is set to counteract the motion of the cart, mostly due to gravity, and resultantly keep the cart upright.

## Microcontroller and Sensors Assignment
Four test codes
- task1.ino prints the gyroscope data
- task2.ino computes the current angle of the MCU using the accelerometer data of the IMU
- task3.ino computes the current angle of the MCU using the gyroscope data
- task4.ino computes a complementary filter angle using both data from the accelerometer and the gyroscope by applying a tuned filter coefficient K
    - The output angle is more dependent on the accelerometer with a lower K and more dependent on the gyroscope with a K closer to 1

The each python scripts display these angles by receiving the printed angle data from the flashed MCU through a serial connection and plots the data using matplotlib.

## PCB Test Plan
Lays out a physical test plan for the fabrication and the functionality of an astable multivibrator circuit that was designed to output an audible frequency.

## Arduino
Collection of source codes for the Arduino Nano 33 BLE in development that may be used or referenced within the scope of this project, including some updates for the scripts placed inside the Microcontroller and Sensors Assignment.

## Proof of Concept
The Proof of Concept program was written to demonstrate the MCU's ability to adjust its PWM inputs into the motor controller based on given commands and its detected angular position.

## Main
The Main program is set to balance the cart using the complementary angle-based DC motor control. The complementary angle is calculated as a combination of the angles calculated each using the accelerometer and the gyroscope. The gyro angle is initialized based on the accelerometer data. Through PID, the required force to balance the cart is computed. This force is converted into the required RPM of the wheels and its directions, then into the required PWM output of the Arduino Nano 33 BLE Sense Rev2 as an input to the DRV8833 motor controllers, using a polynomial mapping function. The angle of the cart, PID, required force, RPM and PWM are outputted through the serial monitor.
