#include "compAngle.h"
#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// IMU Variables
double accelAngle = 0;          // Angle calculated using only the accelerometer
double gyroAngle;           // Angle calculated using only the gyroscope
double compAngle;           // Combined angle using complementary filter

float Kc = 0.4;  // Weight for accelerometer data (adjust as needed)

unsigned long prevTime = 0;

MPU6050 mpu;
bool dmpReady = false;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll

void initIMU() {
    Serial.begin(115200);
    delay(100);  // Short delay to allow serial connection to initialize
    Serial.println("Initializing IMU...");

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
    accelAngle = atan2(ay, az) * 180.0 / PI;
    gyroAngle = accelAngle;  // Set initial gyroAngle as accelAngle
    compAngle = accelAngle;  // Set initial compAngle as accelAngle

    prevTime = millis();  // Initialize time for PID

    Wire.begin(21, 22);  // ESP32 default I2C pins: SDA=21, SCL=22
    mpu.initialize();
    // Optionally check mpu.testConnection() here

    uint8_t devStatus = mpu.dmpInitialize();
    // (Optional) Adjust offsets here if needed:
    // mpu.setXAccelOffset(...); etc.

    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.println("DMP Initialization failed.");
    }
}

float getAccelAngle() {
    if (IMU.accelerationAvailable()) {
        float ax, ay, az;
        IMU.readAcceleration(ax, ay, az);

        // Compute tilt angle directly using the accelerometer
        accelAngle = atan2(ay, az) * 180.0 / PI;
    }
    return accelAngle;
}

float getGyroAngle() {
    // float prevAngle = gyroAngle;
    if (IMU.gyroscopeAvailable()) {
        // Compute tilt angle using the gyroscope
        float gx, gy, gz;
        IMU.readGyroscope(gx, gy, gz);
        float dt = (millis() - prevTime) / 1000.0;  // Time difference in seconds
        prevTime = millis();
        gyroAngle -= gx * dt;
    }
    return (gyroAngle);
}

float getCompAngle() {
    if (!dmpReady) return 0.0;
    // Check FIFO count
    fifoCount = mpu.getFIFOCount();
    while (fifoCount < packetSize) {
        fifoCount = mpu.getFIFOCount();
    }
    // Read data from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;

    // Parse out YPR
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // Convert pitch to degrees (ypr[1]) from radians
    float pitchDegrees = ypr[1] * 180.0 / M_PI;
    return pitchDegrees;
}