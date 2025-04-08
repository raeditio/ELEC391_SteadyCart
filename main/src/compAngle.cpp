#include "compAngle.h"
#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// IMU Variables
double accelAngle = 0;          // Angle calculated using only the accelerometer
double gyroAngle;               // Angle calculated using only the gyroscope
double compAngle;               // Combined angle using complementary filter

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
    Serial.println("Initializing MPU6050...");

    Wire.begin();  // Initialize I2C communication
    mpu.initialize();  // Initialize MPU6050

    // Check MPU6050 connection
    if (!mpu.testConnection()) {
        Serial.println("MPU6050 connection failed!");
        while (1);  // Halt execution if the IMU is not connected
    }
    Serial.println("MPU6050 connected.");

    // Initialize DMP
    uint8_t devStatus = mpu.dmpInitialize();

    // (Optional) Adjust offsets here if needed:
    // mpu.setXAccelOffset(...); etc.

    if (devStatus == 0) {
        mpu.setDMPEnabled(true);
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
        Serial.println("DMP ready.");
    } else {
        Serial.print("DMP Initialization failed (code ");
        Serial.print(devStatus);
        Serial.println(").");
        while (1);  // Halt execution if DMP initialization fails
    }

    // Initialize time for complementary filter
    prevTime = millis();
}

float getAccelAngle() {
    // This function is not used with the DMP but kept for reference
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);

    // Compute tilt angle directly using the accelerometer
    accelAngle = atan2(ay, az) * 180.0 / PI;
    return accelAngle;
}

float getGyroAngle() {
    // This function is not used with the DMP but kept for reference
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);

    // Compute angular velocity (gyro rate) in degrees per second
    float dt = (millis() - prevTime) / 1000.0;  // Time difference in seconds
    prevTime = millis();
    gyroAngle += gx * dt / 131.0;  // Convert raw gyro data to degrees/s
    return gyroAngle;
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