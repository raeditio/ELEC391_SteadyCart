#include <Arduino.h>
#include <ArduinoBLE.h>
#include "compAngle.h"
#include "bms.h"

#define BUFFER_SIZE 20

// Define a custom BLE service and characteristic
BLEService customService("00000000-5EC4-4083-81CD-A10B8D5CF6EC");
BLECharacteristic customCharacteristic(
    "00000001-5EC4-4083-81CD-A10B8D5CF6EC", BLERead | BLEWrite | BLENotify, BUFFER_SIZE, false);

#define leftForward D2
#define leftReverse D3
#define rightForward D4
#define rightReverse D5

const int motors[] = {leftForward, rightForward, leftReverse, rightReverse};
const int forward[] = {leftForward, rightForward};
const int reverse[] = {leftReverse, rightReverse};

int task = 0;
int pwm = 0;
// float targetAngle = 0;  // Desired angle (e.g., keep at level 0°)

void initBalanceSequence() {
    // Initialize motors
    for (int motor : motors) {
        pinMode(motor, OUTPUT);
        digitalWrite(motor, LOW);  // Ensure all motors are off initially
    }
    Serial.println("Motors initialized");

    // initialize IMU
    initIMU();
    Serial.println("IMU initialized");
}

void initBLE() {
    if (!BLE.begin()) {
        Serial.println("Starting BLE failed!");
        while (1);
    }

    // Set the device name and local name
    BLE.setLocalName("BLE-SteadyCart");
    BLE.setDeviceName("BLE-SteadyCart");

    // Add the characteristic to the service
    customService.addCharacteristic(customCharacteristic);

    // Add the service
    BLE.addService(customService);

    // Set an initial value for the characteristic
    customCharacteristic.writeValue("Waiting for data");

    // Start advertising the service
    BLE.advertise();

    Serial.println("Bluetooth® device active, waiting for connections...");
}

void balance() {
    float currentAngle = getCompAngle();  // Get tilt angle

    // Compute PID-controlled motor pwmValue
    int pwmValue = computePID(currentAngle);

    Serial.print(", Adjusted PWM: ");
    if (currentAngle < 0) Serial.print("-");
    Serial.println(pwmValue);

    // If angle is positive, move forward; if negative, move in reverse
    if (currentAngle < 0) {
        analogWrite(leftForward, pwmValue);
        analogWrite(rightForward, pwmValue);
        analogWrite(leftReverse, 0);
        analogWrite(rightReverse, 0);
    } else if (currentAngle > 0) {
        analogWrite(leftReverse, pwmValue);
        analogWrite(rightReverse, pwmValue);
        analogWrite(leftForward, 0);
        analogWrite(rightForward, 0);
    } else {
        // If angle is 0, stop the motors
        analogWrite(leftForward, 0);
        analogWrite(rightForward, 0);
        analogWrite(leftReverse, 0);
        analogWrite(rightReverse, 0);
    }
}

void startBMS() {
    pinMode(LED_BUILTIN, OUTPUT);

    initBMS();  // Initialize BMS
    Serial.println("BMS initialized");
}

void ReceiveBLECommand() {
    // Wait for a BLE central to connect
    BLEDevice central = BLE.central();
    if (central) {
        Serial.print("Connected to central: ");
        Serial.println(central.address());
        digitalWrite(LED_BUILTIN, HIGH); // Turn on LED to indicate connection

        // Keep running while connected
        while (central.connected()) {
            // Check if the characteristic was written
            if (customCharacteristic.written()) {
            // Get the length of the received data
            int length = customCharacteristic.valueLength();

            // Read the received data
            const unsigned char* receivedData = customCharacteristic.value();

            // Create a properly terminated string
            char receivedString[length + 1]; // +1 for null terminator
            memcpy(receivedString, receivedData, length);
            receivedString[length] = '\0'; // Null-terminate the string

            // Print the received data to the Serial Monitor
            Serial.print("Received data: ");
            Serial.println(receivedString);


            // Optionally, respond by updating the characteristic's value
            customCharacteristic.writeValue("Data received");
            }
        }
    digitalWrite(LED_BUILTIN, LOW); // Turn off LED when disconnected
    Serial.println("Disconnected from central.");
    }
}

void setup() {
    Serial.begin(115200);
    while(!Serial);

    // initBLE();  // Initialize BLE
    initBalanceSequence();  // Initialize motors and IMU
    // startBMS();  // Initialize BMS
}

void loop() {
    balance();
    //displayBatteryInfo();
}