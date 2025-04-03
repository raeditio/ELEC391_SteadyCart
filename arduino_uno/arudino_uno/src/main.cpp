#include <Arduino.h>

// Baud rate for communication with ESP32-CAM
// MUST match the SERIAL2_BAUD_RATE on the ESP32-CAM
const long COMMUNICATION_BAUD_RATE = 115200;

// Counter variable to send changing data
long messageCounter = 0;

void setup() {
  // Initialize Serial (Pins 0 RX, 1 TX) for communication with ESP32-CAM
  Serial.begin(COMMUNICATION_BAUD_RATE);
  // You could print to the Arduino monitor *before* connecting to ESP32 if needed for debugging setup,
  // but once connected, this Serial port is dedicated to ESP32 communication.
  // For example:
  // Serial.println("Arduino Sender Starting..."); // This would go to ESP32 now
}

void loop() {
  // Prepare the message to send
  String messageToSend = "Message from Uno #";
  messageToSend += messageCounter;

  // Send the string to the ESP32-CAM, followed by a newline character
  Serial.println(messageToSend);

  // Increment the counter for the next message
  messageCounter++;

  // Wait for a couple of seconds before sending the next message
  delay(2000); // Send data every 2 seconds
}