#include <Arduino.h>

// Define pins for Serial2 communication with Arduino Uno
#define RXD2_PIN 16  // ESP32-CAM Pin GPIO 16 (U2_RXD) connects to Arduino Uno Pin 1 (TX)
#define TXD2_PIN 13  // ESP32-CAM Pin GPIO 13 connects to Arduino Uno Pin 0 (RX)


// Baud rate for communication between ESP32-CAM and Arduino Uno
const long SERIAL2_BAUD_RATE = 115200;
// Baud rate for USB Serial Monitor debugging output
const long MONITOR_BAUD_RATE = 115200;


void setup() {
  // Initialize Serial for the USB Serial Monitor
  Serial.begin(MONITOR_BAUD_RATE);
  // while (!Serial); // Optional: wait for monitor to connect, can sometimes cause issues
  delay(500); // Short delay to allow monitor to connect
  Serial.println("\nESP32-CAM Receiver Ready");
  Serial.printf("Monitoring on Serial (USB)\n");

  // Initialize Serial2 for Arduino communication
  Serial2.begin(SERIAL2_BAUD_RATE, SERIAL_8N1, RXD2_PIN, TXD2_PIN);
  Serial.printf("Listening to Arduino on Serial2 (RX:%d, TX:%d) at %ld baud\n", RXD2_PIN, TXD2_PIN, SERIAL2_BAUD_RATE);
}

void loop() {
  // Check if data is available from the Arduino on Serial2
  if (Serial2.available()) {
    // Read the incoming string until a newline character is received
    String receivedString = Serial2.readStringUntil('\n');

    // Trim leading/trailing whitespace (optional but good practice)
    // receivedString.trim();

    // Print the received string to the main Serial Monitor (via USB)
    Serial.print("Received from Arduino: ");
    Serial.println(receivedString);
  }

  // You can add other ESP32-CAM tasks here
  // delay(10); // Small delay if needed, often not necessary
}
