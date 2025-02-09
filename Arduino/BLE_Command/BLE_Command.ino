#include <ArduinoBLE.h>

// BLE UART Service
BLEService commandService("6E400001-B5A3-F393-E0A9-E50E24DCCA9E"); 
BLECharacteristic rxCharacteristic("6E400002-B5A3-F393-E0A9-E50E24DCCA9E", BLEWrite, 512);

const int pwmPins[] = {D2, D3}; // Define usable PWM pins

void setup() {
  Serial.begin(115200);
  while (!Serial);

  if (!BLE.begin()) {
    Serial.println("Starting BLE failed!");
    while (1);
  }

  BLE.setLocalName("Self-Balancing Cart");
  BLE.setAdvertisedService(commandService);
  commandService.addCharacteristic(rxCharacteristic);
  BLE.addService(commandService);
  
  rxCharacteristic.setEventHandler(BLEWritten, onCommandReceived);
  BLE.advertise();
  
  for (int pin : pwmPins) {
    pinMode(pin, OUTPUT);
  }

  Serial.println("BLE PWM Controller Ready!");
}

void loop() {
  BLEDevice central = BLE.central();
  
  if (central) {
    Serial.print("Connected to: ");
    Serial.println(central.address());

    while (central.connected()) {
      // Loop while connected
      BLE.poll();  // Handle BLE events

      int pwmValue = 128; // Initial PWM 50%

      analogWrite(pwmPins[1], pwmValue);  // Apply PWM output
      Serial.print(" | PWM: ");
      Serial.println(pwmValue);

      delay(50);  // Small delay for stability
    }
    
    Serial.println("Disconnected.");
  }
}

// BLE Command Parser
void onCommandReceived(BLEDevice central, BLECharacteristic characteristic) {
  String command = characteristic.value();
  Serial.println("Received: " + command);
  
  if (command.startsWith("PWM:")) {
    processPWMCommand(command);
  } else {
    Serial.println("Unknown command.");
  }
}

// PWM Command Executer
void processPWMCommand(String command) {
  int firstColon = command.indexOf(':');
  int secondColon = command.indexOf(':', firstColon + 1);

  if (firstColon == -1 || secondColon == -1) {
    Serial.println("Invalid PWM command format.");
    return;
  }

  int pin = command.substring(firstColon + 1, secondColon).toInt();
  int value = command.substring(secondColon + 1).toInt();
  
  if (value < 0) value = 0;
  if (value > 255) value = 255; // PWM range 0-255

  bool validPin = false;
  for (int p : pwmPins) {
    if (p == pin) {
      validPin = true;
      break;
    }
  }

  if (!validPin) {
    Serial.println("Invalid PWM pin.");
    return;
  }

  analogWrite(pin, value);
  Serial.print("Set PWM ");
  Serial.print(pin);
  Serial.print(" to ");
  Serial.println(value);
}
