#include "bms.h"
#include <Arduino.h>
#include "Adafruit_INA3221.h"
#include <LiquidCrystal.h>

#DEFINE LED 10
#DEFINE BUZZER 13

// Creating an INA3221 object
Adafruit_INA3221 ina3221;

// Battery pack configuration
const float FULL_CAPACITY_MAH = 2450.0;  // Total capacity in mAh
const int NUM_CELLS = 8;
const float NOMINAL_CELL_VOLTAGE = 1.2;  // Nominal voltage per cell
const float FULL_VOLTAGE = 1.45 * NUM_CELLS;  // ~1.45V per cell when fully charged
const float EMPTY_VOLTAGE = 1.0 * NUM_CELLS;  // ~1.0V per cell when discharged --- TRY CHANGING TO 1.25V???

// Variables for coulomb counting
float remainingCapacityMah = FULL_CAPACITY_MAH;
unsigned long lastSampleTime = 0;

// Initialize the LCD library with the numbers of the interface pins
// LiquidCrystal(rs, enable, d4, d5, d6, d7)
LiquidCrystal lcd(12, 11, 9, 8, 7, 6);

// Define the struct instance declared in the header
BMSData bmsData;

void initBMS() {
    // Initialize the LCD
    lcd.begin(16, 2);
    lcd.print("INA3221 Monitor");

    // Initialize INA3221
    if (!ina3221.begin(0x40, &Wire)) { // can use other I2C addresses or buses
      Serial.println("Failed to find INA3221 chip");
      lcd.clear();
      lcd.print("INA3221 Error!");
      while (1)
        delay(10);
    }
    Serial.println("INA3221 Found!");
  
    // Show "Ready" on LCD
    lcd.setCursor(0, 1);
    lcd.print("Ready!");
    delay(1000);

    ina3221.setAveragingMode(INA3221_AVG_16_SAMPLES);

    // Set shunt resistances for all channels to 0.05 ohms
    for (uint8_t i = 0; i < 3; i++) {
      ina3221.setShuntResistance(i, 0.05);
    }

    // Set a power valid alert to tell us if ALL channels are between the two
    // limits:
    ina3221.setPowerValidLimits(3.0 /* lower limit */, 15.0 /* upper limit */);    
}

float estimateCapacityFromVoltage(float voltage) {
  // NiMH voltage-to-capacity curve (approximate)
  // Note: This is imprecise for NiMH due to the flat discharge curve
  if (voltage >= FULL_VOLTAGE) return FULL_CAPACITY_MAH;
  if (voltage <= EMPTY_VOLTAGE) return 0.0;
  
  // For NiMH this is very approximate due to flat discharge curve
  if (voltage >= 1.3 * NUM_CELLS) return FULL_CAPACITY_MAH * 0.9;  // 90%
  if (voltage >= 1.25 * NUM_CELLS) return FULL_CAPACITY_MAH * 0.8;  // 80%
  if (voltage >= 1.2 * NUM_CELLS) return FULL_CAPACITY_MAH * 0.5;   // 50%
  if (voltage >= 1.15 * NUM_CELLS) return FULL_CAPACITY_MAH * 0.3;  // 30%
  if (voltage >= 1.1 * NUM_CELLS) return FULL_CAPACITY_MAH * 0.15;  // 15%
  if (voltage >= 1.05 * NUM_CELLS) return FULL_CAPACITY_MAH * 0.05; // 5%
  
  return 0.0;
}

void updateBMS() {
    // Update the INA3221 readings
    unsigned long currentTime = millis();
    // Get channel 1 values (channel index 0)
    // Right motor values 
    bmsData.voltage_1 = ina3221.getBusVoltage(0);
    bmsData.current_1 = ina3221.getCurrentAmps(0) * 1000; // Convert to mA
    bmsData.power_1 = bmsData.voltage_1 * (bmsData.current_1 / 1000.0); // Power in Watts
    // Get channel 2 values (channel index 1)
    // Left motor values 
    bmsData.voltage_2 = ina3221.getBusVoltage(1);
    bmsData.current_2 = ina3221.getCurrentAmps(1) * 1000; // Convert to mA
    bmsData.power_2 = bmsData.voltage_2 * (bmsData.current_2 / 1000.0); // Power in Watts
    // Get channel 3 values (channel index 2)
    // Battery pack values 
    bmsData.voltage_3 = ina3221.getBusVoltage(2);
    bmsData.current_3 = ina3221.getCurrentAmps(2) * 1000; // Convert to mA
    bmsData.power_3 = bmsData.voltage_3 * (bmsData.current_3 / 1000.0); // Power in Watts

  // Calculate time difference in hours
  if (lastSampleTime > 0) {
    float timeDiff = (currentTime - lastSampleTime) / 3600000.0;  // Convert ms to hours
    
    // Update coulomb counting (negative current = discharge)
    remainingCapacityMah -= bmsData.current_3 * timeDiff;
    
    // Constrain to valid range
    remainingCapacityMah = constrain(remainingCapacityMah, 0, FULL_CAPACITY_MAH);
    
    // Periodic voltage-based calibration (when current is low)
    if (abs(bmsData.current_3) < 50) {  // Low current condition
      float voltageEstimatedCapacity = estimateCapacityFromVoltage(bmsData.voltage_3);
      // Apply weighted calibration (80% coulomb counting, 20% voltage estimate)
      remainingCapacityMah = 0.8 * remainingCapacityMah + 0.2 * voltageEstimatedCapacity;
    }
  }
  
  lastSampleTime = currentTime;
}

float getBatteryPercentage() {
    // Calculate battery percentage based on remaining capacity
    return (remainingCapacityMah / FULL_CAPACITY_MAH) * 100.0;
}

void alertLowBattery() {
    // Alert user of low battery using buzzer and LED
    if (getBatteryPercentage() < 25.0) {
        digitalWrite(LED, HIGH);  // Turn on LED
        digitalWrite(BUZZER, HIGH);  // Turn on Buzzer
    } else {
        digitalWrite(LED, LOW);  // Turn off LED
        digitalWrite(BUZZER, LOW);  // Turn off Buzzer
    }
}

void displayBatteryInfo() {
  updateBMS();  // Update BMS data before displaying
  // Use static variables to remember state across calls
  static unsigned long lastUpdate = 0;
  static int displayStep = 0;
  unsigned long now = millis();

  // Only proceed if 1.5 seconds have passed
  if (now - lastUpdate < 1500) {
    return; // Return immediately to avoid blocking
  }
  lastUpdate = now;
  
  // Clear LCD before showing the next screen
  lcd.clear();

  switch (displayStep) {
    case 0:
      // Display channel 1
      lcd.setCursor(0, 0);
      lcd.print("V:");
      lcd.print(bmsData.voltage_1, 1);
      lcd.print("V");
      lcd.setCursor(8, 0);
      lcd.print("I:");
      lcd.print(bmsData.current_1, 0);
      lcd.print("mA");
      lcd.setCursor(0, 1);
      lcd.print("P:");
      lcd.print(bmsData.power_1, 2);
      lcd.print("W");
      lcd.setCursor(9, 1);
      lcd.print("R-Motor");
      break;

    case 1:
      // Display channel 2
      lcd.setCursor(0, 0);
      lcd.print("V:");
      lcd.print(bmsData.voltage_2, 1);
      lcd.print("V");
      lcd.setCursor(8, 0);
      lcd.print("I:");
      lcd.print(bmsData.current_2, 0);
      lcd.print("mA");
      lcd.setCursor(0, 1);
      lcd.print("P:");
      lcd.print(bmsData.power_2, 2);
      lcd.print("W");
      lcd.setCursor(9, 1);
      lcd.print("L-Motor");
      break;

    case 2:
      // Display channel 3
      lcd.setCursor(0, 0);
      lcd.print("V:");
      lcd.print(bmsData.voltage_3, 1);
      lcd.print("V");
      lcd.setCursor(8, 0);
      lcd.print("I:");
      lcd.print(bmsData.current_3, 0);
      lcd.print("mA");
      lcd.setCursor(0, 1);
      lcd.print("P:");
      lcd.print(bmsData.power_3, 2);
      lcd.print("W");
      lcd.setCursor(9, 1);
      lcd.print("Battery");
      break;

    case 3:
      // Display remaining battery
      lcd.setCursor(0, 0);
      lcd.print("Remaining");
      lcd.setCursor(0, 1);
      lcd.print("Battery:");
      lcd.print(getBatteryPercentage(), 1);
      lcd.print("%");
      alertLowBattery();  // Call the alert function
      break;
  }

  // Move to the next step
  displayStep = (displayStep + 1) % 4;
}
