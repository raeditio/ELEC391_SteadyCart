#include <Arduino.h>
#include "compAngle.h"
#include "Adafruit_INA3221.h"
#include <LiquidCrystal.h>

#define leftForward D2
#define leftReverse D3
#define rightForward D4
#define rightReverse D5

const int motors[] = {leftForward, rightForward, leftReverse, rightReverse};
const int forward[] = {leftForward, rightForward};
const int reverse[] = {leftReverse, rightReverse};

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

int task = 0;
int pwm = 0;
// float targetAngle = 0;  // Desired angle (e.g., keep at level 0°)

void setup() {
    Serial.begin(115200);
    for (int motor : motors) {
        pinMode(motor, OUTPUT);
    }
    Serial.println("Motor control initialized");

    //defining pins for battery monitor 
    //pin 10 - LED
    //pin 13 - buzzer (note inverted logic: V+ pin of buzzer will be wired to 3.3V output pin of Arduino and pin 13 will be wired to buzzer V-)
    pinMode(10,OUTPUT);
    pinMode(13,OUTPUT);
    digitalWrite(10,LOW);
    digitalWrite(13,HIGH);

    // Initialize LCD
    lcd.begin(16, 2);
    lcd.print("INA3221 Monitor");

    // Initialize IMU
    initIMU();

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

void loop() {
    float currentAngle = getCompAngle();  // Get tilt angle
    unsigned long currentTime = millis();
    // Get channel 1 values (channel index 0)
    // Right motor values 
    float voltage_1 = ina3221.getBusVoltage(0);
    float current_1 = ina3221.getCurrentAmps(0) * 1000; // Convert to mA
    float power_1 = voltage_1 * current_1 / 1000; // Power in Watts
    // Get channel 2 values (channel index 1)
    // Left motor values 
    float voltage_2 = ina3221.getBusVoltage(1);
    float current_2 = ina3221.getCurrentAmps(1) * 1000; // Convert to mA
    float power_2 = voltage_2 * current_2 / 1000; // Power in Watts
    // Get channel 3 values (channel index 2)
    // Battery pack values 
    float voltage_3 = ina3221.getBusVoltage(2);
    float current_3 = ina3221.getCurrentAmps(2) * 1000; // Convert to mA
    float power_3 = voltage_3 * current_3 / 1000; // Power in Watts

    // Compute PID-controlled motor speed
    int speed = computePID(currentAngle);
    // int speed = 0;

    // Serial.print("Target: ");
    // Serial.print(targetAngle);
    // Serial.print("°, Current: ");
    // Serial.print(currentAngle, 2);
    Serial.print(", Adjusted PWM: ");
    if (currentAngle > 0) Serial.print("-");
    Serial.println(speed);

    // If angle is positive, move forward; if negative, move in reverse
    if (currentAngle < 0) {
        analogWrite(leftForward, speed);
        analogWrite(rightForward, speed);
        analogWrite(leftReverse, 0);
        analogWrite(rightReverse, 0);
    } else if (currentAngle > 0) {
        analogWrite(leftReverse, speed);
        analogWrite(rightReverse, speed);
        analogWrite(leftForward, 0);
        analogWrite(rightForward, 0);
    // } else {
    //     // Stop motors if the angle is near 0
    //     analogWrite(leftForward, 0);
    //     analogWrite(rightForward, 0);
    //     analogWrite(leftReverse, 0);
    //     analogWrite(rightReverse, 0);
    }

    //delay(100);  // Short delay for stability

      // Calculate time difference in hours
  if (lastSampleTime > 0) {
    float timeDiff = (currentTime - lastSampleTime) / 3600000.0;  // Convert ms to hours
    
    // Update coulomb counting (negative current = discharge)
    remainingCapacityMah -= current_3 * timeDiff;
    
    // Constrain to valid range
    remainingCapacityMah = constrain(remainingCapacityMah, 0, FULL_CAPACITY_MAH);
    
    // Periodic voltage-based calibration (when current is low)
    if (abs(current_3) < 50) {  // Low current condition
      float voltageEstimatedCapacity = estimateCapacityFromVoltage(voltage_3);
      // Apply weighted calibration (80% coulomb counting, 20% voltage estimate)
      remainingCapacityMah = 0.8 * remainingCapacityMah + 0.2 * voltageEstimatedCapacity;
    }
  }
  
  lastSampleTime = currentTime;
  
  // Calculate battery percentage
  float batteryPercentage = (remainingCapacityMah / FULL_CAPACITY_MAH) * 100.0;

  //alert user of low battery using buzzer and LED
  if(batteryPercentage < 25.0){

    digitalWrite(10,HIGH);
    digitalWrite(13,LOW);

  }

  // Display on serial monitor (for debugging)
  //Serial.print("Channel 1: Voltage = ");
  //Serial.print(voltage_1, 2);
  //Serial.print(" V, Current = ");
  //Serial.print(current_1, 2);
  //Serial.print(" mA, Power = ");
  //Serial.print(power_1, 2);
  //Serial.println(" W");
  //Serial.print('\n');
  //Serial.print("Channel 2: Voltage = ");
  //Serial.print(voltage_2, 2);
  //Serial.print(" V, Current = ");
  //Serial.print(current_2, 2);
  //Serial.print(" mA, Power = ");
  //Serial.print(power_2, 2);
  //Serial.println(" W");
  //Serial.print('\n');
  //Serial.print("Channel 3: Voltage = ");
  //Serial.print(voltage_3, 2);
  //Serial.print(" V, Current = ");
  //Serial.print(current_3, 2);
  //Serial.print(" mA, Power = ");
  //Serial.print(power_3, 2);
  //Serial.println(" W");

  // Display channel 1 values on LCD
  lcd.clear();
  
  // First row: Voltage and Current
  lcd.setCursor(0, 0);
  lcd.print("V:");
  lcd.print(voltage_1, 1);
  lcd.print("V");
  
  lcd.setCursor(8, 0);
  lcd.print("I:");
  lcd.print(current_1, 0);
  lcd.print("mA");
  
  // Second row: Power and Channel indicator
  lcd.setCursor(0, 1);
  lcd.print("P:");
  lcd.print(power_1, 2);
  lcd.print("W");
  
  lcd.setCursor(9, 1);
  lcd.print("R-Motor");

  // Delay for 1 second before the next reading
  delay(1500);

  // Display channel 2 values on LCD
  lcd.clear();
  
  // First row: Voltage and Current
  lcd.setCursor(0, 0);
  lcd.print("V:");
  lcd.print(voltage_2, 1);
  lcd.print("V");
  
  lcd.setCursor(8, 0);
  lcd.print("I:");
  lcd.print(current_2, 0);
  lcd.print("mA");
  
  // Second row: Power and Channel indicator
  lcd.setCursor(0, 1);
  lcd.print("P:");
  lcd.print(power_2, 2);
  lcd.print("W");
  
  lcd.setCursor(9, 1);
  lcd.print("L-Motor");

  // Delay for 1 second before the next reading
  delay(1500);

  // Display channel 3 values on LCD
  lcd.clear();
  
  // First row: Voltage and Current
  lcd.setCursor(0, 0);
  lcd.print("V:");
  lcd.print(voltage_3, 1);
  lcd.print("V");
  
  lcd.setCursor(8, 0);
  lcd.print("I:");
  lcd.print(current_3, 0);
  lcd.print("mA");
  
  // Second row: Power and Channel indicator
  lcd.setCursor(0, 1);
  lcd.print("P:");
  lcd.print(power_3, 2);
  lcd.print("W");
  
  lcd.setCursor(9, 1);
  lcd.print("Battery");

  // Delay for 1 second before the next reading
  delay(1500);

  lcd.clear();
  // First row: Voltage and Current
  lcd.setCursor(0, 0);
  lcd.print("Remaining");
  lcd.setCursor(0, 1);
  lcd.print("Battery:");
  lcd.print(batteryPercentage, 1);
  lcd.print("%");
  
  delay(1500);
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
