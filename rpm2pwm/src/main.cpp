#include <Arduino.h>

int pwm = 50;  // Start at 50
int pwmStep = 20;
int maxPWM = 255;
unsigned long lastUpdate = 0;
const int stepDuration = 5000;  // 5 seconds per step

void setup() {
    pinMode(D2, OUTPUT); // PWM pin for motor driver
    Serial.begin(115200);
}

void loop() {
    unsigned long currentTime = millis();

    // Change PWM every stepDuration milliseconds
    if (currentTime - lastUpdate > stepDuration) {
        analogWrite(D2, pwm);  // Set new PWM value
        Serial.print("PWM: ");
        Serial.println(pwm);
        
        pwm += pwmStep;  // Increase PWM
        if (pwm > maxPWM) pwm = maxPWM;  // Prevent exceeding 255

        lastUpdate = currentTime;
    }
}