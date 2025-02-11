#include <Arduino.h>

#include "accelAngle.h"

#define leftForward D2
#define leftReverse D3
#define rightForward D4
#define rightReverse D5

const int motors[] = {leftForward, rightForward, leftReverse, rightReverse};
const int forward[] = {leftForward, rightForward};
const int reverse[] = {leftReverse, rightReverse};

int task = 0;
int pwm = 0;

void onCommandReceived(String command) {
  /**
    Parses the incoming command and performs actions based on keywork-- TASK or RPM
    Input: string command
  **/
    char buffer[command.length() + 1];
    command.toCharArray(buffer, sizeof(buffer)); // Convert to C-string
  
    char *cmd = strtok(buffer, ":");  // Extract msg before colon
    char *valStr = strtok(NULL, ":");

    if (valStr == nullptr) {
      Serial.println("Invalid command format.");
      return;
    }

    int val = atoi(valStr);
  
    if (strcmp(cmd, "TASK") == 0) {
      // Set the task number and print action
      task = val;
      switch (task) {
      case 0:
        pwm = 0;
        for (int motor : motors) {
        analogWrite(motor, pwm);
        }
        Serial.println("Task0: Idle");
        break;
      case 1:
        pwm = 0;
        for (int motor : motors) {
        analogWrite(motor, pwm);
        }
        Serial.println("Task1: Forward Drive");
        break;
      case 2:
        pwm = 0;
        for (int motor : motors) {
        analogWrite(motor, pwm);
        }
        Serial.println("Task2: Reverse Drive");
        break;
      case 3:
        pwm = 0;
        for (int motor : motors) {
        analogWrite(motor, pwm);
        }
        Serial.println("Task3: Opposite Directions");
        break;
      case 4:
        pwm = 0;
        for (int motor : motors) {
        analogWrite(motor, pwm);
        }
        Serial.println("Task4: Opposite Direction Reverse");
        break;
      case 5:
        pwm = 0;
        for (int motor : motors) {
        analogWrite(motor, pwm);
        }
        Serial.println("Task4: Angle Counter Drive");
        break;
      default:
        pwm = 0;
        for (int motor : motors) {
        analogWrite(motor, pwm);
        }
        Serial.println("Invalid task number. Only 0-4 are allowed");
      }

    } else if (strcmp(cmd, "RPM") == 0) {
      // Set the pwm value based on the percentage
      switch (val) {
        case 0:
          pwm = 0;
          Serial.println("RPM set to 0% of max speed");
          break;
        case 25:
          pwm = val * 2.55;
          Serial.println("RPM set to 25% of max speed");
          break;
          case 50:
          pwm = val * 2.55;
          Serial.println("RPM set to 50% of max speed");
          break;
        case 75:
          pwm = val * 2.55;
          Serial.println("RPM set to 75% of max speed");
          break;
        case 100:
          pwm = val * 2.55;
          Serial.println("RPM set to 100% of max speed");
          break;
        default:
          Serial.println("Invalid RPM value. Only 25, 50, 75, 100 are allowed");
      }

      switch (task) {
        case 0:
          Serial.println("RPM command not allowed for Task0");
          break;
        case 1:
          for (int motor:forward) {
            analogWrite(motor, pwm);
          }
          break;
        case 2:
          for (int motor : reverse) {
            analogWrite(motor, pwm);
          }
          break;
        case 3:
          analogWrite(forward[0], pwm);
          analogWrite(reverse[1], pwm);
          break;
        case 4:
          analogWrite(reverse[0], pwm);
          analogWrite(forward[1], pwm);
        case 5:
          Serial.println("RPM command not allowed for Task4");
          break;
      }
    }
  }

void setup() {
  Serial.begin(115200);
  for (int motor : motors) {
    pinMode(motor, OUTPUT);
  }
  Serial.println("Motor control initialized");
  
  // Initialize IMU
  initIMU();
  Serial.println("IMU initialized");
  Serial.println("Waiting for commands (TASK:0-5, RPM:xx)...");
}

void loop() {
/**
  Executes command based actions
  A command either begins with "TASK:" or "RPM:"
  A TASK command sets the number of the task from 0-4
  Each task drives the motor in their corresponding modes
  0: idle
  1: forward drive
  2: reverse drive
  3: Opposite directions
  4: Angle counter drive
  RPM commands set the pwm of the motor by percentage. e.g. RPM:50 sets pwm at 50%
**/
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.startsWith("TASK:") || command.startsWith("RPM:")) {
      onCommandReceived(command);
    }
  }
  

  if (task == 5) {
    float accelAngle = getAccelAngle();  // Get accelerometer angle

    // Scale -90° to 90° into 0 to 255 PWM
    int speed = map(abs(accelAngle), 0, 90, 0, 255);
    
    Serial.print("Angle: ");
    Serial.print(accelAngle, 2);
    Serial.print("°, Adjusted PWM: ");
    Serial.print(speed);
    
    // Apply proportional speed to motors
    // If angle is positive, move forward; if negative, move in reverse
    if (speed !=0) {
      if (accelAngle > 0) {
        // Move forward
        Serial.println(", Forward");
        analogWrite(leftForward, speed);
        analogWrite(rightForward, speed);
        analogWrite(leftReverse, 0);
        analogWrite(rightReverse, 0);
      } else {
        // Move in reverse
        Serial.println(", Reverse");
        analogWrite(leftReverse, speed);
        analogWrite(rightReverse, speed);
        analogWrite(leftForward, 0);
        analogWrite(rightForward, 0);
      }
    } else {
      // Stop motors if angle is zero
      Serial.println(", Idle");
      analogWrite(leftForward, 0);
      analogWrite(rightForward, 0);
      analogWrite(leftReverse, 0);
      analogWrite(rightReverse, 0);
  }
}

  delay(100);
}