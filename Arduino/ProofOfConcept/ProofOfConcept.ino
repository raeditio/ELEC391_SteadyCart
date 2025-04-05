#define leftForward D2
#define leftReverse D3
#define rightForward D4
#define rightReverse D5

const int forward[] = {leftForward, rightForward};
const int reverse[] = {leftReverse, rightReverse}
const int motors[] = {forward, reverse};

int task = 0;
int pwm = 0;

void setup() {
  Serial.begin(115200);
  for (motor:motors) {
    pinMode(motor, OUTPUT);
  }
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
  RPM commands set the rpm of the motor by percentage. e.g. RPM:50 sets rpm at 50%
**/
  if (Serial.available()) {
    command = Serial.readStringUntil("\n");
    if (command.startsWith("TASK:" || "RPM:")) {
      onCommandReceived(command);
    }
  }
}

void onCommandReceived(string command) {
/**
  Parses the incoming command and performs actions based on keywork-- TASK or RPM
  Input: string command
**/
  char buffer[command.length() + 1];
  command.toCharArray(buffer, sizeof(buffer)); // Convert to C-string

  char *cmd = strtok(buffer, ":");  // Extract msg before colon
  int val = atoi(strtok(NULL, ":"));   // Extract msg after colon

  if (strcmp(cmd, "TASK") == 0) {
    if (val >= 0 && val <= 5) {
      task = val;
    }
  } else if (strcmp(cmd, "RPM") == 0) {
    pwm = val;
    switch (task) {
      case 0:
        pwm = 0;
        for (motor:motors) {
          analogWrite(motor, pwm);
        }
      case 1:
        for (motor:forward) {
          analogWrite(motor, pwm);
        }
      case 2:
        for (motor:reverse) {
          analogWrite(motor, pwm);
        }
      case 3:
      case 4:
      case 5:
    }
  }

  }
}