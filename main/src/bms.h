#pragma once

struct BMSData {
  float voltage_1;
  float current_1;
  float power_1;
  float voltage_2;
  float current_2;
  float power_2;
  float voltage_3;
  float current_3;
  float power_3;
};

extern BMSData bmsData;  // Make this available project-wide

void initBMS();
void displayBatteryInfo();