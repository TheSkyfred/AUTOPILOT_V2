#ifndef BATTERY_H_
#define BATTERY_H_

#include "def.h"

#include <Wire.h>

#include "INA226.h"


void init_battery();

void   battery_initial_check();
void   battery_check();



float readVoltage();

float readCurrent();

float readPower();
float readBatteryEnergy();

float calculateBatteryPercentage();


#endif /* BATTERY_H_ */