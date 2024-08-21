#ifndef BAROMETER_H_
#define BAROMETER_H_

#include "def.h"

#include <Adafruit_BMP280.h> 
#include <Adafruit_Sensor.h>
#include <Wire.h>




void init_barometer();
void update_barometer();

void check_altitude();

#endif /* BAROMETER_H_ */