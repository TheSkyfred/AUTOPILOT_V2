#ifndef GYROSCOPE_H_
#define GYROSCOPE_H_

#include "def.h"


#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>



void init_gyroscope();
void update_gyroscope();

#endif /* GYROSCOPE_H_ */

