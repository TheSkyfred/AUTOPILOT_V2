#ifndef SERVOS_H_
#define SERVOS_H_

#include "def.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_PWMServoDriver.h>




//Functions :
void init_servo();
void set_servos(int roll_correction, int pitch_correction, int yaw_correction);

#endif /* SERVOS_H_ */