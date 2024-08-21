#ifndef ENGINE_H_
#define ENGINE_H_
#include "def.h"

#include <PID_v1.h>

void init_engine();

void stopMotor();

void engine_update(float target_speed);

void set_engine(int motorSpeedCommand);


#endif /* ENGINE_H_ */