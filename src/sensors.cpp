
#include "sensors.h"

#include "GPS.h"
#include "barometer.h"
#include "gyroscope.h"
#include "myDriver.h"
#include "sonar.h"
#include "engine.h"


void init_sensors(){
init_GPS();
delay(200);
init_barometer();
delay(200);
init_gyroscope();
delay(200);
init_servo();
delay(200);
init_engine();
}


void update_sensors(){

update_GPS();
update_barometer();
update_gyroscope();
get_sonar();

}
