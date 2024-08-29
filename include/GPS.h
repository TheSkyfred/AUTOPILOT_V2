#ifndef GPS_H_
#define GPS_H_

#include "def.h"


#include <TinyGPSPlus.h> //GPS Library



//GPSPoint CurrentPosition;
//GPSPoint homePoint;



//extern TinyGPSPlus gps;         // The TinyGPSPlus object



void init_GPS();
void update_GPS();

bool GPS_signal();

void GPS_set_home();
void updateTraveledDistance();


#endif /* GPS_H_ */