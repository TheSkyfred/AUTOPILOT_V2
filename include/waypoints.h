#ifndef WAYPOINTS_H_
#define WAYPOINTS_H_

#include "def.h"


void update_waypoints();
double calculateTotalDistance();

//double calculateDistance(double lat1, double lon1, double lat2, double lon2);
double degreesToRadians(double degrees);

EEPROM_Waypoint getCurrentWaypoint();
void next_waypoint();
void goToLanding();


void saveWaypointToEEPROM(const MavLinkWayPoint& MLWaypoints);

EEPROM_Waypoint loadWaypointFromEEPROM(uint16_t seq);



#endif /* WAYPOINTS_H_ */