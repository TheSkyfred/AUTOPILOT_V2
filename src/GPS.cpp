

#include "GPS.h"

#include "def.h"
#include "config.h"

#include "mySerial.h"
#include "waypoints.h"
#include "navigation.h"

#include "modes.h"

#include "buzzer.h"

TinyGPSPlus gps;


double current_latitude = 0.0;
double current_longitude = 0.0;
double current_GPS_altitude = 0.0;
double current_GPS_speed = 0.0;

int heading_error = 0;

double last_latitude = 0.0;
double last_longitude = 0.0;
double traveledDistance = 0.0;

unsigned long waypoint_distance = 0;

int satellites_quantity = 0;
double hdop = 0;

void init_GPS()
{
    Serial2.begin(GPS_BAUDRATE);
    // PrintToSerial("GPS SERIAL READY\n");
};

void update_GPS()
{
    if (Serial2.available() > 0)
    {
        if (gps.encode(Serial2.read()))
        {

if(gps.satellites.isValid()){
satellites_quantity = gps.satellites.value(); 
}
            if (gps.location.isValid())
            {
                current_latitude = gps.location.lat();
                current_longitude = gps.location.lng();

                target_heading = TinyGPSPlus::courseTo(current_latitude, current_longitude, target_latitude, target_longitude);

                heading_error = abs(current_heading - target_heading);
                if (heading_error > 180)
                    heading_error -= 360;
                if (heading_error < -180)
                    heading_error += 360;



    heading_error = normalizeHeading(target_heading - current_heading);



                waypoint_distance = (unsigned long)TinyGPSPlus::distanceBetween(current_latitude, current_longitude, target_latitude, target_longitude);
            }

            else
                Serial.println(F("GPS LOCATION INVALID"));

            if (gps.altitude.isValid())
            {
                current_GPS_altitude = gps.altitude.meters();
            }
            else
                Serial.println(F("GPS ALTITUDE INVALID"));

            if (gps.speed.isValid())
            {
                current_GPS_speed = gps.speed.kmph();
            }
            else
            {
                Serial.println(F("GPS SPEED INVALID"));
            }
        }
    }

    if (millis() > 5000 && gps.charsProcessed() < 10)
        Serial.println(F("No GPS data received: check wiring"));
}

// Function to check GPS QUALITY before Take Off
bool GPS_signal()
{
    // Boucle qui attend que le HDOP soit inférieur à 5.0
    while (true)
    {
        while (Serial2.available() > 0) // Lire les données du GPS
        {
            gps.encode(Serial2.read()); // Décoder les données GPS
        }

        if (gps.hdop.isValid()) // Vérifier si le HDOP est valide
        {
            hdop = gps.hdop.hdop(); // Obtenir le HDOP en tant que nombre flottant
        }

        if (hdop < 3.00 && gps.location.isValid() && gps.altitude.isValid()) // CHECK HDOP QUALITY
        {
            Serial.println("GPS SIGNAL QUALITY OK");
            Serial.print("HDOP: ");
            Serial.println(hdop);
            return true; // Retourne true pour indiquer que la qualité du signal GPS est bonne
        }

        else
        {
            Serial.println("GPS SIGNAL QUALITY NOK");
            Serial.print("HDOP: ");
            Serial.println(hdop);
            play_buzzer(NOTE_D2, 500);
            delay(1000); // Attendre 1000 ms avant de réessayer
        }
    }
}

void GPS_set_home()
{

    if (gps.location.isValid())
    {
        home_point.latitude = gps.location.lat();
        home_point.longitude = gps.location.lng();
        home_point.altitude = current_barometer_altitude;

        waypointData[0].latitude = home_point.latitude;
        waypointData[0].longitude = home_point.longitude;
        waypointData[0].altitude = home_point.altitude;
    }

    Serial.println("HOME DEFINED");
    Serial.print("Latitude: ");
    Serial.println(waypointData[0].latitude);
    Serial.print("Longitude: ");
    Serial.println(waypointData[0].longitude);
    Serial.print("Altitude: ");
    Serial.println(waypointData[0].altitude);
};



void updateTraveledDistance() {
  if (gps.location.isUpdated()) {

    if (last_latitude != 0.0 && last_longitude != 0.0) {
      traveledDistance +=  TinyGPSPlus::courseTo(last_latitude, last_longitude, current_latitude, current_longitude);
 
    }

    last_latitude = current_latitude;
    last_longitude = current_longitude;
  }
}

// Vérifier si le waypoint est atteint (distance horizontale et altitude)
bool isWaypointReached(EEPROM_Waypoint current_wayppoint) {
  bool altitudeValid = true;

  // Si l'altitude doit être respectée, vérifier la différence d'altitude
  if (current_wayppoint.altHold) {
    altitudeValid = abs(current_barometer_altitude - target_altitude) <= altitude_threshold;
  }

  return (waypoint_distance <= distance_threshold) && altitudeValid;
}