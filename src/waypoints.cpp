// THIS FILE IS FOR WAYPOINTS MANAGEMENT

/*

Waypoint Normal (MAV_CMD_NAV_WAYPOINT):

param1: Temps de maintien (en secondes) à ce waypoint.
param2: Rayon d'acceptation en mètres pour considérer que le waypoint est atteint.
param3: Indique si l'avion doit passer par ce waypoint (0) ou à une distance spécifiée.
param4: Angle de lacet désiré au waypoint.

Décollage (MAV_CMD_NAV_TAKEOFF):

param1: Inclinaison minimale ou désirée.
param2 à param4: Généralement non utilisés pour le décollage.
x, y, z: Latitude, longitude et altitude de décollage.

Atterrissage (MAV_CMD_NAV_LAND):

param1: Altitude d'abandon en cas d'atterrissage interrompu.
param2: Mode d'atterrissage de précision.
param3 et param4: Généralement non utilisés.
x, y, z: Latitude, longitude et altitude cible pour l'atterrissage.

*/
#include <Arduino.h> // Inclusion de la bibliothèque Arduino pour utiliser Serial

#include "waypoints.h"
#include "config.h"
#include "def.h"
#include "math.h"
#include "GPS.h"

#include <EEPROM.h>  // Bibliothèque pour gérer l'EEPROM

MavLinkWayPoint MLWaypoints[100];


double lastLatitude = 0.0;
double lastLongitude = 0.0;

// Variables pour les waypoints
int target_heading = 0;
double target_latitude = 0;
double target_longitude = 0;
double target_altitude = 0;
double totalDistance = 0;

int current_waypoint_index = 1; // On démarre au point #1 car le zéro est le home_point

// Déclaration globale des tableaux
float current_GPS[] = {0, 0, 0};
double target_GPS[] = {0, 0, 0}; // GPS cible

EEPROM_Waypoint home_point = {0, 0, 0};    // Coordonnées de départ
EEPROM_Waypoint landing_point = {0, 0, 0}; // Point d'atterrissage

EEPROM_Waypoint waypointData[] = {
    {0, 0, 0}, // point HOME
    {43.61327522970758, 3.843247992486303, 40},
    {43.61384303201776, 3.8317396672563824, 40},
    {43.60621397901305, 3.8191815299885015, 40}};

int numWaypoints = sizeof(waypointData) / sizeof(waypointData[0]);

void update_waypoints()
{

    /*
    // Met à jour les coordonnées GPS courantes (voir si nécessaire car déjà fait dans la définition du point Home)
    current_GPS[0] = current_latitude;
    current_GPS[1] = current_longitude;
    current_GPS[2] = current_GPS_altitude;
    */

    // Récupère les valeurs du waypoint actuel
    EEPROM_Waypoint current_wayppoint = getCurrentWaypoint();

    // Met à jour les cibles GPS avec les valeurs correctes
    target_latitude = current_wayppoint.latitude;   // Latitude cible
    target_longitude = current_wayppoint.longitude; // Longitude cible
    target_altitude = current_wayppoint.altitude;   // Altitude cible

    // La variable target_heading semble ne pas être utilisée dans cette fonction
    // Si vous avez besoin d'une logique pour target_heading, ajoutez-la ici
}

double calculateTotalDistance()
{

    EEPROM_Waypoint previousPoint = home_point;

    // Ajouter la distance entre chaque waypoint
    for (int i = 0; i < numWaypoints; i++)
    {
        totalDistance += (unsigned long)TinyGPSPlus::distanceBetween(previousPoint.latitude, previousPoint.longitude, waypointData[i].latitude, waypointData[i].longitude);
        previousPoint = waypointData[i];
    }

    Serial.print("Total Distance : ");
    Serial.print(totalDistance);
    Serial.println(" m ");
    return totalDistance;
}

double degreesToRadians(double degrees)
{
    return degrees * (myPI / 180.0);
}

EEPROM_Waypoint getCurrentWaypoint()
{
    if (current_waypoint_index >= 0 && current_waypoint_index < numWaypoints)
    {
        return waypointData[current_waypoint_index];
    }
    else
    {
        Serial.println("Erreur dans la gestion des Waypoints");
        return {0, 0, 0}; // Erreur: retourne un waypoint vide si l'index est hors de portée
    }
}

void next_waypoint()
{
    if (current_waypoint_index < numWaypoints - 1)
    {
        current_waypoint_index++; // Passer au waypoint suivant
    }
    else
    {
        Serial.println("Last waypoint reached. No more waypoints to navigate.");
        // Si vous voulez recommencer au premier waypoint, décommentez la ligne suivante
        // current_waypoint = 0;
    }
}

void goToLanding()
{
    Serial.println("GOING TO APPROACH POINT FOR LANDING");
    // current_waypoint_index = 0; // LANDING WP
}

// Useless function for landing Point - I'll see if I can do something with it
void updateLandingPoint()
{
    if (numWaypoints > 0)
    {
        // Met à jour les coordonnées du point d'atterrissage avec celles du dernier waypoint
        landing_point.latitude = waypointData[numWaypoints - 1].latitude;
        landing_point.longitude = waypointData[numWaypoints - 1].longitude;
        landing_point.altitude = waypointData[numWaypoints - 1].altitude;

        Serial.println("Point d'atterrissage mis à jour avec les coordonnées du dernier waypoint:");
        Serial.print("Latitude: ");
        Serial.println(landing_point.latitude);
        Serial.print("Longitude: ");
        Serial.println(landing_point.longitude);
        Serial.print("Altitude: ");
        Serial.println(landing_point.altitude);
    }
    else
    {
        Serial.println("Aucun waypoint dans la liste pour mettre à jour le point d'atterrissage.");
    }
}




// Enregistrer un waypoint dans l'EEPROM
void saveWaypointToEEPROM(const MavLinkWayPoint& MLWaypoints) {
    int address = sizeof(MLWaypoints) + MLWaypoints.seq * sizeof(MavLinkWayPoint);
    EEPROM.put(address, MLWaypoints);
    EEPROM.commit();
}

// Charger un waypoint depuis l'EEPROM
EEPROM_Waypoint loadWaypointFromEEPROM(uint16_t seq) {
    int address = sizeof(MLWaypoints) + seq * sizeof(MavLinkWayPoint);
    EEPROM_Waypoint current_ML_waypoint;
    EEPROM.get(address, current_ML_waypoint);
    return current_ML_waypoint;
}