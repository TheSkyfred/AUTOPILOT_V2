// NAVIGATION FILE

#include "def.h"
#include "navigation.h"

#include "GPS.h"
#include "barometer.h"

// Variables pour stocker les données de vitesse et d'altitude
unsigned long last_time = 0;
double last_altitude_bme = 0.0;
double last_altitude_gps = 0.0;
double last_horizontal_speed = 0.0;
float current_time = 0.0;

// Calculer la variation d'altitude pour les deux capteurs
double delta_altitude_bme = current_barometer_altitude- last_altitude_bme;
double delta_altitude_gps = current_GPS_altitude - last_altitude_gps;

// Calculer le temps écoulé
double delta_time = (current_time - last_time) / 1000.0; // en secondes

// Calculer la vitesse verticale en m/s (BME280 et GPS)
double vertical_speed_bme = delta_altitude_bme / delta_time;
double vertical_speed_gps = delta_altitude_gps / delta_time;


// Calculer la vitesse vectorielle totale
double total_speed_bme = sqrt(pow(current_GPS_speed, 2) + pow(vertical_speed_bme, 2));
double total_speed_gps = sqrt(pow(current_GPS_speed, 2) + pow(vertical_speed_gps, 2));

// VERTICAL SPEED FROM BME280
void calculate_vertical_speed()
{

    unsigned long current_time = millis();

    // Calculer la fréquence de boucle à 10 Hz
    if (current_time - last_time >= 100)
    { // 100 ms = 10 Hz


        // Calculer la variation d'altitude pour les deux capteurs
        double delta_altitude_bme = current_barometer_altitude - last_altitude_bme;

        // Calculer le temps écoulé
        double delta_time = (current_time - last_time) / 1000.0; // en secondes

        // Calculer la vitesse verticale en m/s (BME280 et GPS)
        double vertical_speed_bme = delta_altitude_bme / delta_time;
    }
}

// VECTORIAL SPEED FROM HORIZONTAL & VERTICAL SPEED
// Fonction pour calculer la vitesse vectorielle
float calculate_vectorial_speed(float current_GPS_speed_meters, float vertical_speed_bme) {
  return sqrt(pow(current_GPS_speed_meters, 2) + pow(vertical_speed_bme, 2));
}
