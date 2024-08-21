
#include "sonar.h"


#include "def.h"
#include "config.h"

DistanceSensor sonar(TRIG_PIN, ECHO_PIN);

int sonar_num_samples = 10; // Nombre de mesures pour la moyenne du sonar
float distance = 0;

// Function to get the sonar distance in cm
int get_sonar()
{
  // Get distance in cm from the sensor
  distance = sonar.getCM();
  return distance;
};

float getAverageDistance()
{
  float total_sonar_distance = 0;
  for (int i = 0; i < sonar_num_samples; i++)
  {
    delay(50); // Attendre entre les mesures
    distance = sonar.getCM();
    total_sonar_distance += distance;
  }
  return total_sonar_distance / sonar_num_samples;
}


