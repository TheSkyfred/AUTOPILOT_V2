#include "battery.h"
#include "def.h"
#include "config.h"

#include "GPS.h"

#include "waypoints.h"

// Instances
INA226 INA(battery_address);

// Variables globales
float totalEnergy = 0.0;           // en mWh
float lastEnergyMeasurement = 0.0; // Energie mesurée précédemment en mWh

unsigned long prevMillis = 0;

// Paramètres de mesure
const long interval = 10000; // Intervalle de mesure en millisecondes (10 secondes)

void init_battery()
{

  // Initialisation de l'INA226
  if (!INA.begin())
  {
    Serial.println("Impossible de trouver l'INA226. Vérifiez les connexions !");
    while (1)
      ;
  }
  // INA.setCalibration_32V_2A(); // Calibration pour une gamme de 32V et 2A
  Serial.println("Battery Tester Ready !");
}

void battery_initial_control()
{

  // Vérification initiale de la batterie
  if (!battery_initial_check())
  {
    Serial.println("Battery too low for this mission.");
    while (1)
      ; // Arrêter l'exécution si la batterie est insuffisante
  }
  else
  {
    Serial.println("Battery OK - Ready for this mission.");
  }
}

bool battery_initial_check()
{
  // Calculer la distance totale de la mission
  double totalMissionDistance = calculateTotalDistance();

  // Calculer la consommation d'énergie estimée pour la mission
  float estimatedEnergyForMission = calculateBatteryUsage(totalMissionDistance);

  // Calculer la distance de retour au point HOME depuis l'atterrissage
  double distanceToLanding = TinyGPSPlus::courseTo(current_latitude, current_longitude, landing_point.latitude, landing_point.longitude);
  // Calculer la consommation d'énergie estimée pour retourner au point HOME
  float estimatedEnergyToLanding = calculateBatteryUsage(distanceToLanding);

  // Calculer l'énergie totale nécessaire
  float totalRequiredEnergy = estimatedEnergyForMission * (1 + safetyBatteryMargin);

  // Vérifier si la batterie est suffisante
  float initialBatteryPercentage = calculateBatteryPercentage();

  Serial.println("totalRequiredEnergy : ");
  Serial.println(totalRequiredEnergy);
  Serial.println("initialBatteryPercentage : ");
  Serial.println(initialBatteryPercentage);
  Serial.println("estimatedEnergyToLanding : ");
  Serial.println(estimatedEnergyToLanding);

  if (initialBatteryPercentage >= totalRequiredEnergy)
  {
    return true; // Batterie suffisante
  }
  else
  {
    return false; // Batterie insuffisante
  }
}

void battery_check()
{
  unsigned long currentMillis = millis();

  // Mesurer la batterie et la distance
  if (currentMillis - prevMillis >= interval)
  {
    prevMillis = currentMillis;

    // Mise à jour de la consommation d'énergie
    float currentEnergy = readBatteryEnergy(); // Mesure actuelle de la batterie en mWh
    float energyConsumed = currentEnergy - lastEnergyMeasurement;
    lastEnergyMeasurement = currentEnergy;

    totalEnergy += energyConsumed;
  }

  // Vérifier si la batterie est suffisante pour terminer la mission
  float remainingDistance = calculateTotalDistance() - traveledDistance;
  float requiredEnergyForMission = calculateBatteryUsage(remainingDistance);
  if (calculateBatteryPercentage() < requiredEnergyForMission * (1 + safetyBatteryMargin))
  {
    Serial.println("Battery insufficient for mission. Going to Landing.");
    goToLanding();
    return; // Sortir de la boucle principale pour éviter toute autre action
  }

  // Vérifier si la batterie est suffisante pour retourner au point APPROACH
  float remainingDistanceToApproach = TinyGPSPlus::courseTo(current_latitude, current_longitude, landing_point.latitude, landing_point.longitude);
  float requiredEnergyToApproach = calculateBatteryUsage(remainingDistanceToApproach);
  if (calculateBatteryPercentage() < requiredEnergyToApproach * (1 + safetyBatteryMargin))
  {
    Serial.println("Battery insufficient to return home. Returning to home.");
    goToLanding();
    return; // Sortir de la boucle principale pour éviter toute autre action
  }
};

float readVoltage()
{
  return INA.getBusVoltage(); // Tension en Volts
}

float readCurrent()
{
  return INA.getCurrent(); // Courant en mA
}

float readPower()
{
  return INA.getPower(); // Puissance en mW
}

float readBatteryEnergy()
{
  // Calcul de l'énergie de la batterie en mWh depuis le dernier appel
  return (readPower() * interval / 3600000.0); // Energie en mWh
}

float calculateBatteryPercentage()
{
  float voltage = readVoltage();                        // Obtenez la tension mesurée de la batterie
  return ((voltage - V_MIN) / (V_MAX - V_MIN)) * 100.0; // Pourcentage de batterie
}

float calculateConsumptionPerKm()
{
  if (traveledDistance > 0.0)
  {
    return totalEnergy / traveledDistance; // Consommation par km en mWh/km
  }
  else
  {
    return 0.0; // Éviter la division par zéro
  }
}

float calculateBatteryUsage(double distance)
{
  float consumptionPerKm = calculateConsumptionPerKm();
  return consumptionPerKm * distance;
}