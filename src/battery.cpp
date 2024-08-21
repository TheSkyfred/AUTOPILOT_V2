#include "battery.h"
#include "def.h"
#include "config.h"

#include "GPS.h"




// Instances
INA226 INA(battery_address);

// Variables globales
float totalEnergy = 0.0; // en mWh
float lastEnergyMeasurement = 0.0; // Energie mesurée précédemment en mWh

unsigned long prevMillis = 0;

// Paramètres de mesure
const long interval = 10000; // Intervalle de mesure en millisecondes (10 secondes)


void   battery_initial_check(){


};


void   battery_check(){

    unsigned long currentMillis = millis();
    
    // Mesurer la batterie et la distance
    if (currentMillis - prevMillis >= interval) {
      prevMillis = currentMillis;
      
      // Mise à jour de la consommation d'énergie
      float currentEnergy = readBatteryEnergy(); // Mesure actuelle de la batterie en mWh
      float energyConsumed = currentEnergy - lastEnergyMeasurement;
      lastEnergyMeasurement = currentEnergy;
      
      totalEnergy += energyConsumed;

    }
};


void battery_left(){

/*
      // Calcul et affichage de la consommation par km
      float consumptionPerKm = calculateConsumptionPerKm();
      Serial.print("Total Distance (m): ");
      Serial.println(totalDistance);
      Serial.print("Total Energy (mWh): ");
      Serial.println(totalEnergy);
      Serial.print("Consumption per km (mWh/km): ");
      Serial.println(consumptionPerKm);

      // Vérifier si la batterie est suffisante pour terminer la mission
      float remainingDistance = calculateTotalDistance() - totalDistance;
      float requiredEnergyForMission = calculateBatteryUsage(remainingDistance);
      if (calculateBatteryPercentage() < requiredEnergyForMission * (1 + safetyMargin)) {
        Serial.println("Battery insufficient for mission. Returning to home.");
        returnToHome();
        return; // Sortir de la boucle principale pour éviter toute autre action
      }

      // Vérifier si la batterie est suffisante pour retourner au point HOME
      float remainingDistanceToHome = calculateDistance(lastLatitude, lastLongitude, home.latitude, home.longitude);
      float requiredEnergyForHome = calculateBatteryUsage(remainingDistanceToHome);
      if (calculateBatteryPercentage() < requiredEnergyForHome * (1 + safetyMargin)) {
        Serial.println("Battery insufficient to return home. Returning to home.");
        returnToHome();
        return; // Sortir de la boucle principale pour éviter toute autre action
      }
      */
    }
  

float readVoltage() {
  return INA.getBusVoltage(); // Tension en Volts
}

float readCurrent() {
  return INA.getCurrent(); // Courant en mA
}

float readPower() {
  return INA.getPower(); // Puissance en mW
}

float readBatteryEnergy() {
  // Calcul de l'énergie de la batterie en mWh depuis le dernier appel
  return (readPower() * interval / 3600000.0); // Energie en mWh
}

float calculateBatteryPercentage() {
  float voltage = readVoltage(); // Obtenez la tension mesurée de la batterie
  return ((voltage - V_MIN) / (V_MAX - V_MIN)) * 100.0; // Pourcentage de batterie
}