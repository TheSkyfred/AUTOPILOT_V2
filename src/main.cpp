#include <Arduino.h>

#include "def.h"

#include "Wire.h"

#include "config.h"
#include "sensors.h"
#include "modes.h"

#ifdef MYSERIAL
#include "mySerial.h"
#endif

#ifdef SCREEN
#include "screen.h"
#endif

#ifdef MYBLUETOOTH
#include "myBluetooth.h"
#endif

#ifdef ESPNOW
#include "telemetry.h"
#endif

#include "battery.h"
#include "buzzer.h"
#include "GPS.h"
#include "barometer.h"
#include "sonar.h"
#include "stabilization.h"
#include "engine.h"
#include "myDriver.h"
#include "waypoints.h"

#include <EEPROM.h> // Bibliothèque pour gérer l'EEPROM

FlightMode currentMode = INIT;
EngineStatus currentEngineStatus = ENGINE_OFF;
SpeedMode currentSpeedMode = COEFFICIENT_CALC;
NavigationMode currentNavigationMode = STABILIZING;
TurnMode currentTurnMode = NO_TURN;
LandingMode currentLandingMode = INITIAL_APPROACH;

unsigned long lastPrint = 0;
bool isTurning = false;
bool isCorrectingAltitude = false;
float avgDistance = 0;

static unsigned long last_heartbeat = 0;

// BATTERY:
unsigned long previousMillis = 0;
unsigned long lastBatteryCheckMillis = 0; // Temps du dernier contrôle de batterie

// Variables pour mesurer la fréquence du loop
unsigned long startTime = 0;     // Pour enregistrer le temps précédent
unsigned long endTime = 0;       // Pour enregistrer le temps actuel
unsigned long executionTime = 0; // Pour enregistrer le temps actuel

void setup()
{

  // Initialiser l'EEPROM
  EEPROM.begin(EEPROM_SIZE);

  // Charger les paramètres de l'EEPROM
  EEPROM.get(0, MLWaypoints);

  Wire.begin();
  Wire.setClock(400000); // Définir le taux de données I2C à 400 kHz

  pinMode(BUZZERPIN, OUTPUT);

#ifdef MYSERIAL
  Serial.begin(115200);
  Serial.println("SERIAL WORKING");
  delay(10);
#endif

#ifdef SCREEN
  init_screen();
#endif

#ifdef ESPNOW
//  init_telemetry();
#endif

#ifdef BLUETOOTH
  init_bluetooth();
#endif

  init_sensors();
  delay(100);
  init_buzzer();
  delay(2000);

  init_PID();

  GPS_signal();
}

void loop()
{
  startTime = millis();
  unsigned long currentMillis = millis();
  sendHeartbeat(); // SEND HEARBEAT TO TELEMETRY

  // update_telemetry();
  update_sensors();

  switch (currentMode)
  {

  case INIT:

    // Envoyer périodiquement des messages HEARTBEAT pour indiquer que le système est actif
    if (millis() - last_heartbeat > TELEMETRY_INTERVAL)
    {
      sendHeartbeat();
      last_heartbeat = millis();
    }

    GPS_set_home();     // We Define the Home Point Position (Latitude, longitude, altitude)
    update_waypoints(); // Update GPS points with Home Point

    calculateTotalDistance(); // Calculer la distance totale (optionnel)
    check_altitude();

    battery_initial_check(); // See to do this only one time

    currentMode = ARMED; // Passer en mode ARMED

    break; // INIT

  case ARMED:

    // Envoyer périodiquement des messages HEARTBEAT pour indiquer que le système est actif
    if (millis() - last_heartbeat > TELEMETRY_INTERVAL)
    {
      sendHeartbeat();
      last_heartbeat = millis();
    }

    // BUZZER HELP
    target_pitch = takeoff_angle;

    // INSIDE RANGE : READY FOR TAKEOFF
    if (current_pitch >= (takeoff_angle - 2) && current_pitch <= (takeoff_angle + 2))
    {
      buzzer_mute();

      currentMode = TAKEOFF;
      break;
    }

    // OUTSIDE RANGE : WAIT FOR GOOD TAKEOFF ANGLE

    else if (current_pitch >= (takeoff_angle - takeoff_tolerance) && current_pitch <= (takeoff_angle + takeoff_tolerance))
    {
      // Calcul de la fréquence en fonction de l'angle
      float relativeAngle = abs(current_pitch - takeoff_angle);
      buzzer_frequency = map(relativeAngle, 0, takeoff_tolerance, 2000, 500); // Plus l'angle est proche de 15°, plus la fréquence est haute
      // takeoff_buzzer(buzzer_frequency);
    }

    else
    {
      buzzer_mute();
    }

    break; // ARMED

  case TAKEOFF:
    target_roll = 0;
    target_pitch = takeoff_angle;
    // YAW BLOCKED

    // FOR TEST ONLY REMOVE FOR REAL FLIGHT :
    if (current_pitch > 40)
    {
      currentMode = NAVIGATE; // Go to NAVIGATE Mode
      break;
    }

    // check if current_altitude > home_altitude + 20m
    else if (current_GPS_altitude > home_point.altitude + takeoff_security_altitude) // REPLACE BY IF !!
    {
      currentMode = NAVIGATE; // Go to NAVIGATE Mode
      break;
    }

    else
    {
      // STABILIZE THE PLANE
      stabilize(false, target_roll, false, 0, false, target_pitch, false, 0, true, 0, false, 0);
    }

    break; // TAKEOFF

  case CALIBRATE_SPEED:

    break; // CALIBRATE_SPEED

    //////////NAVIGATE////////////
  case NAVIGATE:

    switch (currentNavigationMode)
    {

    case FLYING:
      updateTraveledDistance(); // See if we do this only during FLYING mode or during Navigate Mode

      // Vérifier la batterie toutes les minutes
      if (currentMillis - lastBatteryCheckMillis >= batteryCheckInterval)
      {
        lastBatteryCheckMillis = currentMillis;
        battery_check();
      }

      if (isTurning)
      {
        currentNavigationMode = CORRECTING_HEADING;
      }
      else if (isCorrectingAltitude)
      {
        currentNavigationMode = CORRECTING_ALTITUDE;
      }
      else if (!stabilized)
      {
        currentNavigationMode = STABILIZING;
      }

      else if (abs(heading_error) > heading_tolerance)
      {
        currentNavigationMode = CORRECTING_HEADING;
        break;
      }

      else if (abs(altitude_error) > altitude_tolerance)
      {
        calculate_altitude_factor();
        currentNavigationMode = CORRECTING_ALTITUDE;
        break;
      }

      else
      {
        Serial.println("ENJOY THE FLIGHT");
      }

      break;

    case CORRECTING_HEADING:

      isTurning = true;

      if (abs(heading_error) < heading_tolerance)
      {
        isTurning = false;
        currentNavigationMode = STABILIZING;
      }

      switch (currentTurnMode)
      {
      case NO_TURN:

        if (abs(heading_error) < yaw_turn_range)
        {
          isTurning = true;
          currentTurnMode = YAW_CORRECTION;
          // We do a yaw_turn
          break;
        }
        else
        {
          isTurning = true;
          currentTurnMode = BANKED_TURN;
          break;
        }

        break;

      case BANKED_TURN:

        // Check if the banked turn is DONE
        if (abs(heading_error) < yaw_turn_range && abs(current_roll) < roll_tolerance)
        {
          isTurning = false;
          currentNavigationMode = FLYING;
        }

        target_roll = map(heading_error, -180, 180, -max_roll_angle, max_roll_angle);
        target_pitch = 0; // BLOCKED
        target_yaw = 0;   // BLOCKED

        stabilize(false, target_roll, true, roll_step_value, true, 0, false, 0, true, 0, false, 0);

        break;

      case YAW_CORRECTION:

        if (abs(heading_error) < heading_tolerance)
        {
          isTurning = false;
          currentNavigationMode = FLYING;
        }

        target_roll = 0;
        target_pitch = 0;
        target_yaw = target_heading;
        break;

      default:
        Serial.println("Erreur Turn Mode");
        break;
      }

      break; // END CORRECTING HEADING

    case CORRECTING_ALTITUDE:
      isCorrectingAltitude = true;

      // CHECK IF ALTITUDE CORRECTION IS DONE
      if (altitude_error < altitude_tolerance)
      {
        isCorrectingAltitude = false;
        currentNavigationMode = STABILIZING;
        break;
      }
      else
      {
        // correct altitude
        target_roll = 0;
        target_pitch = adjustPitchForAltitude(current_barometer_altitude, target_altitude);
        target_yaw = 0; // blocked

        stabilize(false, target_roll, false, 0, false, target_pitch, true, pitch_step_value, true, 0, false, 0);

        //   target_pitch = constrain((target_altitude - current_barometer_altitude), -max_roll_angle, max_roll_angle);
      }

      if (abs(altitude_error) < altitude_tolerance && current_pitch < pitch_tolerance)
      {
        isCorrectingAltitude = false;
        currentNavigationMode = FLYING;
      }
      break;

    case STABILIZING:
      target_roll = 0;
      target_pitch = 0;
      target_yaw = 0;

      check_stability();

      if (stabilized)
      {
        currentNavigationMode = FLYING;
      }
      break;

    case RECOVERY:
      // to code
      break;

    default:

      Serial.println("ERREUR NAVIGATION MODE");
      break;
    }

    break; // NAVIGATE
           //////////NAVIGATE - END////////////

  case LANDING:

    switch (currentLandingMode)
    {

    case INITIAL_APPROACH:

      if (current_barometer_altitude < (landing_altitude + 20.0) && current_barometer_altitude >= (landing_altitude + 8.0))
      {
        // Réduire la vitesse progressivement entre landing_altitude+20m et landing_altitude+8m
        target_speed = map(current_barometer_altitude, landing_altitude + 20.0, landing_altitude + 8.0, current_GPS_speed, speed_max);
        target_speed = constrain(target_speed, speed_max, current_GPS_speed);
      }

      if (current_barometer_altitude < (landing_altitude + 8.0))
      {
        currentMode = LANDING;
      }

      break;

    case FINAL_APPROACH:

      // Mesurer la distance avec le sonar
      avgDistance = getAverageDistance();

      if (avgDistance <= 60.0)
      {
        stopMotor();                // Arrêter le moteur lorsque la distance est inférieure à 60 cm
        currentLandingMode = FLARE; // Se mettre en position de planner
      }
      else
      {
        // Réduire progressivement la vitesse jusqu'à 30% de FLIGHTSPEED
        target_speed = map(avgDistance, 60.0, 0.0, speed_max, speed_min);
        target_speed = max(float(current_GPS_speed), speed_min); // Ne pas descendre en dessous de 30% de FLIGHTSPEED
      };

      engine_update(target_speed); // On met à jour la vitesse du moteur

      break;

    case FLARE:

      // Motor already OFF

      target_roll = 0;
      target_pitch = flare_angle;
      target_yaw = 0;

      stabilize(false, target_roll, false, 0, false, target_pitch, true, pitch_step_value, true, 0, false, 0);

      break;

    case TOUCHDOWN:
      // to code

      break;

    default:

      Serial.println("ERREUR LANDING MODE");

      break;
    }
  default:

    Serial.println("ERREUR MAIN MODE");
    break;

  } // End Switch Main Mode

  // LOOP TIMER
  endTime = millis();
  executionTime = endTime - startTime;

  // AFFICHAGE :
  unsigned long currentBatteryMillis = millis();
  if (currentBatteryMillis - lastPrint >= (PRINT_INTERVAL / 4))
  {
    update_screen(); // UPDATE SCREEN
  }
  if (currentBatteryMillis - lastPrint >= PRINT_INTERVAL)
  {

    lastPrint = currentBatteryMillis;

    display_DATA(); // UPDATE SERIAL
  }
}
