#include <Arduino.h>

#include "def.h"
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

FlightMode currentMode = INIT;
EngineStatus currentEngineStatus = ENGINE_OFF;
SpeedMode currentSpeedMode = COEFFICIENT_CALC;
NavigationMode currentNavigationMode = FLYING;
TurnMode currentTurnMode = NO_TURN;
LandingMode currentLandingMode = INITIAL_APPROACH;

unsigned long lastPrint = 0;
bool isTurning = false;
bool isCorrectingAltitude = false;
float avgDistance = 0;

void setup()
{

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

  // battery_initial_check();
  init_PID();
}

void loop()
{

  // FIRST : update telemetry
  // update_telemetry();
  update_sensors();
  // Then :

  switch (currentMode)
  {

  case INIT:

    // Check for GPS Signal Quality
    if (GPS_signal())
    {
      GPS_set_home();     // We Define the Home Point Position (Latitude, longitude, altitude)
      update_waypoints(); // Update GPS points with Home Point

      calculateTotalDistance(); // Calculer la distance totale (optionnel)

      // RGB_LED(BLUE);         // Définit la couleur bleue (optionnel)

      currentMode = ARMED; // Passer en mode ARMÉ (optionnel)

    }; // if GPS Quality is ok, then go to ARMED (see GPS function)

    break;

  case ARMED:

    // wait for instruction

    // BUZZER HELP
    target_pitch = takeoff_angle;

    if (current_pitch >= (takeoff_angle - 2) && current_pitch <= (takeoff_angle + 2))
    {
      currentMode = TAKEOFF;
      buzzer_mute();
    }

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

    break;

  case TAKEOFF:

    target_roll = 0;
    target_pitch = takeoff_angle;
    target_yaw = 0;

    // FOR TEST ONLY REMOVE FOR REAL FLIGHT :
    if (current_pitch > 50)
    {
      currentMode = NAVIGATE; // Go to NAVIGATE Mode
      target_roll = 0;
      target_pitch = 0;
      pitch_step_value = 5; // Remise à plat progressive
      target_yaw = 0;
      //  stabilize(false, target_roll, false, 0, false, target_pitch, true, pitch_step_value, false, 0, true, 0); // Stab R:0, P:takeoff_angle, Y:blocked
    }

    // check if current_altitude > home_altitude + 20m
    else if (current_GPS_altitude > home_point.altitude + takeoff_security_altitude) // REPLACE BY IF !!
    {

      // FOR TEST MODE :
      currentMode = NAVIGATE; // Go to NAVIGATE Mode

      target_roll = 0;
      target_pitch = 0;
      pitch_step_value = 5; // Remise à plat progressive
      target_yaw = 0;
      // stabilize(false, target_roll, false, 0, false, target_pitch, true, pitch_step_value, false, 0, true, 0); // Stab R:0, P:takeoff_angle, Y:blocked

      // USE THIS FOR REAL FLIGHT :
      /*
      if (engine_coefficient == 0) // Check is engine_coefficient is defined or not
      {
        currentMode = CALIBRATE_SPEED; // Go to CALIBRATE_SPEED Mode
      }
      else
      {
        currentMode = NAVIGATE; // Go to NAVIGATE Mode
      }
      */    }

      else
      {
        // stabilize(false, target_roll, false, 0, false, target_pitch, false, 0, true, 0, false, 0); // Stab R:0, P:takeoff_angle, Y:blocked
        set_engine(full_motor_speed); // set engine to full speed
      }

      break;

  case CALIBRATE_SPEED: // A DEVELOPPER

    switch (currentSpeedMode)
    {

    case COEFFICIENT_CALC:

      break;

    case SPEED_APPROACH:

      break;

    case SPEED_PID_TUNING:

      break;
    }

    // stabilize(false, 0, false, 0, false, takeoff_angle, false, 0, false, 0, true, 0); // Stab R:0, P:takeoff_angle, Y:blocked

    // if speed calibrated, go to NAVIGATE mode
    break;

    ///////////////////////////NAVIGATION///////////////////
  case NAVIGATE:

    check_stability();

    switch (currentNavigationMode)
    {

    case FLYING:

      if (isTurning)
      {
        currentNavigationMode = CORRECTING_HEADING;
      }
      else if (isCorrectingAltitude)
      {
        currentNavigationMode = CORRECTING_ALTITUDE;
      }
      else if (abs(heading_error) > heading_tolerance)
      {
        currentNavigationMode = CORRECTING_HEADING;
      }
      else if (abs(altitude_error) > altitude_tolerance)
      {
        currentNavigationMode = CORRECTING_ALTITUDE;
      }

      else
      {
        if (!stabilized)
        {
          currentNavigationMode = STABILIZING;
        }
      }

      break;

    case STABILIZING:

      if (stabilized)
      {
        currentNavigationMode = FLYING;
      }
      else
      {
        // stabilize(false, 0, false, 0, false, 0, false, 0, false, 0, true, 0);
      }

      break;

    case CORRECTING_HEADING:

      Serial.print(currentTurnMode);

      switch (currentTurnMode)
      {

        // 1 - check if plane is already turning
      case NO_TURN:

        // Check if the plane is stable

        if (stabilized)
        {

          if (abs(heading_error) < heading_tolerance)
          {
            currentNavigationMode = FLYING;
            break;
          }
          else if (heading_tolerance < abs(heading_error) || abs(heading_error) > yaw_turn_range)
          {
            currentTurnMode = YAW_CORRECTION;
            // We do a yaw_turn
          }
          else
          {

            // We do a banked_turn
            currentNavigationMode = CORRECTING_HEADING;
            currentTurnMode = BANKED_TURN;
          }
        }
        else
        {
          target_roll = 0;
          target_pitch = 0;
          target_yaw = 0;
          // stabilize(false, target_roll, false, 0, false, target_pitch, false, 0, false, target_yaw, true, 0); // A CORRIGER
          currentNavigationMode = STABILIZING;
        }

        ;

        break;

      case BANKED_TURN:

        target_roll = current_roll - constrain((heading_error - current_yaw) / 2, -max_roll_angle, max_roll_angle);
        target_pitch = 0;
        stabilize(false, target_roll, false, 0, false, target_pitch, false, 0, true, 0, false, 0); // A CORRIGER

        break;

      case YAW_CORRECTION:

        break;
      }

      break; //  case CORRECTING_HEADING

    case CORRECTING_ALTITUDE:

      if (stabilized)
      {

        check_altitude();

        if (altitude_error < altitude_tolerance)
        {

          currentMode = NAVIGATE;
        }
        else
        {
          // correct altitude
        }
      }
    }

    break;

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
        stopMotor(); // Arrêter le moteur lorsque la distance est inférieure à 60 cm
                     // Se mettre en position de planner
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
      // to code

      break;

    case TOUCHDOWN:
      // to code

      break;
    }
  }

  set_servos(RollOutput, PitchOutput, YawOutput);

  // AFFICHAGE :
  unsigned long currentMillis = millis();
  if (currentMillis - lastPrint >= PRINT_INTERVAL)
  {

    lastPrint = currentMillis;
    update_screen(); // UPDATE SCREEN

    display_DATA(); // UPDATE SERIAL
  }
}

void display_DATA()
{

  Serial.print("Mode:");
  Serial.print(FlightModeToString(currentMode));
  Serial.print("- WP:");
  Serial.print(current_waypoint_index);
  Serial.print(F(" - t_lat:"));
  Serial.print(target_latitude);
  Serial.print(F(" - t_long:"));
  Serial.print(target_longitude);

  switch (currentMode)
  {
  case INIT:

    Serial.print(F(" -BMPAlt:"));
    Serial.print(current_barometer_altitude);
    Serial.print(F(" -Temp:"));
    Serial.println(barometer_temperature);
    break;

  case ARMED:
    Serial.print("-Heading:");
    Serial.print(current_heading);
    Serial.print(F("-Heading Target = "));
    Serial.print(target_heading);
    Serial.print(" -Roll:");
    Serial.print(current_roll);
    Serial.print("- Pitch :");
    Serial.print(current_pitch);
    Serial.print("-Target Pitch:");
    Serial.print(target_pitch);
    // Serial.print("-Hauteur sol:");
    // Serial.print(current_GPS_altitude - home_altitude);
    Serial.print("-Alti GPS:");
    Serial.print(current_GPS_altitude);
    Serial.print("-Alti BME:");
    Serial.println(current_barometer_altitude);
    break;

  case TAKEOFF:

    Serial.print("-Heading:");
    Serial.print(current_heading);
    Serial.print(F("-Heading Target = "));
    Serial.print(target_heading);
    Serial.print("- Pitch :");
    Serial.print(current_pitch);
    Serial.print("-Target Pitch:");
    Serial.print(target_pitch);
    // Serial.print("-Hauteur sol:");
    // Serial.print(current_GPS_altitude - home_altitude);
    Serial.print("-Alti GPS:");
    Serial.print(current_GPS_altitude);
    Serial.print("-Alti BME:");
    Serial.print(current_barometer_altitude);
    Serial.print(" -Roll:");
    Serial.print(current_roll);
    Serial.print(" -Pitch:");
    Serial.print(current_pitch);
    Serial.print(" -Yaw:");
    Serial.println(current_yaw);

    break;

  case CALIBRATE_SPEED:
    Serial.print(" CALIBRATION SPEED TEXT");

    break;

  case NAVIGATE:

    Serial.print(" currentNavMode: ");
    Serial.print(NavigationModeToString(currentNavigationMode));

    Serial.print(" -TRoll:");
    Serial.print(target_roll);
    Serial.print(" -TPitch:");
    Serial.print(target_pitch);
    Serial.print(" -TYaw:");
    Serial.println(target_yaw);

    break;

  case LANDING:

    break;
  }

  /*Serial.print(F(" ; Press = "));
  Serial.print(pressure);
  Serial.println(" hPa");
  */

  /*    Serial.print("Error:");
      Serial.print(headingError);
      Serial.print(F("Angle = "));
      Serial.print(angle);
      */

  /*
      Serial.print(F(" ; RollInput = "));
      Serial.print(RollInput);
      Serial.print(F(" ; RollOutput = "));
      Serial.print(RollOutput);

      Serial.print(F(" ; RollservoPosition = "));
      Serial.print(RollservoPosition);
      Serial.print(F(" ; PitchInput = "));
      Serial.print(PitchInput);
      Serial.print(F(" ; PitchOuput = "));
      Serial.print(PitchOutput);
      Serial.print(F(" ; PitchservoPosition = "));
      Serial.print(PitchservoPosition);
      Serial.print(F(" ; YawInput = "));
      Serial.print(YawInput);
      Serial.print(F(" ; YawOutput = "));
      Serial.print(YawOutput);

      Serial.print(F(" ; YawservoPosition = "));
      Serial.println(YawservoPosition);
      */
}