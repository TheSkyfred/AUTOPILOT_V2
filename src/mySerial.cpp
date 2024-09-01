
#include "mySerial.h"
#include "modes.h"

#include "config.h"

void display_DATA()
{
  /*
Serial.print("Fr: ");
Serial.print(executionTime);
Serial.print(" Hz-");
Serial.print("S:");
Serial.print(stabilized);
*/

  Serial.print("-Mode:");
  Serial.print(FlightModeToString(currentMode));
  Serial.print("- WP:");
  Serial.print(current_waypoint_index);

  Serial.print("- Heading Er:");
  Serial.print(heading_error);
  /*
  Serial.print(F(" - t_lat:"));
  Serial.print(target_latitude);
  Serial.print(F(" - t_long:"));
  Serial.print(target_longitude);
*/
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
    Serial.println("TAKEOFF IN PROGRESS");

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
    Serial.print(" NavMode: ");
    Serial.print(NavModeToString(currentNavigationMode));

    switch (currentNavigationMode)
    {

    case FLYING:
      break;
    case CORRECTING_HEADING:
      Serial.print("- TurnMode: ");
      Serial.print(TurnModeToString(currentTurnMode));
      Serial.print("Heading Error :");

      Serial.print(heading_error);

      break;
    case CORRECTING_ALTITUDE:

      Serial.print("- Alt Error: ");
      Serial.print(altitude_error);
      Serial.print("- K_Alt: ");
      Serial.print(K_altitude);

      break;
    case STABILIZING:
      break;

      break;
    }

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

/*
PrintToSerial (){

// Initialiser les données de vol pour les essais
sentData.current_latitude = current_latitude;
sentData.current_longitude = current_longitude;
sentData.satellites = Satellites;

Serial.print(F("- latitude: "));
Serial.print(current_latitude, 7);

Serial.print(F("- longitude: "));
Serial.print(current_longitude, 7);

Serial.print(F("- altitude: "));

sentData.current_GPS_speed = current_GPS_speed;
Serial.print(gps.speed.kmph());
Serial.println(F(" km/h"));

sentData.current_GPS_altitude = current_GPS_altitude;

}
*/