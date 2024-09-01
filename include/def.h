#ifndef DEF_H_
#define DEF_H_

#include <stdint.h>

extern const double myPI;

extern bool isTurning;
extern bool isCorrectingAltitude;
extern bool stabilized;

extern double current_heading;
extern int target_heading;

extern double target_latitude;
extern double target_longitude;
extern double target_altitude;
extern double target_speed;

extern int currentWPIndex;

extern double roll_correction, pitch_correction, yaw_correction;

// SPEED
extern float speed_min;
extern float speed_max;
extern float FLIGHTSPEED;
extern float full_motor_speed;

// TAKEOFF
extern int takeoff_angle;
extern int takeoff_security_altitude;
// GYROSCOPE

extern double current_roll, current_pitch, current_yaw;
extern double current_radian_roll, current_radian_pitch, current_radian_yaw;
extern double target_roll, target_pitch, target_yaw;

// LANDING
extern float landing_altitude;
extern int langing_angle;
extern int flare_angle;

// BATTERY
extern const float V_MIN;
extern const float V_MAX;
extern const long batteryCheckInterval;

// PID
extern float PID_TAKEOFF[], PID_NAVIGATE[], PID_LANDING[];

// BATTERY
extern const float safetyBatteryMargin;

// BUZZER
extern float buzzer_frequency;

// SERVOS
extern const int SERVOMAX, SERVOMID, SERVOMIN;

// STABILIZATION
extern bool isStable;

extern double RollInput, RollOutput, RollSetpoint, rollKp, rollKi, rollKd;
extern double PitchInput, PitchOutput, PitchSetpoint, pitchKp, pitchKi, pitchKd;
extern double YawInput, YawOutput, YawSetpoint, yawKp, yawKi, yawKd;
extern bool roll_block_condition, pitch_block_condition, yaw_block_condition;
extern double roll_correction, pitch_correction, yaw_correction;
extern bool roll_step_condition, pitch_step_condition, yaw_step_condition;
extern float roll_step_value, pitch_step_value, yaw_step_value;

extern const int yaw_turn_range;
extern const int max_roll_angle;
extern const int max_pitch_angle;

extern const int roll_tolerance;
extern const int pitch_tolerance;

// GPS & CORRECTIONS

extern int satellites_quantity;
extern double current_latitude;
extern double current_longitude;

extern float home_latitude;  // A PASSER EN DOUBLE
extern float home_longitude; // A PASSER EN DOUBLE
extern float home_altitude;

extern int heading_error;
extern unsigned long waypoint_distance;
extern float altitude_error;

extern double current_GPS_speed;
extern double current_GPS_altitude;

// NAVIGATION
extern float current_vertical_speed;
extern float current_vectorial_speed;

// BAROMETER
extern float current_barometer_altitude;
extern float barometer_temperature;

// Altitude Correction

extern float StartAltitudeError;
extern float StartWaypointDistance;
extern float K_altitude;

// TOLERANCE
extern int altitude_tolerance; // Tolerance in meter
extern int takeoff_tolerance;
extern const int heading_tolerance;
extern const int STABILIZATION_TOLERANCE;
extern const unsigned long STABILIZATION_PERIOD;

extern float distance_threshold; // Distance horizontale (en mètres) considérée comme atteinte pour un waypoint
extern float altitude_threshold; // Seuil de tolérance d'altitude (en mètres)

// ENGINE
extern const int motorSpeedMIN;
extern const int motorSpeedMAX;
extern double engine_coefficient;

extern const int ECHO_PIN;
extern const int TRIG_PIN;

// defining the data messages we want to sent to reciever
typedef struct struct_message
{
  // int bat;
  double current_latitude;
  double current_longitude;
  float current_GPS_altitude;
  float current_barometer_altitude;
  double current_GPS_speed;
  double current_Heading;
  int current_roll;
  int current_pitch;
  int current_yaw;

  int satellites;
  // float currentMode;

} struct_message;

extern struct_message sentData;

// Points de la mission
typedef struct EEPROM_Waypoint
{
  double latitude;
  double longitude;
  double altitude;
  int altHold;
} EEPROM_Waypoint;

extern EEPROM_Waypoint waypointData[];
extern EEPROM_Waypoint home_point;
extern EEPROM_Waypoint landing_point;
extern int numWaypoints;
extern int current_waypoint_index;

// DISTANCE
extern double traveledDistance;

// Structure pour stocker les informations des waypoints
struct MavLinkWayPoint
{
  float param1;
  float param2;
  float param3;
  float param4;
  int32_t x; // Latitude
  int32_t y; // Longitude
  float z;   // Altitude
  uint16_t seq;
  uint16_t command;
  uint8_t target_system;
  uint8_t target_component;
  uint8_t frame;
  uint8_t current;
  uint8_t autocontinue;
  uint8_t mission_type;
};

extern MavLinkWayPoint MLWaypoints[100];

struct MAVLinkMessageStructure
{
  uint8_t system_id;
  uint8_t component_id;

  // Position globale (GLOBAL_POSITION_INT)
  uint32_t time_boot_ms; // Temps depuis le démarrage en millisecondes
  int32_t lat;           // Latitude en degrés * 1E7
  int32_t lon;           // Longitude en degrés * 1E7
  int32_t alt;           // Altitude absolue en millimètres (par rapport au niveau de la mer)
  int32_t relative_alt;  // Altitude relative en millimètres (par rapport au terrain)
  int16_t vx;            // Vitesse dans l'axe X en cm/s
  int16_t vy;            // Vitesse dans l'axe Y en cm/s
  int16_t vz;            // Vitesse dans l'axe Z en cm/s
  uint16_t hdg;          // Cap en centi-degrés (0.01 degrés)

  // Attitude (ATTITUDE)
  float roll;       // Roulis en radians
  float pitch;      // Tangage en radians
  float yaw;        // Lacet en radians
  float rollspeed;  // Vitesse de roulis en radians/s
  float pitchspeed; // Vitesse de tangage en radians/s
  float yawspeed;   // Vitesse de lacet en radians/s

  // État de la batterie (BATTERY_STATUS)
  uint8_t battery_id;          // ID de la batterie
  uint8_t battery_function;    // Fonction de la batterie
  uint8_t battery_type;        // Type de batterie
  int16_t battery_temperature; // Température de la batterie en centi-degrés Celsius
  uint16_t voltages[4];        // Tension de chaque cellule de la batterie en millivolts
  int16_t current_battery;     // Courant de la batterie en 10 mA
  int32_t current_consumed;    // Courant total consommé en mAh
  int32_t energy_consumed;     // Énergie totale consommée en 1/1000 de Joules
  int8_t battery_remaining;    // Pourcentage de batterie restante (0 à 100)

  // État du système (HEARTBEAT)
  uint8_t autopilot_mode; // Mode de l'autopilote
  uint8_t base_mode;      // Mode de base du système
  uint32_t custom_mode;   // Mode personnalisé
  uint8_t system_status;  // État général du système

  // Mission (MISSION_CURRENT et MISSION_ITEM_REACHED)
  uint16_t current_mission_seq;   // Numéro du waypoint actuellement actif
  uint16_t current_mission_total; // Numéro du waypoint actuellement actif

  uint8_t mission_result; // Résultat de la mission ou du waypoint (succès, échec, etc.)
  uint8_t mission_state;
  uint8_t mission_mode;
  uint32_t mission_id;
  uint32_t fence_id;
  uint32_t rally_points_id;

  // Statut GPS (GPS_RAW_INT)
  uint8_t gps_fix_type;       // Type de correction GPS (2D, 3D, DGPS, etc.)
  uint8_t satellites_visible; // Nombre de satellites visibles

  // Vitesse air (VFR_HUD)
  float airspeed;    // Vitesse de l'air en m/s
  float groundspeed; // Vitesse par rapport au sol en m/s

  // Autres données pertinentes
  // Ajoute ici tout autre paramètre nécessaire à ton application
};

#endif /* DEF_H_ */