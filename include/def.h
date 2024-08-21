#ifndef DEF_H_
#define DEF_H_

#define SerialMonitor

extern const double myPI;

extern bool stabilized;

extern int current_heading;
extern int target_heading;

extern double target_latitude;
extern double target_longitude;
extern double target_altitude;
extern double target_speed;

extern int currentWPIndex;

extern float roll_correction, pitch_correction, yaw_correction;

// SPEED
extern float speed_min;
extern float speed_max;
extern float FLIGHTSPEED;
extern float full_motor_speed;

// TAKEOFF
extern int takeoff_angle;
extern int takeoff_security_altitude;
// GYROSCOPE

extern int current_roll, current_pitch, current_yaw;
extern int target_roll, target_pitch, target_yaw;

// LANDING
extern float landing_altitude;

// BATTERY
extern const float V_MIN;
extern const float V_MAX;

// BUZZER
extern float buzzer_frequency;

// SERVOS
extern const int SERVOMAX, SERVOMID, SERVOMIN;

// STABILIZATION
extern bool isStable;

extern double RollInput, RollOutput, RollSetpoint, rollKp, rollKi, rollKd;
extern double PitchInput, PitchOutput, PitchSetpoint, pitchKp, pitchKi, pitchKd;
extern double YawInput, YawOutput, YawSetpoint, yawKp, yawKi, yawKd;
extern bool is_stable;
extern bool roll_block_condition, pitch_block_condition, yaw_block_condition;
extern float roll_correction, pitch_correction, yaw_correction;
extern bool roll_step_condition, pitch_step_condition, yaw_step_condition;
extern float roll_step_value, pitch_step_value, yaw_step_value;

extern const int yaw_turn_range;
extern const int max_roll_angle;

// GPS & CORRECTIONS

extern double current_latitude;
extern double current_longitude;

extern float home_latitude;  // A PASSER EN DOUBLE
extern float home_longitude; // A PASSER EN DOUBLE
extern float home_altitude;

extern float landing_altitude;

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

// TOLERANCE
extern int altitude_tolerance; // Tolerance in meter
extern int takeoff_tolerance;
extern const int heading_tolerance;
extern const int STABILIZATION_TOLERANCE;
extern const unsigned long STABILIZATION_PERIOD;

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
  int current_Heading;
  int current_roll;
  int current_pitch;
  int current_yaw;
  int satellites;
  // float currentMode;

} struct_message;

extern struct_message sentData;

// Points de la mission
typedef struct Waypoint
{
  double latitude;
  double longitude;
  double altitude;
} Waypoint;

extern Waypoint waypointData[];
extern Waypoint home_point;
extern int numWaypoints;
extern int current_waypoint_index;

// DISTANCE
extern double totalDistance;

#endif /* DEF_H_ */