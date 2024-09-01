#include "def.h"

const double myPI = 3.14159;

// ANGLES
int takeoff_angle = 15;
int langing_angle = 5;
int flare_angle = 10 ;// 5 à 10

// BATTERY
//  Paramètres de la batterie
const float V_MIN = 10.2; // Tension minimale en Volts
const float V_MAX = 12.6; // Tension pleine charge en Volts

// Paramètres de mesure
const float safetyBatteryMargin = 0.2;       // Coefficient de sécurité de 20%
const long batteryCheckInterval = 60 * 1000; // Intervalle de vérification de la batterie en millisecondes (60 secondes)

// BUZZER
float buzzer_frequency;

// TURNS
const int yaw_turn_range = 15;
const int max_roll_angle = 30;
const int max_pitch_angle = 30;
const int roll_tolerance = 2;
const int pitch_tolerance = 2;

// HEADING
const int heading_tolerance = 2;

// BAROMETER
int altitude_tolerance = 2; // Tolerance in meter

// SERVOS
//  Limites de sécurité des servos
const int SERVOMAX = 525;                 // Longueur d'impulsion maximale (180°)
const int SERVOMID = 365;                 // Longueur d'impulsion minimale (0°)
const int SERVOMIN = SERVOMAX - SERVOMID; // Longueur d'impulsion minimale (0°)

// TAKEOFF
int takeoff_security_altitude = 40;
int takeoff_tolerance = 2;

const int ECHO_PIN = 12;
const int TRIG_PIN = 13;
const int dhtPin = 2;
// #define SOUND_SPEED 340
// #define TRIG_PULSE_DURATION_US 10

// SPEED
float full_motor_speed = 100;
float FLIGHTSPEED = 40.0;             // vitesse de consigne
float speed_min = FLIGHTSPEED * 0.30; // 30% de FLIGHTSPEED
float speed_max = FLIGHTSPEED * 0.60; // 60% de FLIGHTSPEED

// LANDING
float landing_altitude = 0.00;

// SCREEN

// Stabilization
const unsigned long STABILIZATION_PERIOD = 3000; // 3 secondes
const int STABILIZATION_TOLERANCE = 4;
bool isStable = false;

float PID_TAKEOFF[] = {2.5, 0.5, 1.0};
float PID_NAVIGATE[] = {1.5, 1.0, 0.5};
float PID_LANDING[] = {1.0, 0.8, 0.3};

/*
//  Définir les paramètres de réglage du PID pour le roulis et le tangage
double rollKp = 1.03, rollKi = 0.000, rollKd = 2.0003;
double pitchKp = 1.02, pitchKi = 0.000, pitchKd = 2.0002;
double yawKp = 1.025, yawKi = 0.000, yawKd = 0.0002;
*/

// SONAR
int num_samples = 10; // Nombre de mesures pour la moyenne du sonar

// DISTANCE
// double totalDistance = 0;
float distance_threshold =   5.0 ; // Distance horizontale (en mètres) considérée comme atteinte pour un waypoint
float altitude_threshold = 2.0;  // Seuil de tolérance d'altitude (en mètres)

// TELEMETRY
