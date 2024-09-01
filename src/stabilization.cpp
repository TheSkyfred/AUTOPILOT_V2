#include "stabilization.h"

#include "def.h"
#include "config.h"

#include "modes.h"

#include "gyroscope.h"
#include "config.h"
#include "GPS.h"

#include "myDriver.h"

// PID
double rollKp = 0, rollKi = 0.000, rollKd = 0.0;
double pitchKp = 0.0, pitchKi = 0.000, pitchKd = 0.0;
double yawKp = 0.0, yawKi = 0.000, yawKd = 0.0;

// KALMANFILTER
extern SimpleKalmanFilter xKalmanFilter;
extern SimpleKalmanFilter yKalmanFilter;
extern SimpleKalmanFilter zKalmanFilter;

// Variables pour l'orientation actuelle basée sur l'IMU
double target_roll = 0;
double target_pitch = 0;
double target_yaw = 0;

int prev_roll_correction = 0;
int prev_pitch_correction = 0;
int prev_yaw_correction = 0;

// Intput & Output
int acc_roll_input, acc_pitch_input;
int gyro_pitch_input, gyro_roll_input;
int pitch_output, roll_output;

// Corrections
double roll_correction = 0;
double pitch_correction = 0;
double yaw_correction = 0;

bool roll_block_condition = 0;
bool roll_step_condition = 0;
float roll_step_value = 0;
bool pitch_block_condition = 0;
bool pitch_step_condition = 0;
float pitch_step_value = 0;
bool yaw_block_condition = 0;
bool yaw_step_condition = 0;
float yaw_step_value = 0;

// Altitude corrections
float StartAltitudeError = 0;
float StartWaypointDistance = 0;
float K_altitude = 0;
float altitudeError = 0;

// Définir des variables pour le PID
double RollSetpoint, RollInput, RollOutput;
double PitchSetpoint, PitchInput, PitchOutput;
double YawSetpoint, YawInput, YawOutput;

// Objets PID avec entrées et sorties en tant que pointeurs
PID rollPID(&current_roll, &roll_correction, &target_roll, rollKp, rollKi, rollKd, DIRECT);
PID pitchPID(&current_pitch, &pitch_correction, &target_pitch, pitchKp, pitchKi, pitchKd, DIRECT);
PID yawPID(&YawInput, &yaw_correction, &target_yaw, yawKp, yawKi, yawKd, DIRECT);

void init_PID()
{
    // Initialize PID controllers
    rollPID.SetMode(AUTOMATIC);
    rollPID.SetOutputLimits(-SERVOMIN, SERVOMAX); // Limites de sortie du PID pour correspondre à la position du servomoteur
    pitchPID.SetMode(AUTOMATIC);
    pitchPID.SetOutputLimits(-SERVOMIN, SERVOMAX); // Limites de sortie du PID pour correspondre à la position du servomoteur
    yawPID.SetMode(AUTOMATIC);
    yawPID.SetOutputLimits(-SERVOMIN, SERVOMAX); // Limites de sortie du PID pour correspondre à la position du servomoteur
};

void stabilize(
    bool roll_block_condition,
    float roll_correction,
    bool roll_step_condition,
    float roll_step_value,
    bool pitch_block_condition,
    float pitch_correction,
    bool pitch_step_condition,
    float pitch_step_value,
    bool yaw_block_condition,
    float yaw_correction,
    bool yaw_step_condition,
    float yaw_step_value)
{
    update_PID();
    compute_PID();

    if (roll_block_condition)
    {
        roll_correction = SERVOMID;
    }

    if (pitch_block_condition)
    {
        pitch_correction = SERVOMID;
    }

    if (yaw_block_condition)
    {
        yaw_correction = SERVOMID;
    }

    if (roll_block_condition || pitch_step_condition)
    {
        // RECOVERY :
        if (abs(current_roll - prev_roll_correction) > roll_tolerance || abs(current_pitch - prev_pitch_correction > pitch_tolerance))
        {
            isTurning = false; // Situation is too dangerous, we quit the current turn
            isCorrectingAltitude = false;
            currentNavigationMode = STABILIZING;
        }

        else
        { // CORRECTION WITH STEP :
            if (roll_step_condition && (current_roll - prev_roll_correction) < 1)
            {
                // Si l'inclinaison précédente a été atteinte, on met à jour l'inclinaison demandée
                prev_roll_correction = updateRollAngleRequest(prev_roll_correction, target_roll);
            }

            else if (!roll_step_condition)
            {
            }

            if (pitch_step_condition && (current_roll - prev_roll_correction) < 1)
            {
                // Si l'inclinaison précédente a été atteinte, on met à jour l'inclinaison demandée
                prev_roll_correction = updatePitchAngleRequest(prev_pitch_correction, target_pitch);
            }
        }
    }
    else
    {
        prev_roll_correction = 0;
        prev_pitch_correction = 0;
        prev_yaw_correction = 0;
    }

    if (currentTurnMode = YAW_CORRECTION)
    {
        yaw_correction = SERVOMID + yaw_correction;
    }
    set_servos(roll_correction, pitch_correction, yaw_correction);
};

unsigned long stabilizationStartTime = 0;
bool wasStabilizing = false;
bool stabilized = false;
bool stabilizing = false;

void check_stability()
{
    if (abs(current_roll) >= (target_roll - STABILIZATION_TOLERANCE) && abs(current_roll) <= (target_roll + STABILIZATION_TOLERANCE) && abs(current_pitch) >= (target_pitch - STABILIZATION_TOLERANCE) && abs(current_pitch) <= (target_pitch + STABILIZATION_TOLERANCE))

    {
        stabilizing = true;
    }
    else
    {
        stabilizing = false;
    }

    // Vérifier si la condition est restée vraie pendant 3 secondes
    if (stabilizing)
    {
        if (!wasStabilizing)
        {
            // Si c'est la première fois que isStabilising devient vrai, démarrer le chronomètre
            stabilizationStartTime = millis();
            wasStabilizing = true;
        }
        else
        {
            // Si isStabilising était déjà vrai, vérifier le temps écoulé
            if (millis() - stabilizationStartTime >= STABILIZATION_PERIOD)
            {
                stabilized = true;
            }
        }
    }
    else
    {
        // Réinitialiser si isStabilising devient faux
        wasStabilizing = false;
        stabilized = false;
        // Keep stabilizing
    }
}

void update_PID() // UPDATE PID ACCORDING TO THE CURRENT MODE
{

    switch (currentNavigationMode)
    {
    case TAKEOFF:
        rollKp = PID_TAKEOFF[0];
        rollKi = PID_TAKEOFF[1];
        rollKd = PID_TAKEOFF[2];
        break;

    case NAVIGATE:
        rollKp = PID_NAVIGATE[0];
        rollKi = PID_NAVIGATE[1];
        rollKd = PID_NAVIGATE[2];
        break;

    case LANDING:
        rollKp = PID_LANDING[0];
        rollKi = PID_LANDING[1];
        rollKd = PID_LANDING[2];
        break;

    default:
        Serial.println("Erreur dans les modes de PID");
        break;
    }

    // Mise à jour des PID. A placer dans le loop si MAJ en cours de vol
    rollPID.SetTunings(rollKp, rollKi, rollKd);
    pitchPID.SetTunings(pitchKp, pitchKi, pitchKd);
    yawPID.SetTunings(yawKp, yawKi, yawKd);
};

void compute_PID()
{
    RollInput = current_roll;
    PitchInput = current_pitch;
    YawInput = current_heading;

    RollSetpoint = target_roll;
    PitchSetpoint = target_pitch;
    YawSetpoint = target_yaw;

    rollPID.Compute();
    pitchPID.Compute();
    yawPID.Compute();
}

void adjustRollForHeading()
{
}

// Fonction pour mettre à jour la demande d'inclinaison degré par degré
double updateRollAngleRequest(double prev_roll_correction, double target_roll)
{
    if (target_roll > prev_roll_correction)
    {
        return prev_roll_correction + roll_step_value; // Incliner d'un degré vers le haut
    }
    else if (target_roll < prev_roll_correction)
    {
        return prev_roll_correction - roll_step_value; // Incliner d'un degré vers le bas
    }
    else
    {
        return prev_roll_correction; // Pas de changement
    }
}

// Fonction pour mettre à jour la demande d'inclinaison degré par degré
double updatePitchAngleRequest(double prev_pitch_correction, double target_pitch)
{
    if (target_pitch > prev_pitch_correction)
    {
        return prev_pitch_correction + pitch_step_value; // Incliner d'un degré vers le haut
    }
    else if (target_pitch < prev_pitch_correction)
    {
        return target_pitch - pitch_step_value; // Incliner d'un degré vers le bas
    }
    else
    {
        return prev_pitch_correction; // Pas de changement
    }
}

/////////////// ALTITUDE ///////////////

void calculate_altitude_factor()
{
    StartAltitudeError = target_altitude - current_barometer_altitude;
    StartWaypointDistance = waypoint_distance;
}

double adjustPitchForAltitude(float current_barometer_altitude, float target_altitude)
{
    double adjustedPitchforAltitude = 0;
    altitudeError = target_altitude - current_barometer_altitude;

    K_altitude = (altitude_error * waypoint_distance) / (StartAltitudeError * StartWaypointDistance);

    // Calculer la correction du pitch proportionnellement à l'erreur d'altitude
    adjustedPitchforAltitude = K_altitude * altitudeError;

    // Limiter l'angle de pitch pour ne pas dépasser max_pitch_angle
    if (adjustedPitchforAltitude > max_pitch_angle)
    {
        adjustedPitchforAltitude = max_pitch_angle;
    }
    else if (adjustedPitchforAltitude < -max_pitch_angle)
    {
        adjustedPitchforAltitude = -max_pitch_angle;
    }

    return adjustedPitchforAltitude;
    /*
        if (distanceToTarget < endTransitionDistance)
        {
            // Réduction progressive de la correction à l'approche de la cible
            transitionFactor = waypoint_distance / endTransitionDistance;
        }
        else if (waypoint_distance < startTransitionDistance)
        {
            // Correction progressive au début de la montée ou descente
            transitionFactor = 1.0 - ((distanceToTarget - endTransitionDistance) / (startTransitionDistance - endTransitionDistance));
        }
        else
        {
            transitionFactor = 1.0;
        }

        pitchCorrection *= transitionFactor;

        // Ajuster la correction du pitch en fonction de la direction du mouvement
        float servoPosition;
        if (altitudeError > 0)
        { // Si l'avion doit monter
            servoPosition = servoMid + pitchCorrection * (servoMax - servoMid);
        }
        else
        { // Si l'avion doit descendre
            servoPosition = servoMid + pitchCorrection * (servoMid - servoMin);
        }

        // Conserver la position du servo dans les limites
        servoPosition = constrain(servoPosition, servoMin, servoMax);

        applyPitchCorrection(servoPosition);

        */
}
