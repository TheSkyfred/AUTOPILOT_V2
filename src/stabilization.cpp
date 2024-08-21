#include "stabilization.h"

#include "def.h"
#include "config.h"

#include "modes.h"

#include "gyroscope.h"
#include "config.h"
#include "GPS.h"

// PID
//  Définir les paramètres de réglage du PID pour le roulis et le tangage
double rollKp = 1.03, rollKi = 0.000, rollKd = 2.0003;
double pitchKp = 1.02, pitchKi = 0.000, pitchKd = 2.0002;
double yawKp = 1.025, yawKi = 0.000, yawKd = 0.0002;

// KALMANFILTER
extern SimpleKalmanFilter xKalmanFilter;
extern SimpleKalmanFilter yKalmanFilter;
extern SimpleKalmanFilter zKalmanFilter;

// Variables pour l'orientation actuelle basée sur l'IMU
int target_roll = 0;
int target_pitch = 0;
int target_yaw = 0;

int prev_roll_correction = 0;
int prev_pitch_correction = 0;
int prev_yaw_correction = 0;

// Intput & Output
int acc_roll_input, acc_pitch_input;
int gyro_pitch_input, gyro_roll_input;
int pitch_output, roll_output;

// Corrections
float roll_correction = 0;
float pitch_correction = 0;
float yaw_correction = 0;

bool roll_block_condition = 0;
bool roll_step_condition = 0;
float roll_step_value = 0;
bool pitch_block_condition = 0;
bool pitch_step_condition = 0;
float pitch_step_value = 0;
bool yaw_block_condition = 0;
bool yaw_step_condition = 0;
float yaw_step_value = 0;

// Définir des variables pour le PID
double RollSetpoint, RollInput, RollOutput;
double PitchSetpoint, PitchInput, PitchOutput;
double YawSetpoint, YawInput, YawOutput;

// Objets PID avec entrées et sorties en tant que pointeurs
PID rollPID(&RollInput, &RollOutput, &RollSetpoint, rollKp, rollKi, rollKd, DIRECT);
PID pitchPID(&PitchInput, &PitchOutput, &PitchSetpoint, pitchKp, pitchKi, pitchKd, DIRECT);
PID yawPID(&YawInput, &YawOutput, &YawSetpoint, yawKp, yawKi, yawKd, DIRECT);

bool is_stable = false;

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

    if (roll_step_condition)
    {
        if (current_roll == prev_roll_correction)
        {
            roll_correction += roll_step_value;
        }

        else
        {
            roll_correction = prev_roll_correction;
        }
    }

    if (pitch_step_condition)
    {
        if (current_pitch == prev_pitch_correction)
        {

            pitch_correction += pitch_step_value;
        }

        else
        {
            pitch_correction = prev_pitch_correction;
        }
    }

    if (yaw_step_condition)
    {
        if (current_yaw == prev_yaw_correction)
        {

            yaw_correction += yaw_step_value;
        }

        else
        {
            yaw_correction = prev_yaw_correction;
        }
    }
};

unsigned long stabilizationStartTime = 0;
bool wasStabilizing = false;
bool stabilized = false;
bool stabilizing = false;

void check_stability()
{
    if (abs(current_roll) <= STABILIZATION_TOLERANCE && abs(current_pitch) <= STABILIZATION_TOLERANCE)
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


void update_PID()
{
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

    PitchSetpoint = current_pitch;

    RollSetpoint = target_roll;
    PitchSetpoint = target_pitch;
    YawSetpoint = target_yaw;

    rollPID.Compute();
    pitchPID.Compute();
    yawPID.Compute();
}