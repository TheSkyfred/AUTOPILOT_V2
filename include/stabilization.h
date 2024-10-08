
#ifndef STABILIZATON_H_
#define STABILIZATON_H_


#include "def.h"

#include <PID_v1.h>
#include <SimpleKalmanFilter.h>



//Functions :
void init_PID();
void update_PID();
void compute_PID();

void update_stabilization();

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
    float yaw_step_value);


void check_stability();

void calculate_altitude_factor();
double adjustPitchForAltitude(float current_barometer_altitude, float target_altitude);

double updateRollAngleRequest(double prev_roll_correction, double target_roll);
double updatePitchAngleRequest(double prev_pitch_correction, double target_pitch);


#endif /* STABILIZATON_H_ */


