#pragma once

#include <stdbool.h>
#include "cfmath.h"
#include "imu_types.h"

#define GRAVITY (9.81f)
#define SPEED_OF_LIGHT (299792458)

// constants based on Julian Förster's System Identification of a Crazyflie

extern float CRAZYFLIE_ARM_LENGTH; // arm length from CF center to motor center
extern float CRAZYFLIE_COG_HEIGHT; // height of CF CenterOfGravity from floor
extern float CRAZYFLIE_MASS;

extern float CRAZYFLIE_INERTIA[3][3];
extern arm_matrix_instance_f32 CRAZYFLIE_INERTIA_m;

// thrust force = a * pwm^2 + b * pwm
extern float PWM_TO_THRUST_a;
extern float PWM_TO_THRUST_b;

// motor torque = m * thrust force
extern float THRUST_TO_TORQUE_m;

// noises
extern float NOISE_GYRO_ROLLPITCH;
extern float NOISE_GYRO_YAW;
extern float NOISE_ACC_XY;
extern float NOISE_ACC_Z;

// biases to be calibrated
extern float CALIBRATION_DURATION;
extern float CALIBRATION_IMUCONSTANT;
extern float CALIBRATION_MOTORCONSTANT;
extern Axis3f GYRO_BIAS;
extern Axis3f GYRO_VARIANCE;
extern Axis3f ACC_BIAS;
extern Axis3f ACC_VARIANCE;
extern float ACC_SCALE;
extern float MOTOR_SCALE[4];
extern float OMEGA_BIAS[3];
extern float THRUST_BIAS;

// physical states
extern bool IS_CALIBRATING;
extern bool FINISHED_CALIBRATING;
extern bool IS_INFLIGHT;

