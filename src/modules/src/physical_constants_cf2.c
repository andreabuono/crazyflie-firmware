#include "physical_constants.h"

#include "param.h"
#include "log.h"

// constants based on Julian Förster's System Identification of a Crazyflie

float CRAZYFLIE_INERTIA[3][3] =
    {{16.6e-6f, 0.83e-6f, 0.72e-6f},
     {0.83e-6f, 16.6e-6f, 1.8e-6f},
     {0.72e-6f, 1.8e-6f, 29.3e-6f}};

arm_matrix_instance_f32 CRAZYFLIE_INERTIA_m = {3, 3, (float*)CRAZYFLIE_INERTIA};

// thrust = a * pwm^2 + b * pwm
float PWM_TO_THRUST_a = .091492681f;
float PWM_TO_THRUST_b = .067673604f;

// motor torque = m * thrust
float THRUST_TO_TORQUE_m = 0.005964552f;

// constants
float CRAZYFLIE_ARM_LENGTH = 0.046f; // m
float CRAZYFLIE_COG_HEIGHT = 0.02f; // m
float CRAZYFLIE_MASS = 34e-3f; // kg

// noises
 float NOISE_GYRO_ROLLPITCH = 0.1f; // radians per second
 float NOISE_GYRO_YAW = 0.25f; // radians per second
 float NOISE_ACC_XY = 0.5f; // m/s^2
 float NOISE_ACC_Z = 1.5f; // m/s^2

// biases to be calibrated
float CALIBRATION_DURATION = 5;
float CALIBRATION_IMUCONSTANT = 10;
float CALIBRATION_MOTORCONSTANT = 5;

Axis3f GYRO_BIAS = {0};
Axis3f GYRO_VARIANCE = {0};

Axis3f ACC_BIAS = {0};
Axis3f ACC_VARIANCE = {0};

float ACC_SCALE = 1;
float MOTOR_SCALE[4] = {1, 1, 1, 1};
float OMEGA_BIAS[3] = {0};
float THRUST_BIAS = 0;

// physical states
bool IS_INFLIGHT = false;
bool IS_CALIBRATING = false;
bool FINISHED_CALIBRATING = false;

LOG_GROUP_START(physical)
LOG_ADD(LOG_UINT8, isInFlight, &IS_INFLIGHT)
LOG_ADD(LOG_FLOAT, gyroBiasX, &GYRO_BIAS.x)
LOG_ADD(LOG_FLOAT, gyroBiasY, &GYRO_BIAS.y)
LOG_ADD(LOG_FLOAT, gyroBiasZ, &GYRO_BIAS.z)
LOG_ADD(LOG_FLOAT, gyroVarX, &GYRO_VARIANCE.x)
LOG_ADD(LOG_FLOAT, gyroVarY, &GYRO_VARIANCE.y)
LOG_ADD(LOG_FLOAT, gyroVarZ, &GYRO_VARIANCE.z)
LOG_ADD(LOG_FLOAT, accBiasX, &ACC_BIAS.x)
LOG_ADD(LOG_FLOAT, accBiasY, &ACC_BIAS.y)
LOG_ADD(LOG_FLOAT, accBiasZ, &ACC_BIAS.z)
LOG_ADD(LOG_FLOAT, accVarX, &ACC_VARIANCE.x)
LOG_ADD(LOG_FLOAT, accVarY, &ACC_VARIANCE.y)
LOG_ADD(LOG_FLOAT, accVarZ, &ACC_VARIANCE.z)
LOG_ADD(LOG_FLOAT, accScale, &ACC_SCALE)
LOG_ADD(LOG_FLOAT, motorScale1, &MOTOR_SCALE[0])
LOG_ADD(LOG_FLOAT, motorScale2, &MOTOR_SCALE[1])
LOG_ADD(LOG_FLOAT, motorScale3, &MOTOR_SCALE[2])
LOG_ADD(LOG_FLOAT, motorScale4, &MOTOR_SCALE[3])
LOG_GROUP_STOP(physical)

PARAM_GROUP_START(physical)
PARAM_ADD(PARAM_UINT8, calibTime, &CALIBRATION_DURATION)
PARAM_ADD(PARAM_UINT8, calibrate, &IS_CALIBRATING)
PARAM_ADD(PARAM_FLOAT, mass, &CRAZYFLIE_MASS)
PARAM_ADD(PARAM_FLOAT, noiseGyroRP, &NOISE_GYRO_ROLLPITCH)
PARAM_ADD(PARAM_FLOAT, noiseGyroYAW, &NOISE_GYRO_YAW)
PARAM_ADD(PARAM_FLOAT, noiseAccXY, &NOISE_ACC_XY)
PARAM_ADD(PARAM_FLOAT, noiseAccZ, &NOISE_ACC_Z)
PARAM_GROUP_STOP(physical)