/**
 * Authored by Michael Hamer (http://www.mikehamer.info), November 2016.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include "FreeRTOS.h"
#include "task.h"
#include "cfmath.h"

#include "motor_calibration.h"

#include "motors.h"
#include "sensors.h"
#include "crtp.h"
#include "log.h"
#include "physical_constants.h"
#include "param.h"
#include "num.h"

#define MAX_COVARIANCE (100)
#define MIN_COVARIANCE (1e-6f)

static float tau_rp = 0.5f;
static float tau_rp_rate = 0.01f;

static float motor_pwm[4] = {0};
static float ki = 0.5e-3f;
static float angleErrorIntegrator = 1;
static float angleError = 0;
static float currentAngle = 0;

static float desiredAngle = 15.0f;

static bool isInit = false;
static uint32_t initTicks = 0;
static uint32_t lastTicks = 0;

static float Q[4] = {0};
static arm_matrix_instance_f32 Qm = {4, 1, Q};

static float S[3] = {0};
static arm_matrix_instance_f32 Sm = {3, 1, S};

static float P[3][3] = {0};
static arm_matrix_instance_f32 Pm = {3, 3, (float*)P};

void motorCalibrationInit() {
    initTicks = xTaskGetTickCount();

    Q[0] = 1;
    Q[1] = Q[2] = Q[3] = 0;
    
    S[0] = S[1] = S[2] = 0;
    
    for (int i=0; i<3; i++) {
      for (int j=0; j<3; j++) {
       P[i][j] = 0; 
      }
    }

    P[0][0] = P[1][1] = P[2][2] = MAX_COVARIANCE;

    currentAngle = 0;
    // angleErrorIntegrator = 1;

    isInit = true;
}

void motorCalibrationEstimateBodyAngle(sensorData_t *sensors) __attribute__((optimize("O2")));

void motorCalibrationEstimateBodyAngle(sensorData_t *sensors)
{
  // process (gyroscope) update
  //float dt = (xTaskGetTickCount() - lastTicks)/configTICK_RATE_HZ;

  // // linearized dynamics for covariance update
  // float A[3][3];
  // arm_matrix_instance_f32 Am = { 3, 3, (float *)A}; 

  // // PREDICTION STEP - STATE UPDATE

  // // this is the gyroscope angular velocity integrated over the sample period
  // float dtwx = dt*sensors->gyro.x * DEG_TO_RAD;
  // float dtwy = dt*sensors->gyro.y * DEG_TO_RAD;
  // float dtwz = dt*sensors->gyro.z * DEG_TO_RAD;

  // // compute the quaternion values in [w,x,y,z] order
  // float angle = arm_sqrt(dtwx*dtwx + dtwy*dtwy + dtwz*dtwz);
  // float ca = cosf(angle/2.0f);
  // float sa = sinf(angle/2.0f);
  // float dQ[4] = {ca , sa*dtwx/angle , sa*dtwy/angle , sa*dtwz/angle};

  // // rotate the quad's attitude by the delta quaternion vector computed above
  // Q[0] = (dQ[0]*Q[0] - dQ[1]*Q[1] - dQ[2]*Q[2] - dQ[3]*Q[3]);
  // Q[1] = (dQ[1]*Q[0] + dQ[0]*Q[1] + dQ[3]*Q[2] - dQ[2]*Q[3]);
  // Q[2] = (dQ[2]*Q[0] - dQ[3]*Q[1] + dQ[0]*Q[2] + dQ[1]*Q[3]);
  // Q[3] = 0.999*(dQ[3]*Q[0] + dQ[2]*Q[1] - dQ[1]*Q[2] + dQ[0]*Q[3]);
  // quaternion_normalize(&Qm);

  // // PREDICTION STEP - COVARIANCE UPDATE

  // float d0 = sensors->gyro.x*dt/2;
  // float d1 = sensors->gyro.y*dt/2;
  // float d2 = sensors->gyro.z*dt/2;

  // A[0][0] =  1 - d1*d1/2 - d2*d2/2;
  // A[0][1] =  d2 + d0*d1/2;
  // A[0][2] = -d1 + d0*d2/2;

  // A[1][0] = -d2 + d0*d1/2;
  // A[1][1] =  1 - d0*d0/2 - d2*d2/2;
  // A[1][2] =  d0 + d1*d2/2;

  // A[2][0] =  d1 + d0*d2/2;
  // A[2][1] = -d0 + d1*d2/2;
  // A[2][2] = 1 - d0*d0/2 - d1*d1/2;

  // float AT[3][3]; arm_matrix_instance_f32 ATm = { 3, 3, (float*)AT};
  // mat_trans(&Am, &ATm); // A'

  // float AP[3][3]; arm_matrix_instance_f32 APm = { 3, 3, (float*)AP};
  // mat_mult(&Am, &Pm, &APm); // A P

  // mat_mult(&APm, &ATm, &Pm); // P = A P A'

  // P[0][0] += powf(measNoiseGyro_rollpitch * dt, 2);
  // P[1][1] += powf(measNoiseGyro_rollpitch * dt, 2);
  // P[2][2] += powf(measNoiseGyro_yaw * dt, 2); // P = A P A' + Q

  // for (int i=0; i<3; i++) {
  //   for (int j=i; j<3; j++) {
  //     float p = 0.5f*P[i][j] + 0.5f*P[j][i]; // add measurement noise
  //     if (isnan(p) || p > MAX_COVARIANCE) {
  //       P[i][j] = P[j][i] = MAX_COVARIANCE;
  //     } else if ( i==j && p < MIN_COVARIANCE ) {
  //       P[i][j] = P[j][i] = MIN_COVARIANCE;
  //     } else {
  //       P[i][j] = P[j][i] = p;
  //     }
  //   }
  // }

  // // MEASUREMENT UPDATE - GAIN CALCULATION
  
  // // we have measurements of the bank column of the rotation matrix
  // float H[3][3] = {{2*Q[0]*Q[3]-2*Q[1]*Q[2], Q[0]*Q[0]+Q[1]*Q[1]-Q[2]*Q[2]-Q[3]*Q[3], 0},
  //                  {-Q[0]*Q[0]+Q[1]*Q[1]-Q[2]*Q[2]+Q[3]*Q[3], 2*Q[0]*Q[3]+2*Q[1]*Q[2], 0},
  //                  {-2*Q[0]*Q[1]-2*Q[2]*Q[3], -2*Q[0]*Q[2]+2*Q[1]*Q[3], 0}};
  // arm_matrix_instance_f32 Hm = { 3, 3, (float*)H };

  // float HT[3][3]; arm_matrix_instance_f32 HTm = { 3, 3, (float*)HT };
  // float PHT[3][3]; arm_matrix_instance_f32 PHTm = { 3, 3, (float*)PHT };
  // float HPHT[3][3]; arm_matrix_instance_f32 HPHTm = { 3, 3, (float*)HPHT };
  // float HPHTI[3][3]; arm_matrix_instance_f32 HPHTIm = { 3, 3, (float*)HPHTI };
  // float K[3][3]; arm_matrix_instance_f32 Km = { 3, 3, (float*)K };
  // float KT[3][3]; arm_matrix_instance_f32 KTm = { 3, 3, (float*)KT };

  // mat_trans(&Hm, &HTm); // HT
  // mat_mult(&Pm, &HTm, &PHTm); // P H'
  // mat_mult(&Hm, &PHTm, &HPHTm); // H P H'
  // HPHT[0][0] += procNoiseAcc_xy;
  // HPHT[1][1] += procNoiseAcc_xy;
  // HPHT[2][2] += procNoiseAcc_z; // (H P H' + R)
  // mat_inv(&HPHTm, &HPHTIm); // (H P H' + R)^-1
  // mat_mult(&PHTm, &HPHTIm, &Km); // K = P H'(H P H' + R)^-1
  // mat_trans(&Km, &KTm);

  // // MEASUREMENT UPDATE - STATE UPDATE
  // float Z[3] = {sensors->acc.x, sensors->acc.y, sensors->acc.z};
  // arm_matrix_instance_f32 Zm = { 3, 1, (float*)Z };

  // // (z - h(x))
  // Z[0] -= 2 * Q[1] * Q[3] + 2 * Q[0] * Q[2];
  // Z[1] -= 2 * Q[2] * Q[3] - 2 * Q[0] * Q[1];
  // Z[2] -= Q[0] * Q[0] - Q[1] * Q[1] - Q[2] * Q[2] + Q[3] * Q[3];
  
  // float KZ[3]; arm_matrix_instance_f32 KZm = { 3, 1, KZ };
  // mat_mult(&Km, &Zm, &KZm); // K (z-h(x))

  // // x <- x + K (z-h(x))
  // S[0] += KZ[0];
  // S[1] += KZ[1];
  // S[2] += KZ[2];

  // // MEASUREMENT UPDATE - COVARIANCE UPDATE

  // float KH[3][3]; arm_matrix_instance_f32 KHm = {3, 3, (float*)KH};
  // mat_mult(&Km, &Hm, &KHm);
  // KH[0][0] -= 1;
  // KH[1][1] -= 1;
  // KH[2][2] -= 1;

  // float KHT[3][3]; arm_matrix_instance_f32 KHTm = {3, 3, (float*)KHT};
  // mat_trans(&KHm, &KHTm);

  // float KHP[3][3]; arm_matrix_instance_f32 KHPm = {3, 3, (float*)KHP};
  // mat_mult(&KHm, &Pm, &KHPm);

  // float KHPKHT[3][3]; arm_matrix_instance_f32 KHPKHTm = {3, 3, (float*)KHPKHT};
  // mat_mult(&KHPm, &KHTm, &KHPKHTm);

  // float R[3][3] = {{procNoiseAcc_xy, 0, 0}, {0, procNoiseAcc_xy, 0}, {0, 0, procNoiseAcc_z}};
  // arm_matrix_instance_f32 Rm = {3, 3, (float*)R};

  // float KR[3][3]; arm_matrix_instance_f32 KRm = {3, 3, (float*)KR};
  // mat_mult(&Km, &Rm, &KRm);
  // mat_mult(&KRm, &KTm, &Rm);

  // for(int i=0; i<3; i++) {
  //   for(int j=0; j<3; j++) {
  //     P[i][j] = KHPKHT[i][j] + R[i][j];
  //   }
  // }

  // // FINALIZATION

  // float v0 = S[0];
  // float v1 = S[1];
  // float v2 = S[2];

  // // compute the quaternion values in [w,x,y,z] order
  // angle = arm_sqrt(v0*v0 + v1*v1 + v2*v2);
  // ca = arm_cos_f32(angle/2.0f);
  // sa = arm_sin_f32(angle/2.0f);
  // dQ[0] = ca; dQ[1] = sa*v0/angle; dQ[2] = sa*v1/angle; dQ[3] = sa*v2/angle;

  // // rotate the quad's attitude by the delta quaternion vector computed above
  // Q[0] = (dQ[0]*Q[0] - dQ[1]*Q[1] - dQ[2]*Q[2] - dQ[3]*Q[3]);
  // Q[1] = (dQ[1]*Q[0] + dQ[0]*Q[1] + dQ[3]*Q[2] - dQ[2]*Q[3]);
  // Q[2] = (dQ[2]*Q[0] - dQ[3]*Q[1] + dQ[0]*Q[2] + dQ[1]*Q[3]);
  // Q[3] = 0.999*(dQ[3]*Q[0] + dQ[2]*Q[1] - dQ[1]*Q[2] + dQ[0]*Q[3]);
  // quaternion_normalize(&Qm);

  // S[0]=S[1]=S[2] = 0;

  // // rotate the covariance
  // d0 = v0/2; // the attitude error vector (v0,v1,v2) is small,
  // d1 = v1/2; // so we use a first order approximation to d0 = tan(|v0|/2)*v0/|v0|
  // d2 = v2/2;

  // A[0][0] =  1 - d1*d1/2 - d2*d2/2;
  // A[0][1] =  d2 + d0*d1/2;
  // A[0][2] = -d1 + d0*d2/2;

  // A[1][0] = -d2 + d0*d1/2;
  // A[1][1] =  1 - d0*d0/2 - d2*d2/2;
  // A[1][2] =  d0 + d1*d2/2;

  // A[2][0] =  d1 + d0*d2/2;
  // A[2][1] = -d0 + d1*d2/2;
  // A[2][2] = 1 - d0*d0/2 - d1*d1/2;

  // mat_trans(&Am, &ATm); // A'
  // mat_mult(&Am, &Pm, &APm); // AP
  // mat_mult(&APm, &Am, &Pm); //APA'

  for (int i=0; i<3; i++) {
    for (int j=i; j<3; j++) {
      float p = 0.5f*P[i][j] + 0.5f*P[j][i]; // add measurement noise
      if (isnan(p) || p > MAX_COVARIANCE) {
        P[i][j] = P[j][i] = MAX_COVARIANCE;
      } else if ( i==j && p < MIN_COVARIANCE ) {
        P[i][j] = P[j][i] = MIN_COVARIANCE;
      } else {
        P[i][j] = P[j][i] = p;
      }
    }
  }

  lastTicks = xTaskGetTickCount();
}

bool motorCalibrationRun(control_t *control, sensorData_t *sensors)
{
  if (!isInit) {
    motorCalibrationInit();
  }

  if (xTaskGetTickCount() - initTicks > M2T(10000)) {
    isInit = false;
    return false;
  }

  sensorsReadAcc(&sensors->acc);
  sensorsReadGyro(&sensors->gyro);

  float dt = (float)(xTaskGetTickCount() - lastTicks)/(float)configTICK_RATE_HZ;

  // a unit vector pointing in the direction of the desired body
  float zI_des[3] = {0, sinf(radians(desiredAngle)), cosf(radians(desiredAngle))};
  arm_matrix_instance_f32 zI_des_m = {3, 1, zI_des};
  vec_normalize(&zI_des_m);

  // a unit vector pointing in the direction of the current body
  float zI_cur[3];
  arm_matrix_instance_f32 zI_cur_m = {3, 1, zI_cur};
  zI_cur[0] = sensors->acc.x * GRAVITY;
  zI_cur[1] = sensors->acc.y * GRAVITY;
  zI_cur[2] = sensors->acc.z * GRAVITY;
  vec_normalize(&zI_cur_m);

  currentAngle += dt*radians(sensors->gyro.x);
  currentAngle = 0.99989f*currentAngle + 0.0001f*asinf(zI_cur[1]);

  angleError = radians(desiredAngle) - currentAngle;

  if (xTaskGetTickCount() - initTicks < M2T(500)) {
    motorsSetRatio(0, 0);
    motorsSetRatio(1, 0);
    motorsSetRatio(2, 5000);
    motorsSetRatio(3, 5000);
    lastTicks = xTaskGetTickCount();
    return true;
  }

  if (fabsf(currentAngle) > radians(75.0f)) {
    motor_pwm[0] = 0;
    motor_pwm[1] = 0;
    motor_pwm[2] = 0;
    motor_pwm[3] = 0;
    for (int i=0; i<4; i++) {
      motorsSetRatio(i, 0);
    }
    return false;
  }

  // the current body rates  
  float omega[3] = {0};
  omega[0] = radians(sensors->gyro.x);
  omega[1] = radians(sensors->gyro.y);
  omega[2] = radians(sensors->gyro.z);

  float omegaDes = 2.0f / tau_rp * angleError;
  float omegaErr = (omegaDes - omega[0])/tau_rp_rate;

  if (xTaskGetTickCount() - initTicks > M2T(3000)) {
    angleErrorIntegrator += ki*angleError;
  }

  float torqueDes = CRAZYFLIE_INERTIA[0][0] * omegaErr + ((CRAZYFLIE_ARM_LENGTH/sqrtf(2.0f)-CRAZYFLIE_COG_HEIGHT*tanf(currentAngle))*cosf(currentAngle))*(CRAZYFLIE_MASS*GRAVITY) * min(1.0, (float)(xTaskGetTickCount() - initTicks)/(float)M2T(2000));
  float forceDes = torqueDes/(2.0f*CRAZYFLIE_ARM_LENGTH/sqrtf(2.0f));
  float motor_forces[4];

  motor_forces[0] = 0;
  motor_forces[1] = 0;
  motor_forces[2] = angleErrorIntegrator * forceDes/2.0f;
  motor_forces[3] = angleErrorIntegrator * forceDes/2.0f;

  for (int i=0; i<4; i++) {
    if (motor_forces[i] <= 0) {
      motor_forces[i] = 0;
      motor_pwm[i] = 0;
    } else {
      motor_pwm[i] = (-PWM_TO_THRUST_b + arm_sqrt(PWM_TO_THRUST_b*PWM_TO_THRUST_b + 4.0f * PWM_TO_THRUST_a * motor_forces[i])) /
                     (2.0f * PWM_TO_THRUST_a);
    }
    
    motor_pwm[i] = constrain(motor_pwm[i], 0, 1);
    motorsSetRatio(i, (uint16_t)(65535*motor_pwm[i]));
  }

  lastTicks = xTaskGetTickCount();

  return true;
}

PARAM_GROUP_START(a)
PARAM_ADD(PARAM_FLOAT, ki, &ki)
PARAM_ADD(PARAM_FLOAT, trp, &tau_rp)
PARAM_ADD(PARAM_FLOAT, trp_dot, &tau_rp_rate)
PARAM_ADD(PARAM_FLOAT, angle, &desiredAngle)
PARAM_GROUP_STOP(a)

LOG_GROUP_START(motorcalib)
LOG_ADD(LOG_FLOAT, m1, &motor_pwm[0])
LOG_ADD(LOG_FLOAT, m2, &motor_pwm[1])
LOG_ADD(LOG_FLOAT, m3, &motor_pwm[2])
LOG_ADD(LOG_FLOAT, m4, &motor_pwm[3])
LOG_ADD(LOG_FLOAT, aErr, &angleError)
LOG_ADD(LOG_FLOAT, aCur, &currentAngle)
LOG_ADD(LOG_FLOAT, aInt, &angleErrorIntegrator)
LOG_GROUP_STOP(motorcalib)