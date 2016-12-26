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
 *
 * ============================================================================
 *
 * The controller implemented in this file is based on the paper:
 *
 * "Nonlinear Quadrocopter Attitude Control"
 * http://e-collection.library.ethz.ch/eserv/eth:7387/eth-7387-01.pdf
 *
 * and
 *
 * "Kalman filtering with an attitude" as published in the PhD thesis "Increased autonomy for quadrocopter systems: trajectory generation, fail-safe strategies, and state estimation"
 * http://dx.doi.org/10.3929/ethz-a-010655275
 * TODO: Update the above reference once the paper has been published
 *
 * Academic citation would be appreciated.
 *
 * BIBTEX ENTRIES:
      @ARTICLE{BrescianiniNonlinearController2013,
               title={Nonlinear quadrocopter attitude control},
               author={Brescianini, Dario and Hehn, Markus and D'Andrea, Raffaello},
               year={2013},
               publisher={ETH Zurich}}
 *
 * ============================================================================
 */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "arm_math.h"

#include "controller_new.h"
#include "crtp.h"
#include "log.h"
#include "param.h"
#include "num.h"


#ifdef ESTIMATOR_TYPE_kalman
#include "estimator_kalman.h"
#endif

#define GRAVITY (9.81f)
#define DEG_TO_RAD (((float)M_PI)/180.0f)
#define ARCMINUTE (((float)M_PI)/10800.0f)

// TODO: move this into a physical constants file
static float INERTIA[3][3] =
    {{16.6e-6f, 0.83e-6f, 0.72e-6f},
     {0.83e-6f, 16.6e-6f, 1.8e-6f},
     {0.72e-6f, 1.8e-6f, 29.3e-6f}}; // from Julian Förster's System Identification of a Crazyflie
static arm_matrix_instance_f32 INERTIA_m = {3, 3, (float*)INERTIA};

static float thrust_reduction_fairness = 0.0; // 1 -> even reduction across x, y, z, 0 -> z gets what it wants
static float mixing_factor = 1.0;

static float tau_xy = 0.3;
static float zeta_xy = 0.65;

static float tau_z = 0.2;
static float zeta_z = 0.55;

static float tau_rp = 0.25;

static float tau_rp_rate = 0.015;
static float tau_yaw_rate = 0.0075;

static float coll_min = 5;
static float coll_max = 15;

static float omega_rp_max = 30;
static float omega_yaw_max = 30;

static float igain_yaw = 0.01f;
static float imax_yaw = 1.0f;
static float igain_rate = 0.001f;



static inline void mat_trans(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_trans_f32(pSrc, pDst)); }
static inline void mat_inv(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_inverse_f32(pSrc, pDst)); }
static inline void mat_mult(const arm_matrix_instance_f32 * pSrcA, const arm_matrix_instance_f32 * pSrcB, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_mult_f32(pSrcA, pSrcB, pDst)); }
static inline float arm_sqrt(float32_t in)
{ float pOut = 0; arm_status result = arm_sqrt_f32(in, &pOut); configASSERT(ARM_MATH_SUCCESS == result); return pOut; }


static inline float vec_norm(const arm_matrix_instance_f32 *pSrcA) {
  float32_t s = 0;
  configASSERT( pSrcA->numCols == 1 );
  
  float *a = pSrcA->pData;
  
  for (int i=0; i<pSrcA->numRows; i++) {
    s += a[0+1*i]*a[0+1*i];
  }
  return arm_sqrt(s);
}

static inline void vec_normalize(arm_matrix_instance_f32 *pSrcA) {
  float norm = vec_norm(pSrcA);
  
  float *a = pSrcA->pData;
  
  for (int i=0; i<pSrcA->numRows; i++) {
    a[i] = a[i]/norm;
  }
}

static inline void vec_negate(arm_matrix_instance_f32 *pSrcA) {
  configASSERT( pSrcA->numCols == 1 );
  
  float *a = pSrcA->pData;
  
  for (int i=0; i<pSrcA->numRows; i++) {
    a[i] = -1*a[i];
  }
}

static inline float vec_dot(const arm_matrix_instance_f32 *pSrcA, const arm_matrix_instance_f32 *pSrcB) {
  float32_t s = 0;
  configASSERT( pSrcA->numCols == 1 );
  configASSERT( pSrcB->numCols == 1 );
  configASSERT( pSrcA->numRows == pSrcB->numRows );
  
  float *a = pSrcA->pData;
  float *b = pSrcB->pData;
  
  for (int i=0; i<pSrcA->numRows; i++) {
    s += a[0+1*i] * b[0+1*i];
  }
  return s;
}

static inline void vec_cross(const arm_matrix_instance_f32 *pSrcA, const arm_matrix_instance_f32 *pSrcB, arm_matrix_instance_f32 *pDst) {
  configASSERT( pSrcA->numCols == 1 );
  configASSERT( pSrcB->numCols == 1 );
  configASSERT( pDst->numCols == 1 );
  configASSERT( pSrcA->numRows == 3 );
  configASSERT( pSrcB->numRows == 3 );
  configASSERT( pDst->numRows == 3 );
  
  float *a = pSrcA->pData;
  float *b = pSrcB->pData;
  float *c = pDst->pData;
  
  c[0] = a[1]*b[2] - a[2]*b[1];
  c[1] = a[2]*b[0] - a[0]*b[2];
  c[2] = a[0]*b[1] - a[1]*b[0];
}

static inline void quaternion_normalize(arm_matrix_instance_f32 *pSrcA) {
  configASSERT( pSrcA->numCols == 1 );
  configASSERT( pSrcA->numRows == 4 ); // vector is a quaternion [w, x, y, z]
  
  vec_normalize(pSrcA);
  if (pSrcA->pData[0] < 0) {
    vec_negate(pSrcA);
  }
}

static inline void quaternion_inverse(const arm_matrix_instance_f32 *pSrcA, arm_matrix_instance_f32 *pDst) {
  configASSERT( pSrcA->numCols == 1 );
  configASSERT( pSrcA->numRows == 4 ); // vector is a quaternion [w, x, y, z]
  configASSERT( pDst->numCols == 1 );
  configASSERT( pDst->numRows == 4 ); // vector is a quaternion [w, x, y, z]
  
  float *a = pSrcA->pData;
  float *c = pDst->pData;
  c[0] = a[0];
  c[1] = -a[1];
  c[2] = -a[2];
  c[3] = -a[3];
  
  vec_normalize(pDst);
}

static inline void quaternion_multiply(const arm_matrix_instance_f32 *pSrcA, const arm_matrix_instance_f32 *pSrcB, arm_matrix_instance_f32 *pDst) {
  configASSERT( pSrcA->numCols == 1 );
  configASSERT( pSrcA->numRows == 4 ); // vector is a quaternion [w, x, y, z]
  configASSERT( pSrcB->numCols == 1 );
  configASSERT( pSrcB->numRows == 4 ); // vector is a quaternion [w, x, y, z]
  configASSERT( pDst->numCols == 1 );
  configASSERT( pDst->numRows == 4 ); // vector is a quaternion [w, x, y, z]
  
  float *a = pSrcA->pData;
  float *b = pSrcB->pData;
  float *c = pDst->pData;
  
  c[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
  c[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
  c[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
  c[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

static uint32_t lastReferenceTimestamp;
static uint32_t lastExternalPositionTimestamp;
static xQueueHandle externalPositionQueue;
static xQueueHandle referenceQueue;

// Struct for logging position information
static positionMeasurement_t ext_pos;

static bool isInit = false;

static void stateControllerCrtpCB(CRTPPacket* pk);

void stateControllerInit(void)
{
  if (isInit) {
    return;
  }

  externalPositionQueue = xQueueCreate(1, sizeof(positionMeasurement_t));
  referenceQueue = xQueueCreate(1, sizeof(controlReference_t));
  
  crtpRegisterPortCB(CRTP_PORT_CONTROL, stateControllerCrtpCB);
  
  isInit = true;
}

static controlReference_t ref;
static float omegaIntegrators[3];
static float yawIntegrator;
static uint32_t lastControlUpdate;
#define CONTROL_RATE RATE_100_HZ // this is slower than the IMU update rate of 500Hz

void stateControllerRun(control_t *control, const sensorData_t *sensors, const state_t *state)
{  
  uint32_t ticksSinceLastCommand = (xTaskGetTickCount() - lastReferenceTimestamp);
  if (ticksSinceLastCommand > M2T(100)) { // require commands at 10Hz
    control->enable = false;
    return;
  }
  
  bool referenceReceived = (pdTRUE == xQueueReceive(referenceQueue, &ref, 0));
  
  if (referenceReceived)
  {
    if (ref.setEmergency)
    {
      control->enable = false;
    }
    else if (ref.resetEmergency)
    {
      control->enable = true;
      omegaIntegrators[0] = omegaIntegrators[1] = omegaIntegrators[2] = 0;
    }
  }
  
  if (!control->enable)
  {
    return;
  }

  // define this here, since we do body-rate control at 1000Hz below the following if statement
  float omega[3] = {0};
  omega[0] = sensors->gyro.x * DEG_TO_RAD;
  omega[1] = sensors->gyro.y * DEG_TO_RAD;
  omega[2] = sensors->gyro.z * DEG_TO_RAD;
  
  // update at the CONTROL_RATE (100Hz)
  if (referenceReceived || (xTaskGetTickCount()-lastControlUpdate) > configTICK_RATE_HZ/CONTROL_RATE)
  {
    lastControlUpdate = xTaskGetTickCount();
    
    // desired accelerations
    float accDes[3] = {0};  
    // desired thrust
    float collCmd = 0;
  
    // attitude error as computed by the reduced attitude controller
    float attErrorReduced[4] = {0};
    arm_matrix_instance_f32 attErrorReduced_m = {4, 1, attErrorReduced};
  
    // desired attitude as computed by the reduced attitude controller
    float attDesiredReduced[4] = {0};
    arm_matrix_instance_f32 attDesiredReduced_m = {4, 1, attDesiredReduced};
  
    // attitude error as computed by the full attitude controller
    float attErrorFull[4] = {0};
    arm_matrix_instance_f32 attErrorFull_m = {4, 1, attErrorFull};
  
    // desired attitude as computed by the full attitude controller
    float attDesiredFull[4] = {0};
    arm_matrix_instance_f32 attDesiredFull_m = {4, 1, attDesiredFull};
  
    // current attitude
    quaternion_t sq = state->attitudeQuaternion;
    float q[4] = {sq.w, sq.x, sq.y, sq.z};
    arm_matrix_instance_f32 attitude_m = {4, 1, q};
  
    // inverse of current attitude
    float qI[4] = {0};
    arm_matrix_instance_f32 attitudeI_m = {4, 1, qI};
    quaternion_inverse(&attitude_m, &attitudeI_m);
    
    // body frame -> inertial frame :  vI = R*vB
    float R[3][3] = {0};
  
    R[0][0] = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
    R[0][1] = 2 * q[1] * q[2] - 2 * q[0] * q[3];
    R[0][2] = 2 * q[1] * q[3] + 2 * q[0] * q[2];
  
    R[1][0] = 2 * q[1] * q[2] + 2 * q[0] * q[3];
    R[1][1] = q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3];
    R[1][2] = 2 * q[2] * q[3] - 2 * q[0] * q[1];
  
    R[2][0] = 2 * q[1] * q[3] - 2 * q[0] * q[2];
    R[2][1] = 2 * q[2] * q[3] + 2 * q[0] * q[1];
    R[2][2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
    
    // a few temporary quaternions
    float temp1[4] = {0};
    arm_matrix_instance_f32 temp1_m = {4, 1, temp1};
  
    float temp2[4] = {0};
    arm_matrix_instance_f32 temp2_m = {4, 1, temp2};
    
    // compute the position and velocity errors
    float pError[] = {ref.x[0] - state->position.x,
                      ref.y[0] - state->position.y,
                      ref.z[0] - state->position.z};
  
    float vError[] = {ref.x[1] - state->velocity.x,
                      ref.y[1] - state->velocity.y,
                      ref.z[1] - state->velocity.z};
  
  
    // ====== LINEAR CONTROL ======
  
    // compute desired accelerations in X, Y and Z
    accDes[0] = 0;
    if (CONTROLMODE_POSITION(ref.xmode))
    { accDes[0] += 1.0f / tau_xy / tau_xy * pError[0]; }
    if (CONTROLMODE_VELOCITY(ref.xmode))
    { accDes[0] += 2.0f * zeta_xy / tau_xy * vError[0]; }
    if (CONTROLMODE_ACCELERATION(ref.xmode))
    { accDes[0] += ref.x[2]; }
    accDes[0] = constrain(accDes[0], -coll_max, coll_max);
  
    accDes[1] = 0;
    if (CONTROLMODE_POSITION(ref.ymode))
    { accDes[1] += 1.0f / tau_xy / tau_xy * pError[1]; }
    if (CONTROLMODE_VELOCITY(ref.ymode))
    { accDes[1] += 2.0f * zeta_xy / tau_xy * vError[1]; }
    if (CONTROLMODE_ACCELERATION(ref.ymode))
    { accDes[1] += ref.y[2]; }
    accDes[1] = constrain(accDes[1], -coll_max, coll_max);
  
    accDes[2] = GRAVITY;
    if (CONTROLMODE_POSITION(ref.zmode))
    { accDes[2] += 1.0f / tau_z / tau_z * pError[2]; }
    if (CONTROLMODE_VELOCITY(ref.zmode))
    { accDes[2] += 2.0f * zeta_z / tau_z * vError[2]; }
    if (CONTROLMODE_ACCELERATION(ref.zmode))
    { accDes[2] += ref.z[2]; }
    accDes[2] = constrain(accDes[2], -coll_max, coll_max);
  
  
    // ====== THRUST CONTROL ======
  
    // compute commanded thrust required to achieve the z acceleration
    collCmd = accDes[2] / R[2][2];
    
    if (collCmd > coll_max) {
      // exceeding the thrust threshold
      // we compute a reduction factor r based on fairness f \in [0,1] such that:
      // collMax^2 = (r*x)^2 + (r*y)^2 + (r*f*z + (1-f)z + g)^2
      float x = accDes[0];
      float y = accDes[1];
      float z = accDes[2] - GRAVITY;
      float g = GRAVITY;
      float f = constrain(thrust_reduction_fairness, 0, 1);
      
      // solve as a quadratic
      float a = powf(x, 2) + powf(y, 2) + powf(z*f, 2);
      float b = 2 * z*f*((1-f)*z + g);
      float c = powf((1-f)*z + g, 2) - powf(coll_max, 2);
      
      float r = (-b + arm_sqrt(powf(b, 2) - 4.0f*a*c))/(2.0f*a);
      r = constrain(r,0,1);
      
      accDes[0] = r*x;
      accDes[1] = r*y;
      accDes[2] = (r*f+(1-f))*z + g;
    }
    collCmd = constrain(accDes[2] / R[2][2], coll_min, coll_max);
  
    // FYI: this thrust will result in the accelerations
    // xdd = R02*coll
    // ydd = R12*coll
  
    // a unit vector pointing in the direction of the desired thrust (ie. the direction of body's z axis in the inertial frame)
    float zI_des[3] = {accDes[0], accDes[1], accDes[2]};
    arm_matrix_instance_f32 zI_des_m = {3, 1, zI_des};
    vec_normalize(&zI_des_m);
  
    // a unit vector pointing in the direction of the current thrust
    float zI_cur[3] = {R[0][2], R[1][2], R[2][2]};
    arm_matrix_instance_f32 zI_cur_m = {3, 1, zI_cur};
    vec_normalize(&zI_cur_m);
  
    // a unit vector pointing in the direction of the inertial frame z-axis
    float zI[3] = {0, 0, 1};
    arm_matrix_instance_f32 zI_m = {3, 1, zI};
  
  
  
    // ====== REDUCED ATTITUDE CONTROL ======
  
    // compute the error angle between the current and the desired thrust directions
    float dotProd = vec_dot(&zI_cur_m, &zI_des_m);
    dotProd = constrain(dotProd, -1, 1);
    float alpha = acosf(dotProd);
  
    // the axis around which this rotation needs to occur in the inertial frame (ie. an axis orthogonal to the two)
    float rotAxisI[3] = {0};
    arm_matrix_instance_f32 rotAxisI_m = {3, 1, rotAxisI};
    if (fabsf(alpha) > 1 * ARCMINUTE)
    {
      vec_cross(&zI_cur_m, &zI_des_m, &rotAxisI_m);
      vec_normalize(&rotAxisI_m);
    }
    else
    {
      rotAxisI[0] = 1;
      rotAxisI[1] = 1;
      rotAxisI[2] = 0;
    }
  
    // the attitude error quaternion
    attErrorReduced[0] = cosf(alpha / 2.0f);
    attErrorReduced[1] = sinf(alpha / 2.0f) * rotAxisI[0];
    attErrorReduced[2] = sinf(alpha / 2.0f) * rotAxisI[1];
    attErrorReduced[3] = sinf(alpha / 2.0f) * rotAxisI[2];
  
    // choose the shorter rotation
    if (sinf(alpha / 2.0f) < 0)
    {
      vec_negate(&rotAxisI_m);
    }
    if (cosf(alpha / 2.0f) < 0)
    {
      vec_negate(&rotAxisI_m);
      vec_negate(&attErrorReduced_m);
    }
    
    quaternion_multiply(&attitude_m, &attErrorReduced_m, &attDesiredReduced_m);
    quaternion_normalize(&attErrorReduced_m);
    quaternion_normalize(&attDesiredReduced_m);
  
  
    // ====== FULL ATTITUDE CONTROL ======
  
    // compute the error angle between the inertial and the desired thrust directions
    dotProd = vec_dot(&zI_m, &zI_des_m);
    dotProd = constrain(dotProd, -1, 1);
    alpha = acosf(dotProd);
  
    // the axis around which this rotation needs to occur in the inertial frame (ie. an axis orthogonal to the two)
    if (fabsf(alpha) > 1 * ARCMINUTE)
    {
      vec_cross(&zI_m, &zI_des_m, &rotAxisI_m);
      vec_normalize(&rotAxisI_m);
    }
    else
    {
      rotAxisI[0] = 1;
      rotAxisI[1] = 1;
      rotAxisI[2] = 0;
    }
  
    // the quaternion corresponding to a roll and pitch around this axis
    float attFullReqPitchRoll[4] = {0};
    arm_matrix_instance_f32 attFullReqPitchRoll_m = {4, 1, attFullReqPitchRoll};
    attFullReqPitchRoll[0] = cosf(alpha / 2.0f);
    attFullReqPitchRoll[1] = sinf(alpha / 2.0f) * rotAxisI[0];
    attFullReqPitchRoll[2] = sinf(alpha / 2.0f) * rotAxisI[1];
    attFullReqPitchRoll[3] = sinf(alpha / 2.0f) * rotAxisI[2];
  
    // the quaternion corresponding to a rotation to the desired yaw
    float attFullReqYaw[4] = {0};
    arm_matrix_instance_f32 attFullReqYaw_m = {4, 1, attFullReqYaw};
    attFullReqYaw[0] = cosf(ref.yaw[0] / 2.0f);
    attFullReqYaw[1] = 0;
    attFullReqYaw[2] = 0;
    attFullReqYaw[3] = sinf(ref.yaw[0] / 2.0f);
  
    // the full rotation (roll & pitch, then yaw)
    quaternion_multiply(&attFullReqPitchRoll_m, &attFullReqYaw_m, &attDesiredFull_m);
  
    // back transform from the current attitude to get the error between rotations
    quaternion_multiply(&attitudeI_m, &attDesiredFull_m, &attErrorFull_m);
  
    // correct rotation
    if (attErrorFull[0] < 0)
    {
      vec_negate(&attErrorFull_m);
      quaternion_multiply(&attitude_m, &attErrorFull_m, &attDesiredFull_m);
    }
  
    quaternion_normalize(&attErrorFull_m);
    quaternion_normalize(&attDesiredFull_m);
  
  
    // ====== MIXING FULL & REDUCED CONTROL ======
 
    float attError[4] = {0};
    arm_matrix_instance_f32 attError_m = {4, 1, attError};
  
    if (mixing_factor <= 0)
    {
      // 100% reduced control (no yaw control)
      memcpy(attError, attErrorReduced, sizeof(attError));
    }
    else if (mixing_factor >= 1)
    {
      // 100% full control (yaw controlled with same time constant as roll & pitch)
      memcpy(attError, attErrorFull, sizeof(attError));
    }
    else
    {
      // mixture of reduced and full control
    
      // calculate rotation between the two errors
      quaternion_inverse(&attErrorReduced_m, &temp1_m);
      quaternion_multiply(&temp1_m, &attErrorFull_m, &temp2_m);
      quaternion_normalize(&temp2_m);
    
      // by defintion this rotation has the form [cos(alpha/2), 0, 0, sin(alpha/2)]
      // where the first element gives the rotation angle, and the last the direction
      alpha = 2.0f * acosf(constrain(temp2[0], -1, 1));
    
      // bisect the rotation from reduced to full control
      temp1[0] = cosf(alpha * mixing_factor / 2.0f);
      temp1[1] = 0;
      temp1[2] = 0;
      temp1[3] = sinf(alpha * mixing_factor / 2.0f) * (temp2[3] < 0 ? -1 : 1); // rotate in the correct direction
      
      quaternion_multiply(&attErrorReduced_m, &temp1_m, &attError_m);
      quaternion_normalize(&attError_m);
    }
  
    // TODO: Heuristic for yaw rotation direction based on current rotation axis & speed?
  
  
    // ====== COMPUTE CONTROL SIGNALS ======
      
    // compute the commanded body rates
    control->omega[0] = 2.0f / tau_rp * attError[1];
    control->omega[1] = 2.0f / tau_rp * attError[2];
    control->omega[2] = 2.0f / tau_rp * attError[3] + ref.yaw[1]; // due to the mixing, this will behave with time constant tau_yaw
  
    if (igain_yaw > 0)
    {
      yawIntegrator = constrain((1.0f - igain_yaw) * yawIntegrator + igain_yaw * attError[3], -imax_yaw, imax_yaw);
      control->omega[2] += yawIntegrator;
    }
    
    // scale the commands to satisfy rate constraints
    float scaling = 1;
    scaling = max(scaling, fabsf(control->omega[0]) / omega_rp_max);
    scaling = max(scaling, fabsf(control->omega[1]) / omega_rp_max);
    scaling = max(scaling, fabsf(control->omega[2]) / omega_yaw_max);
  
    control->omega[0] /= scaling;
    control->omega[1] /= scaling;
    control->omega[2] /= scaling;
    control->thrust = collCmd;
  }
  
  // control the body torques
  float omegaErr[3] = {(control->omega[0] - omega[0])/tau_rp_rate,
                       (control->omega[1] - omega[1])/tau_rp_rate,
                       (control->omega[2] - omega[2])/tau_yaw_rate};
  
  if (igain_rate > 0)
  {
    for (int i = 0; i < 3; i++)
    {
      omegaIntegrators[i] = (1.0f - igain_rate) * omegaIntegrators[i] + igain_rate * omegaErr[i];
      omegaErr[i] += omegaIntegrators[i];
    }
  }
    
  
  arm_matrix_instance_f32 omegaErr_m = {3, 1, omegaErr};
  arm_matrix_instance_f32 torques_m = {3, 1, control->torque};
  
  // update the commanded body torques based on the current error in body rates
  mat_mult(&INERTIA_m, &omegaErr_m, &torques_m);
}


static void stateControllerCrtpCB(CRTPPacket* pk)
{
  crtpControlPacketHeader_t *header = (crtpControlPacketHeader_t*)pk->data;
  
  if (header->packetHasExternalReference)
  {
    crtpControlPacketWithExternalPosition_t *packet = (crtpControlPacketWithExternalPosition_t*)pk->data;
    
    positionMeasurement_t externalPosition;
    externalPosition.x = half2single(packet->x[3]);
    externalPosition.y = half2single(packet->y[3]);
    externalPosition.z = half2single(packet->z[3]);
    externalPosition.stdDev = EXTERNAL_MEASUREMENT_STDDEV;
  
    controlReference_t controlReference;
    controlReference.xmode = header->controlModeX;
    controlReference.ymode = header->controlModeY;
    controlReference.zmode = header->controlModeZ;
    controlReference.setEmergency = packet->header.setEmergency;
    controlReference.resetEmergency = packet->header.resetEmergency;
  
    for (int i = 0; i<3; i++) { controlReference.x[i] = half2single(packet->x[i]); }
    for (int i = 0; i<3; i++) { controlReference.y[i] = half2single(packet->y[i]); }
    for (int i = 0; i<3; i++) { controlReference.z[i] = half2single(packet->z[i]); }
    for (int i = 0; i<2; i++) { controlReference.yaw[i] = half2single(packet->yaw[i]); }
    
    xQueueOverwrite(externalPositionQueue, &externalPosition);
    lastExternalPositionTimestamp = xTaskGetTickCount();
    xQueueOverwrite(referenceQueue, &controlReference);
    lastReferenceTimestamp = xTaskGetTickCount();
  }
  else
  {
    crtpControlPacket_t *packet = (crtpControlPacket_t *) pk->data;
  
    controlReference_t controlReference;
    controlReference.xmode = header->controlModeX;
    controlReference.ymode = header->controlModeY;
    controlReference.zmode = header->controlModeZ;
    controlReference.setEmergency = packet->header.setEmergency;
    controlReference.resetEmergency = packet->header.resetEmergency;
  
    for (int i = 0; i<3; i++) { controlReference.x[i] = half2single(packet->x[i]); }
    for (int i = 0; i<3; i++) { controlReference.y[i] = half2single(packet->y[i]); }
    for (int i = 0; i<3; i++) { controlReference.z[i] = half2single(packet->z[i]); }
    for (int i = 0; i<2; i++) { controlReference.yaw[i] = half2single(packet->yaw[i]); }
  
    xQueueOverwrite(referenceQueue, &controlReference);
    lastReferenceTimestamp = xTaskGetTickCount();
  }
}


void stateControllerUpdateStateWithExternalPosition()
{
  // Only use position information if it's valid and recent
  if ((xTaskGetTickCount() - lastExternalPositionTimestamp) < M2T(5)) {
    // Get the updated position from the mocap
    positionMeasurement_t externalPosition;
    if (pdTRUE == xQueueReceive(externalPositionQueue, &externalPosition, 0))
    {
      stateEstimatorEnqueuePosition(&externalPosition);
    }
  }
}

bool stateControllerTest(void) { return true; }

LOG_GROUP_START(ext_pos)
  LOG_ADD(LOG_FLOAT, X, &ext_pos.x)
  LOG_ADD(LOG_FLOAT, Y, &ext_pos.y)
  LOG_ADD(LOG_FLOAT, Z, &ext_pos.z)
LOG_GROUP_STOP(ext_pos)

PARAM_GROUP_START(ctrlr)
PARAM_ADD(PARAM_FLOAT, tau_xy, &tau_xy)
PARAM_ADD(PARAM_FLOAT, zeta_xy, &zeta_xy)
PARAM_ADD(PARAM_FLOAT, tau_z, &tau_z)
PARAM_ADD(PARAM_FLOAT, zeta_z, &zeta_z)
PARAM_ADD(PARAM_FLOAT, tau_rp, &tau_rp)
PARAM_ADD(PARAM_FLOAT, mixing_factor, &mixing_factor)
PARAM_ADD(PARAM_FLOAT, coll_fairness, &thrust_reduction_fairness)
PARAM_ADD(PARAM_FLOAT, tau_rp_rate, &tau_rp_rate)
PARAM_ADD(PARAM_FLOAT, tau_yaw_rate, &tau_yaw_rate)
PARAM_ADD(PARAM_FLOAT, coll_min, &coll_min)
PARAM_ADD(PARAM_FLOAT, coll_max, &coll_max)
PARAM_ADD(PARAM_FLOAT, omega_rp_max, &omega_rp_max)
PARAM_ADD(PARAM_FLOAT, omega_yaw_max, &omega_yaw_max)
PARAM_GROUP_STOP(ctrlr)

