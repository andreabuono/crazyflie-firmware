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
 */

#include "FreeRTOS.h"
#include "task.h"
#include "arm_math.h"

#include "controller_new.h"
#include "crtp.h"
#include "log.h"
#include "num.h"


#ifdef ESTIMATOR_TYPE_kalman
#include "estimator_kalman.h"
#endif

static float tau_xy = 0.5f;
static float zeta_xy = 0.2f;

static float tau_z = 0.5f;
static float zeta_z = 0.75f;

static float tau_q = 0.1f;

static float zdd_min = -0.9f;
static float f_max = 1.8f;

#define GRAVITY (9.81f)

inline float norm(const int n, const float * const a) {
  float s = 0;
  for (int i=0; i<n; i++) {
    s += a[i]*a[i];
  }
  return sqrtf(s);
}

inline float dot(const int n, const float * const a, const float * const b) {
  float s = 0;
  for (int i=0; i<n; i++) {
    s += a[i]*b[i];
  }
  return s;
}

inline void quaternion_normalize(float a[4]) {
  float n = norm(4,a);
  for (int i=0; i<4; i++) {
    a[i] /= n;
  }
}

inline void quaternion_invert(const float * const a, float * result) {
  result[0] = a[0];
  result[1] = -a[1];
  result[2] = -a[2];
  result[3] = -a[3];
}

inline void quaternion_multiply(const float * const a, float * const b, float * result) {
  result[0] = a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3];
  result[1] = a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2];
  result[2] = a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1];
  result[3] = a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0];
}



static struct
{
  controlReference_t controlReference[2];
  bool activeSide;
  uint32_t timestamp; // FreeRTOS ticks
} ControlReferenceCache;

static struct
{
  positionMeasurement_t externalPosition[2];
  bool activeSide;
  uint32_t timestamp; // FreeRTOS ticks
} ExternalPositionCache;

// Struct for logging position information
static positionMeasurement_t ext_pos;

static bool isInit = false;

static void stateControllerCrtpCB(CRTPPacket* pk);

void stateControllerInit(void)
{
  if (isInit) {
    return;
  }

  crtpRegisterPortCB(CRTP_PORT_POSITION, stateControllerCrtpCB);
  isInit = true;
}


void stateControllerRun(control_t *control, const sensorData_t *sensors, const state_t *state)
{
  controlReference_t ref = ControlReferenceCache.controlReference[ControlReferenceCache.activeSide];
  
  float f_des = 0; // desired thrust
  float omega_des[3] = {0}; // desired body rates
  
  quaternion_t sq = state->attitudeQuaternion;
  float q[4] = {sq.w, sq.x, sq.y, sq.z};
  float R[3][3] = {0};
  
  // convert the attitude to a rotation matrix, such that we can rotate body-frame velocity and acc
  R[0][0] = q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
  R[0][1] = 2 * q[1] * q[2] - 2 * q[0] * q[3];
  R[0][2] = 2 * q[1] * q[3] + 2 * q[0] * q[2];
  
  R[1][0] = 2 * q[1] * q[2] + 2 * q[0] * q[3];
  R[1][1] = q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3];
  R[1][2] = 2 * q[2] * q[3] - 2 * q[0] * q[1];
  
  R[2][0] = 2 * q[1] * q[3] - 2 * q[0] * q[2];
  R[2][1] = 2 * q[2] * q[3] + 2 * q[0] * q[1];
  R[2][2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
  
  // compute required z acceleration
  float zdd = (1.0f/(tau_z*tau_z) * (ref.z.pos - state->position.z) +
               2.0f*zeta_z/tau_z * (ref.z.vel - state->velocity.z) +
               ref.z.acc);
  
  zdd = max(zdd, zdd_min*GRAVITY); // don't let us fall faster than gravity
  
  // compute required thrust to achieve zdd
  f_des = 1.0f/R[2][2] * (zdd + GRAVITY);
  f_des = min(f_max*GRAVITY, f_des); // cap thrust at 1.8*g
  
  // FYI: this thrust should result in the accelerations
  // xdd = R02*f
  // ydd = R12*f
  
  // compute required body angle based on desired x and y accelerations
  float xdd_des = (1.0f / (tau_xy*tau_xy) * (ref.x.pos - state->position.x) +
             2.0f * zeta_xy / tau_xy * (ref.x.vel - state->velocity.x) +
             ref.x.acc);
  
  float ydd_des = (1.0f / (tau_xy*tau_xy) * (ref.y.pos - state->position.y) +
             2.0f * zeta_xy / tau_xy * (ref.y.vel - state->velocity.y) +
             ref.y.acc);

  // desired acceleration in global frame
  float a_des[3] = {xdd_des, ydd_des, zdd+GRAVITY};
  float n_des = norm(3, a_des);
  
  
  // TODO: convert desired acceleration into roll, pitch, thrust
  // TODO: read current attitude from state->attitudeQuaternion
  // TODO: calculate rates based on the above 3 points (1st order control with time constant tau_rp and tau_yaw on angle difference)
  // TODO: complete the control structure with these calculated rates
  
}


static void stateControllerCrtpCB(CRTPPacket* pk)
{
  crtpControlPacketHeader_t *header = (crtpControlPacketHeader_t*)pk->data;
  
  if (header->packetHasExternalReference)
  {
    crtpControlPacketWithExternalPosition_t *packet = (crtpControlPacketWithExternalPosition_t*)pk->data;
    ExternalPositionCache.externalPosition[!ExternalPositionCache.activeSide].x = half2single(packet->x.extPos);
    ExternalPositionCache.externalPosition[!ExternalPositionCache.activeSide].y = half2single(packet->y.extPos);
    ExternalPositionCache.externalPosition[!ExternalPositionCache.activeSide].z = half2single(packet->z.extPos);
    ExternalPositionCache.externalPosition[!ExternalPositionCache.activeSide].stdDev = EXTERNAL_MEASUREMENT_STDDEV;
    
    ExternalPositionCache.activeSide = !ExternalPositionCache.activeSide;
    ExternalPositionCache.timestamp = xTaskGetTickCount();
    
    ControlReferenceCache.controlReference[!ControlReferenceCache.activeSide].x.pos = half2single(packet->x.pos);
    ControlReferenceCache.controlReference[!ControlReferenceCache.activeSide].x.vel = half2single(packet->x.vel);
    ControlReferenceCache.controlReference[!ControlReferenceCache.activeSide].x.acc = half2single(packet->x.acc);
    ControlReferenceCache.controlReference[!ControlReferenceCache.activeSide].x.mode = header->controlModeX;
  
    ControlReferenceCache.controlReference[!ControlReferenceCache.activeSide].y.pos = half2single(packet->y.pos);
    ControlReferenceCache.controlReference[!ControlReferenceCache.activeSide].y.vel = half2single(packet->y.vel);
    ControlReferenceCache.controlReference[!ControlReferenceCache.activeSide].y.acc = half2single(packet->y.acc);
    ControlReferenceCache.controlReference[!ControlReferenceCache.activeSide].y.mode = header->controlModeY;
  
    ControlReferenceCache.controlReference[!ControlReferenceCache.activeSide].z.pos = half2single(packet->z.pos);
    ControlReferenceCache.controlReference[!ControlReferenceCache.activeSide].z.vel = half2single(packet->z.vel);
    ControlReferenceCache.controlReference[!ControlReferenceCache.activeSide].z.acc = half2single(packet->z.acc);
    ControlReferenceCache.controlReference[!ControlReferenceCache.activeSide].z.mode = header->controlModeZ;
    
    ControlReferenceCache.controlReference[!ControlReferenceCache.activeSide].yaw = packet->yaw;
    
    ControlReferenceCache.activeSide = !ControlReferenceCache.activeSide;
    ControlReferenceCache.timestamp = xTaskGetTickCount();
  }
  else
  {
    crtpControlPacket_t *packet = (crtpControlPacket_t *) pk->data;
  
    ControlReferenceCache.controlReference[!ControlReferenceCache.activeSide].x.pos = half2single(packet->x.pos);
    ControlReferenceCache.controlReference[!ControlReferenceCache.activeSide].x.vel = half2single(packet->x.vel);
    ControlReferenceCache.controlReference[!ControlReferenceCache.activeSide].x.acc = packet->x.acc;
    ControlReferenceCache.controlReference[!ControlReferenceCache.activeSide].x.mode = header->controlModeX;
  
    ControlReferenceCache.controlReference[!ControlReferenceCache.activeSide].y.pos = half2single(packet->y.pos);
    ControlReferenceCache.controlReference[!ControlReferenceCache.activeSide].y.vel = half2single(packet->y.vel);
    ControlReferenceCache.controlReference[!ControlReferenceCache.activeSide].y.acc = packet->y.acc;
    ControlReferenceCache.controlReference[!ControlReferenceCache.activeSide].y.mode = header->controlModeY;
  
    ControlReferenceCache.controlReference[!ControlReferenceCache.activeSide].z.pos = half2single(packet->z.pos);
    ControlReferenceCache.controlReference[!ControlReferenceCache.activeSide].z.vel = half2single(packet->z.vel);
    ControlReferenceCache.controlReference[!ControlReferenceCache.activeSide].z.acc = packet->z.acc;
    ControlReferenceCache.controlReference[!ControlReferenceCache.activeSide].z.mode = header->controlModeZ;
  
    ControlReferenceCache.controlReference[!ControlReferenceCache.activeSide].yaw = packet->yaw;
  
    ControlReferenceCache.activeSide = !ControlReferenceCache.activeSide;
    ControlReferenceCache.timestamp = xTaskGetTickCount();
  }
}


void stateControllerUpdateStateWithExternalPosition()
{
  // Only use position information if it's valid and recent
  if ((xTaskGetTickCount() - ExternalPositionCache.timestamp) < M2T(5)) {
    // Get the updated position from the mocap
    ext_pos.x = ExternalPositionCache.externalPosition[ExternalPositionCache.activeSide].x;
    ext_pos.y = ExternalPositionCache.externalPosition[ExternalPositionCache.activeSide].y;
    ext_pos.z = ExternalPositionCache.externalPosition[ExternalPositionCache.activeSide].z;
    ext_pos.stdDev = 0.01;
    stateEstimatorEnqueuePosition(&ext_pos);
  }
}

bool stateControllerTest(void) { return true; }

LOG_GROUP_START(ext_pos)
  LOG_ADD(LOG_FLOAT, X, &ext_pos.x)
  LOG_ADD(LOG_FLOAT, Y, &ext_pos.y)
  LOG_ADD(LOG_FLOAT, Z, &ext_pos.z)
LOG_GROUP_STOP(ext_pos)

