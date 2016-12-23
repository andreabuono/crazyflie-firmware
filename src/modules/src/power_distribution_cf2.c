/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
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
 * power_distribution_cf2.c - Crazyflie 2.0 stock power distribution code
 */
#include "FreeRTOS.h"
#include "task.h"
#include "arm_math.h"

#include "power_distribution.h"

#include "log.h"
#include "param.h"
#include "num.h"

#include "motors.h"

// TODO: move these into a physical constants file
// thrust = a * pwm^2 + b * pwm
const float PWM_TO_THRUST_a = .091492681f;
const float PWM_TO_THRUST_b = .067673604f;
const float PWM_TO_THRUST_b_sq = .067673604f*.067673604f;
const float PWM_TO_THRUST_c = 5.484560e-4f;

const float SHUTOFF_THRUST = 4.47e-3; // this would cause all motors to rotate with cmd 1000, too slow.

// motor torque = m * thrust
const float THRUST_TO_TORQUE_m = 0.005964552f;

// constants
const float CRAZYFLIE_ARM_LENGTH = 0.046f; // m
const float CRAZYFLIE_MASS = 30e-3f; // kg

static inline float arm_sqrt(float32_t in)
{ float pOut = 0; arm_status result = arm_sqrt_f32(in, &pOut); configASSERT(ARM_MATH_SUCCESS == result); return pOut; }

static float motor_pwm[4] = {0};

void powerDistributionInit(void)
{
  motorsInit(motorMapDefaultBrushed);
}

bool powerDistributionTest(void)
{
  bool pass = true;

  pass &= motorsTest();

  return pass;
}

#define limitThrust(VAL) limitUint16(VAL)

void powerDistribution(const control_t *control)
{
  if (!control->enable || control->thrust * CRAZYFLIE_MASS < SHUTOFF_THRUST) {
    motorsSetRatio(MOTOR_M1, 0);
    motorsSetRatio(MOTOR_M2, 0);
    motorsSetRatio(MOTOR_M3, 0);
    motorsSetRatio(MOTOR_M4, 0);
    return;
  }
  
  float motor_forces[4] = {0};
  float Tpart = control->thrust * CRAZYFLIE_MASS; // force to provide control->thrust
  float Zpart = control->torque[2] / THRUST_TO_TORQUE_m; // force to provide z torque
  
  #ifdef QUAD_FORMATION_X
  float Xpart = control->torque[0] / (CRAZYFLIE_ARM_LENGTH * .707106781f); // force to provide x torque
  float Ypart = control->torque[1] / (CRAZYFLIE_ARM_LENGTH * .707106781f); // force to provide y torque
  
  motor_forces[0] = Tpart/4.0f - Xpart/4.0f - Ypart/4.0f + Zpart/4.0f;
  motor_forces[1] = Tpart/4.0f - Xpart/4.0f + Ypart/4.0f - Zpart/4.0f;
  motor_forces[2] = Tpart/4.0f + Xpart/4.0f + Ypart/4.0f + Zpart/4.0f;
  motor_forces[3] = Tpart/4.0f + Xpart/4.0f - Ypart/4.0f - Zpart/4.0f;
    
  #else // QUAD_FORMATION_NORMAL
    motorPower.m1 = limitThrust(control->thrust + control->pitch +
                               control->yaw);
    motorPower.m2 = limitThrust(control->thrust - control->roll -
                               control->yaw);
    motorPower.m3 =  limitThrust(control->thrust - control->pitch +
                               control->yaw);
    motorPower.m4 =  limitThrust(control->thrust + control->roll -
                               control->yaw);
  #endif
  
  for (int i=0; i<4; i++) {
    if (motor_forces[i] < 0) {
      motor_forces[i] = 0;
      motor_pwm[i] = 0;
    } else {
      motor_pwm[i] = (-PWM_TO_THRUST_b + arm_sqrt(PWM_TO_THRUST_b_sq + 4.0f * PWM_TO_THRUST_a * motor_forces[i])) /
                     (2.0f * PWM_TO_THRUST_a);
      motor_pwm[i] = constrain(motor_pwm[i], 0.1, 1);
    }
    
    motorsSetRatio(i, (uint16_t)(65536.0f*motor_pwm[i]/10.0f));
  }
}

LOG_GROUP_START(motorpwm)
LOG_ADD(LOG_FLOAT, m1, &motor_pwm[0])
LOG_ADD(LOG_FLOAT, m2, &motor_pwm[1])
LOG_ADD(LOG_FLOAT, m3, &motor_pwm[2])
LOG_ADD(LOG_FLOAT, m4, &motor_pwm[3])
LOG_GROUP_STOP(motorpwm)
