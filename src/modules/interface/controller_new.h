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


#pragma once

#include "stabilizer_types.h"

void stateControllerInit(void);
bool stateControllerTest(void);
void stateControllerRun(control_t *control, const sensorData_t *sensors, const state_t *state);

#define CONTROL_RATE RATE_100_HZ // this is slower than the IMU update rate of 500Hz

#define CONTROLMODE_ACCELERATION(mode) ((0b001 & mode) != 0)
#define CONTROLMODE_VELOCITY(mode)     ((0b010 & mode) != 0)
#define CONTROLMODE_POSITION(mode)     ((0b100 & mode) != 0)

typedef struct {
  bool setEmergency;
  bool resetEmergency;
  uint8_t xmode, ymode, zmode;
  float x[3];
  float y[3];
  float z[3];
  float yaw;
  float p, q, r;
} controlReference_t;

typedef struct {
  uint16_t setEmergency:1;
  uint16_t resetEmergency:1;
  uint16_t controlModeX:3;
  uint16_t controlModeY:3;
  uint16_t controlModeZ:3;
  uint16_t :4;
} __attribute__((packed)) crtpControlPacketHeader_t; // size 2

typedef struct
{
  crtpControlPacketHeader_t header; // size 2
  uint16_t x[3]; //x pos, vel, acc
  uint16_t y[3]; //y pos, vel, acc
  uint16_t z[3]; //z pos, vel, acc
  uint16_t yaw; // yaw angle
  uint16_t p, q, r; // body-rate feed forward
} __attribute__((packed)) crtpControlPacket_t;
