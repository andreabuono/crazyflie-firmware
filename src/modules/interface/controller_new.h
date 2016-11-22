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
void stateControllerUpdateStateWithExternalPosition();

#define EXTERNAL_MEASUREMENT_STDDEV (0.02)

typedef enum {
  CONTROLMODE_ACCELERATION,
  CONTROLMODE_VELOCITY,
  CONTROLMODE_POSITION
} controlMode_t;

typedef struct {
  controlMode_t mode;
  float pos;
  float vel;
  float acc;
} controlReferenceAxis_t;

typedef struct {
  controlReferenceAxis_t x;
  controlReferenceAxis_t y;
  controlReferenceAxis_t z;
  float yaw;
} controlReference_t;

typedef struct {
  uint16_t pos; // use uint16_t to hold float16_t
  uint16_t vel; // use uint16_t to hold float16_t
  float acc;
} __attribute__((packed)) crtpControlReference_t;

typedef struct {
  uint16_t pos; // use uint16_t to hold float16_t
  uint16_t vel; // use uint16_t to hold float16_t
  uint16_t acc; // use uint16_t to hold float16_t
  uint16_t extPos; // use uint16_t to hold float16_t
} __attribute__((packed)) crtpControlReferenceWithExternalPosition_t;

typedef struct {
  uint8_t packetHasExternalReference:1;
  uint8_t setEmergency:1;
  uint8_t resetEmergency:1;
  uint8_t :5;
  uint8_t controlModeX:2;
  uint8_t controlModeY:2;
  uint8_t controlModeZ:2;
  uint8_t :2;
} __attribute__((packed)) crtpControlPacketHeader_t; // size 2

typedef struct
{
  crtpControlPacketHeader_t header; // size 2
  crtpControlReference_t x; // size 8
  crtpControlReference_t y; // size 8
  crtpControlReference_t z; // size 8
  float yaw;
} __attribute__((packed)) crtpControlPacket_t;

typedef struct
{
  crtpControlPacketHeader_t header;
  crtpControlReferenceWithExternalPosition_t x;
  crtpControlReferenceWithExternalPosition_t y;
  crtpControlReferenceWithExternalPosition_t z;
  float yaw;
} __attribute__((packed)) crtpControlPacketWithExternalPosition_t;
