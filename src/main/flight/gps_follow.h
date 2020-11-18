/*
 * This file is part of Betaflight.
 *
 * Betaflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Betaflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Betaflight. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "common/axis.h"

#include "pg/pg.h"

typedef struct gpsFollow_s {
    uint16_t angle;                     // degrees
    uint16_t initialAltitudeM;          // meters
    uint16_t descentDistanceM;          // meters
    uint16_t crosstrackGroundSpeed;     // cm / sec
    uint16_t followGroundSpeed;         // cm / sec
    uint16_t throttleP, throttleI, throttleD;
    uint16_t velP, velI, velD;
    uint16_t yawP;
    uint16_t throttleMin;
    uint16_t throttleMax;
    uint16_t throttleHover;
    uint8_t sanityChecks;
    uint8_t minSats;
    uint8_t minSatsAux;
    uint16_t minFollowDistance;
    uint8_t allowArmingWithoutFix;
    uint8_t useMag;
    uint16_t targetFollowAltitudeM;
    uint16_t targetFollowDistanceM;
    uint8_t altitudeMode;               // might not need this
    uint16_t ascendRate;
    uint16_t descendRate;
} gpsFollowConfig_t;

PG_DECLARE(gpsFollowConfig_t, gpsFollowConfig);

extern int32_t gpsFollowAngle[ANGLE_INDEX_COUNT]; // centidegrees

void updateGPSFollowState(void);
void followNewGpsData(void);

float gpsFollowGetYawRate(void);
float gpsFollowGetThrottle(void);
bool gpsFollowIsConfigured(void);
bool gpsFollowIsAvailable(void);
bool gpsFollowIsDisabled(void);
bool gpsFollowDisableMag(void);
