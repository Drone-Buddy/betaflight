/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include "io/common_gps.h"

PG_DECLARE(gpsConfig_t, gpsConfig);

extern char gpsPacketLog[GPS_PACKET_LOG_ENTRY_COUNT];

extern int32_t GPS_home[2];
extern uint16_t GPS_distanceToHome;        // distance to home point in meters
extern int16_t GPS_directionToHome;        // direction to home or hol point in degrees
extern uint32_t GPS_distanceFlownInCm;     // distance flown since armed in centimeters
extern int16_t GPS_verticalSpeedInCmS;     // vertical speed in cm/s
extern int16_t GPS_angle[ANGLE_INDEX_COUNT];                // it's the angles that must be applied for GPS correction
extern float dTnav;             // Delta Time in milliseconds for navigation computations, updated with every good GPS read
extern float GPS_scaleLonDown;  // this is used to offset the shrinking longitude as we go towards the poles
extern int16_t nav_takeoff_bearing;

extern gpsData_t gpsData;
extern gpsSolutionData_t gpsSol;

extern uint8_t GPS_update;       // toogle to distinct a GPS position update (directly or via MSP)
extern uint32_t GPS_packetCount;
extern uint32_t GPS_svInfoReceivedCount;
extern uint8_t GPS_numCh;                  // Number of channels
extern uint8_t GPS_svinfo_chn[16];         // Channel number
extern uint8_t GPS_svinfo_svid[16];        // Satellite ID
extern uint8_t GPS_svinfo_quality[16];     // Bitfield Qualtity
extern uint8_t GPS_svinfo_cno[16];         // Carrier to Noise Ratio (Signal Strength)

void gpsInit(void);
void gpsUpdate(timeUs_t currentTimeUs);
bool gpsNewFrame(uint8_t c);
bool gpsIsHealthy(void); // Check for healthy communications
struct serialPort_s;
void gpsEnablePassthrough(struct serialPort_s *gpsPassthroughPort);
void onGpsNewData(void);
void GPS_reset_home_position(void);
void GPS_calc_longitude_scaling(int32_t lat);
void GPS_distance_cm_bearing(int32_t *currentLat1, int32_t *currentLon1, int32_t *destinationLat2, int32_t *destinationLon2, uint32_t *dist, int32_t *bearing);

