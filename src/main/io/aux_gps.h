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

typedef gpsConfig_t auxGpsConfig_t;
PG_DECLARE(auxGpsConfig_t, auxGpsConfig);

extern char auxGpsPacketLog[GPS_PACKET_LOG_ENTRY_COUNT];

extern int32_t aux_GPS_home[2];
extern uint16_t aux_GPS_distanceToHome;        // distance to home point in meters
extern int16_t aux_GPS_directionToHome;        // direction to home or hol point in degrees
extern uint32_t aux_GPS_distanceTraveledInCm;     // distance flown since armed in centimeters
extern int16_t aux_GPS_angle[ANGLE_INDEX_COUNT];  // it's the angles that must be applied for GPS correction
extern float aux_dTnav;                           // Delta Time in milliseconds for navigation computations, updated with every good GPS read
extern float aux_GPS_scaleLonDown;                // this is used to offset the shrinking longitude as we go towards the poles

extern gpsData_t auxGpsData;
extern gpsSolutionData_t auxGpsSol;

extern uint8_t aux_GPS_update;                 // toggle to distinct a GPS position update (directly or via MSP)
extern uint32_t aux_GPS_packetCount;
extern uint32_t aux_GPS_svInfoReceivedCount;
extern uint8_t aux_GPS_numCh;                  // Number of channels
extern uint8_t aux_GPS_svinfo_chn[16];         // Channel number
extern uint8_t aux_GPS_svinfo_svid[16];        // Satellite ID
extern uint8_t aux_GPS_svinfo_quality[16];     // Bitfield Qualtity
extern uint8_t aux_GPS_svinfo_cno[16];         // Carrier to Noise Ratio (Signal Strength)

void auxGpsInit(void);
void auxGpsUpdate(timeUs_t currentTimeUs);
bool auxGpsNewFrame(uint8_t c);
bool auxGpsIsHealthy(void); // Check for healthy communications
struct serialPort_s;
void auxGpsEnablePassthrough(struct serialPort_s *auxGpsPassthroughPort);
void onAuxGpsNewData(void);
void aux_GPS_reset_home_position(void);
void aux_GPS_calc_longitude_scaling(int32_t lat);
void aux_GPS_calc_longitude_scaling(int32_t lat);
void aux_GPS_distance_cm_bearing(int32_t *currentLat1, int32_t *currentLon1, int32_t *destinationLat2, int32_t *destinationLon2, uint32_t *dist, int32_t *bearing);

