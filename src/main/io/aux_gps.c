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

#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <string.h>
#include <math.h>

#include "platform.h"

#ifdef USE_AUX_GPS

#include "build/build_config.h"
#include "build/debug.h"

#include "common/axis.h"
#include "common/gps_conversion.h"
#include "common/maths.h"
#include "common/utils.h"

#include "config/feature.h"
#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "drivers/light_led.h"
#include "drivers/time.h"

#include "io/dashboard.h"
#include "io/gps.h"
#include "io/aux_gps.h"
#include "io/serial.h"

#include "config/config.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/gps_follow.h"

#include "sensors/sensors.h"

#define LOG_ERROR        '?'
#define LOG_IGNORED      '!'
#define LOG_SKIPPED      '>'
#define LOG_NMEA_GGA     'g'
#define LOG_NMEA_RMC     'r'
#define LOG_UBLOX_SOL    'O'
#define LOG_UBLOX_STATUS 'S'
#define LOG_UBLOX_SVINFO 'I'
#define LOG_UBLOX_POSLLH 'P'
#define LOG_UBLOX_VELNED 'V'

#define GPS_SV_MAXSATS   16

char auxGpsPacketLog[GPS_PACKET_LOG_ENTRY_COUNT];
static char *auxGpsPacketLogChar = auxGpsPacketLog;
// **********************
// GPS
// **********************
uint16_t aux_GPS_distanceToHome;        // distance to home point in meters
int16_t aux_GPS_directionToHome;        // direction to home or hol point in degrees
uint32_t aux_GPS_distanceTraveledInCm;     // distance flown since armed in centimeters
int16_t aux_GPS_verticalSpeedInCmS;     // vertical speed in cm/s
float aux_dTnav;             // Delta Time in milliseconds for navigation computations, updated with every good GPS read

#define GPS_DISTANCE_FLOWN_MIN_SPEED_THRESHOLD_CM_S 15 // 5.4Km/h 3.35mph

gpsSolutionData_t auxGpsSol;
uint32_t aux_GPS_packetCount = 0;
uint32_t aux_GPS_svInfoReceivedCount = 0; // SV = Space Vehicle, counter increments each time SV info is received.
uint8_t aux_GPS_update = 0;             // toogle to distinct a GPS position update (directly or via MSP)

uint8_t aux_GPS_numCh;                          // Number of channels
uint8_t aux_GPS_svinfo_chn[GPS_SV_MAXSATS];     // Channel number
uint8_t aux_GPS_svinfo_svid[GPS_SV_MAXSATS];    // Satellite ID
uint8_t aux_GPS_svinfo_quality[GPS_SV_MAXSATS]; // Bitfield Qualtity
uint8_t aux_GPS_svinfo_cno[GPS_SV_MAXSATS];     // Carrier to Noise Ratio (Signal Strength)

// GPS timeout for wrong baud rate/disconnection/etc in milliseconds (default 2.5second)
#define GPS_TIMEOUT (2500)
// How many entries in auxGpsInitData array below
#define GPS_INIT_ENTRIES (GPS_BAUDRATE_MAX + 1)
#define GPS_BAUDRATE_CHANGE_DELAY (200)
// Timeout for waiting ACK/NAK in GPS task cycles (0.1s at 100Hz)
#define UBLOX_ACK_TIMEOUT_MAX_COUNT (10)

static serialPort_t *auxGpsPort;

typedef struct auxGpsInitData_s {
    uint8_t index;
    uint8_t baudrateIndex; // see baudRate_e
    const char *ubx;
    const char *mtk;
} auxGpsInitData_t;

// NMEA will cycle through these until valid data is received
static const auxGpsInitData_t auxGpsInitData[] = {
    { GPS_BAUDRATE_115200,  BAUD_115200, "$PUBX,41,1,0003,0001,115200,0*1E\r\n", "$PMTK251,115200*1F\r\n" },
    { GPS_BAUDRATE_57600,    BAUD_57600, "$PUBX,41,1,0003,0001,57600,0*2D\r\n", "$PMTK251,57600*2C\r\n" },
    { GPS_BAUDRATE_38400,    BAUD_38400, "$PUBX,41,1,0003,0001,38400,0*26\r\n", "$PMTK251,38400*27\r\n" },
    { GPS_BAUDRATE_19200,    BAUD_19200, "$PUBX,41,1,0003,0001,19200,0*23\r\n", "$PMTK251,19200*22\r\n" },
    // 9600 is not enough for 5Hz updates - leave for compatibility to dumb NMEA that only runs at this speed
    { GPS_BAUDRATE_9600,      BAUD_9600, "$PUBX,41,1,0003,0001,9600,0*16\r\n", "" }
};

#define GPS_INIT_DATA_ENTRY_COUNT (sizeof(auxGpsInitData) / sizeof(auxGpsInitData[0]))

#define DEFAULT_BAUD_RATE_INDEX 0

#ifdef USE_GPS_UBLOX
static const uint8_t ubloxInit[] = {
    //Preprocessor Pedestrian Dynamic Platform Model Option
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x03, 0x03, 0x00,           // CFG-NAV5 - Set engine settings
    0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00,           // Collected by resetting a GPS unit to defaults. Changing mode to Pedestrian and
    0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00,           // capturing the data from the U-Center binary console.
    0x00, 0xC8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17, 0xC2,

    // DISABLE NMEA messages
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x05, 0x00, 0xFF, 0x19,           // VGS: Course over ground and Ground speed
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x03, 0x00, 0xFD, 0x15,           // GSV: GNSS Satellites in View
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x01, 0x00, 0xFB, 0x11,           // GLL: Latitude and longitude, with time of position fix and status
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x00, 0x00, 0xFA, 0x0F,           // GGA: Global positioning system fix data
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x02, 0x00, 0xFC, 0x13,           // GSA: GNSS DOP and Active Satellites
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x04, 0x00, 0xFE, 0x17,           // RMC: Recommended Minimum data

    // Enable UBLOX messages
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x02, 0x01, 0x0E, 0x47,           // set POSLLH MSG rate
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x03, 0x01, 0x0F, 0x49,           // set STATUS MSG rate
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x06, 0x01, 0x12, 0x4F,           // set SOL MSG rate
    //0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x30, 0x01, 0x3C, 0xA3,           // set SVINFO MSG rate (every cycle - high bandwidth)
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x30, 0x05, 0x40, 0xA7,           // set SVINFO MSG rate (evey 5 cycles - low bandwidth)
    0xB5, 0x62, 0x06, 0x01, 0x03, 0x00, 0x01, 0x12, 0x01, 0x1E, 0x67,           // set VELNED MSG rate

    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A,             // set rate to 5Hz (measurement period: 200ms, navigation rate: 1 cycle)
};

static const uint8_t ubloxAirborne[] = {
    //Preprocessor Airborne_1g Dynamic Platform Model Option
    #if defined(GPS_UBLOX_MODE_AIRBORNE_1G)
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00,           // CFG-NAV5 - Set engine settings
    0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00,           // Collected by resetting a GPS unit to defaults. Changing mode to Airborne with
    0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00,           // <1g acceleration and capturing the data from the U-Center binary console.
    0x00, 0xC8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1A, 0x28,

    //Default Airborne_4g Dynamic Platform Model
    #else
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x08, 0x03, 0x00,           // CFG-NAV5 - Set engine settings
    0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00,           // Collected by resetting a GPS unit to defaults. Changing mode to Airborne with
    0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x3C, 0x00, 0x00, 0x00,           // <4g acceleration and capturing the data from the U-Center binary console.
    0x00, 0xC8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1C, 0x6C,
    #endif
};

typedef struct {
    uint8_t preamble1;
    uint8_t preamble2;
    uint8_t msg_class;
    uint8_t msg_id;
    uint16_t length;
} ubx_header;

typedef struct {
    uint8_t mode;
    uint8_t usage;
    uint8_t maxSBAS;
    uint8_t scanmode2;
    uint32_t scanmode1;
} ubx_sbas;

typedef struct {
    uint8_t gnssId;
    uint8_t resTrkCh;
    uint8_t maxTrkCh;
    uint8_t reserved1;
    uint32_t flags;
} ubx_configblock;

typedef struct {
    uint8_t msgVer;
    uint8_t numTrkChHw;
    uint8_t numTrkChUse;
    uint8_t numConfigBlocks;
    ubx_configblock configblocks[7];
} ubx_gnss;

typedef union {
    ubx_sbas sbas;
    ubx_gnss gnss;
} ubx_payload;

typedef struct {
    ubx_header header;
    ubx_payload payload;
} __attribute__((packed)) ubx_message;

#define UBLOX_MODE_ENABLED    0x1
#define UBLOX_MODE_TEST       0x2

#define UBLOX_USAGE_RANGE     0x1
#define UBLOX_USAGE_DIFFCORR  0x2
#define UBLOX_USAGE_INTEGRITY 0x4

#define UBLOX_GNSS_ENABLE     0x1
#define UBLOX_GNSS_DEFAULT_SIGCFGMASK 0x10000

#define UBLOX_SBAS_MESSAGE_LENGTH 14
#define UBLOX_GNSS_MESSAGE_LENGTH 66

#endif // USE_GPS_UBLOX

typedef enum {
    GPS_UNKNOWN,
    GPS_INITIALIZING,
    GPS_INITIALIZED,
    GPS_CHANGE_BAUD,
    GPS_CONFIGURE,
    GPS_RECEIVING_DATA,
    GPS_LOST_COMMUNICATION
} gpsState_e;

gpsData_t auxGpsData;


PG_REGISTER_WITH_RESET_TEMPLATE(auxGpsConfig_t, auxGpsConfig, PG_AUX_GPS_CONFIG, 0);

PG_RESET_TEMPLATE(auxGpsConfig_t, auxGpsConfig,
    .provider = GPS_NMEA,
    .sbasMode = SBAS_NONE,
    .autoConfig = GPS_AUTOCONFIG_ON,
    .autoBaud = GPS_AUTOBAUD_OFF,
    .gps_ublox_use_galileo = false,
    .gps_ublox_mode = UBLOX_AIRBORNE,
    .gps_set_home_point_once = false,
    .gps_use_3d_speed = false,
    .sbas_integrity = false
);

static void shiftPacketLog(void)
{
    uint32_t i;

    for (i = ARRAYLEN(auxGpsPacketLog) - 1; i > 0 ; i--) {
        auxGpsPacketLog[i] = auxGpsPacketLog[i-1];
    }
}

static void auxGpsNewData(uint16_t c);
#ifdef USE_GPS_NMEA
static bool auxGpsNewFrameNMEA(char c);
#endif
#ifdef USE_GPS_UBLOX
static bool auxGpsNewFrameUBLOX(uint8_t data);
#endif

static void auxGpsSetState(gpsState_e state)
{
    auxGpsData.state = state;
    auxGpsData.state_position = 0;
    auxGpsData.state_ts = millis();
    auxGpsData.messageState = GPS_MESSAGE_STATE_IDLE;
}

void auxGpsInit(void)
{
    auxGpsData.baudrateIndex = 0;
    auxGpsData.errors = 0;
    auxGpsData.timeouts = 0;

    memset(auxGpsPacketLog, 0x00, sizeof(auxGpsPacketLog));

    // init auxGpsData structure. if we're not actually enabled, don't bother doing anything else
    auxGpsSetState(GPS_UNKNOWN);

    auxGpsData.lastMessage = millis();

    if (auxGpsConfig()->provider == GPS_MSP) { // no serial ports used when GPS_MSP is configured
        auxGpsSetState(GPS_INITIALIZED);
        return;
    }

    const serialPortConfig_t *auxGpsPortConfig = findSerialPortConfig(FUNCTION_AUX_GPS);
    if (!auxGpsPortConfig) {
        return;
    }

    while (auxGpsInitData[auxGpsData.baudrateIndex].baudrateIndex != auxGpsPortConfig->gps_baudrateIndex) {
        auxGpsData.baudrateIndex++;
        if (auxGpsData.baudrateIndex >= GPS_INIT_DATA_ENTRY_COUNT) {
            auxGpsData.baudrateIndex = DEFAULT_BAUD_RATE_INDEX;
            break;
        }
    }

    portMode_e mode = MODE_RXTX;
#if defined(GPS_NMEA_TX_ONLY)
    if (auxGpsConfig()->provider == GPS_NMEA) {
        mode &= ~MODE_TX;
    }
#endif

    // no callback - buffer will be consumed in auxGpsUpdate()
    auxGpsPort = openSerialPort(auxGpsPortConfig->identifier, FUNCTION_AUX_GPS, NULL, NULL, baudRates[auxGpsInitData[auxGpsData.baudrateIndex].baudrateIndex], mode, SERIAL_NOT_INVERTED);
    if (!auxGpsPort) {
        return;
    }

    // signal GPS "thread" to initialize when it gets to it
    auxGpsSetState(GPS_INITIALIZING);
}

#ifdef USE_GPS_NMEA
void auxGpsInitNmea(void)
{
#if !defined(GPS_NMEA_TX_ONLY)
    uint32_t now;
#endif
    switch (auxGpsData.state) {
        case GPS_INITIALIZING:
#if !defined(GPS_NMEA_TX_ONLY)
           now = millis();
           if (now - auxGpsData.state_ts < 1000) {
               return;
           }
           auxGpsData.state_ts = now;
           if (auxGpsData.state_position < 1) {
               serialSetBaudRate(auxGpsPort, 4800);
               auxGpsData.state_position++;
           } else if (auxGpsData.state_position < 2) {
               // print our FIXED init string for the baudrate we want to be at
               serialPrint(auxGpsPort, "$PSRF100,1,115200,8,1,0*05\r\n");
               auxGpsData.state_position++;
           } else {
               // we're now (hopefully) at the correct rate, next state will switch to it
               auxGpsSetState(GPS_CHANGE_BAUD);
           }
           break;
#endif
        case GPS_CHANGE_BAUD:
#if !defined(GPS_NMEA_TX_ONLY)
           now = millis();
           if (now - auxGpsData.state_ts < 1000) {
               return;
           }
           auxGpsData.state_ts = now;
           if (auxGpsData.state_position < 1) {
               serialSetBaudRate(auxGpsPort, baudRates[auxGpsInitData[auxGpsData.baudrateIndex].baudrateIndex]);
               auxGpsData.state_position++;
           } else if (auxGpsData.state_position < 2) {
               serialPrint(auxGpsPort, "$PSRF103,00,6,00,0*23\r\n");
               auxGpsData.state_position++;
           } else
#else
           {
               serialSetBaudRate(auxGpsPort, baudRates[auxGpsInitData[auxGpsData.baudrateIndex].baudrateIndex]);
           }
#endif
               auxGpsSetState(GPS_RECEIVING_DATA);
            break;
    }
}
#endif // USE_GPS_NMEA

#ifdef USE_GPS_UBLOX
static void ubloxSendByteUpdateChecksum(const uint8_t data, uint8_t *checksumA, uint8_t *checksumB)
{
    *checksumA += data;
    *checksumB += *checksumA;
    serialWrite(auxGpsPort, data);
}

static void ubloxSendDataUpdateChecksum(const uint8_t *data, uint8_t len, uint8_t *checksumA, uint8_t *checksumB)
{
    while (len--) {
        ubloxSendByteUpdateChecksum(*data, checksumA, checksumB);
        data++;
    }
}

static void ubloxSendConfigMessage(const uint8_t *data, uint8_t len)
{
    uint8_t checksumA = 0, checksumB = 0;
    serialWrite(auxGpsPort, data[0]);
    serialWrite(auxGpsPort, data[1]);
    ubloxSendDataUpdateChecksum(&data[2], len-2, &checksumA, &checksumB);
    serialWrite(auxGpsPort, checksumA);
    serialWrite(auxGpsPort, checksumB);

    // Save state for ACK waiting
    auxGpsData.ackWaitingMsgId = data[3]; //save message id for ACK
    auxGpsData.ackTimeoutCounter = 0;
    auxGpsData.ackState = UBLOX_ACK_WAITING;
}

void auxGpsInitUblox(void)
{
    uint32_t now;
    // UBX will run at the serial port's baudrate, it shouldn't be "autodetected". So here we force it to that rate

    // Wait until GPS transmit buffer is empty
    if (!isSerialTransmitBufferEmpty(auxGpsPort))
        return;


    switch (auxGpsData.state) {
        case GPS_INITIALIZING:
            now = millis();
            if (now - auxGpsData.state_ts < GPS_BAUDRATE_CHANGE_DELAY)
                return;

            if (auxGpsData.state_position < GPS_INIT_ENTRIES) {
                // try different speed to INIT
                baudRate_e newBaudRateIndex = auxGpsInitData[auxGpsData.state_position].baudrateIndex;

                auxGpsData.state_ts = now;

                if (lookupBaudRateIndex(serialGetBaudRate(auxGpsPort)) != newBaudRateIndex) {
                    // change the rate if needed and wait a little
                    serialSetBaudRate(auxGpsPort, baudRates[newBaudRateIndex]);
                    return;
                }

                // print our FIXED init string for the baudrate we want to be at
                serialPrint(auxGpsPort, auxGpsInitData[auxGpsData.baudrateIndex].ubx);

                auxGpsData.state_position++;
            } else {
                // we're now (hopefully) at the correct rate, next state will switch to it
                auxGpsSetState(GPS_CHANGE_BAUD);
            }
            break;
        case GPS_CHANGE_BAUD:
            serialSetBaudRate(auxGpsPort, baudRates[auxGpsInitData[auxGpsData.baudrateIndex].baudrateIndex]);
            auxGpsSetState(GPS_CONFIGURE);
            break;
        case GPS_CONFIGURE:

            // Either use specific config file for GPS or let dynamically upload config
            if ( auxGpsConfig()->autoConfig == GPS_AUTOCONFIG_OFF ) {
                auxGpsSetState(GPS_RECEIVING_DATA);
                break;
            }

            if (auxGpsData.messageState == GPS_MESSAGE_STATE_IDLE) {
                auxGpsData.messageState++;
            }

            if (auxGpsData.messageState == GPS_MESSAGE_STATE_INIT) {
                if (auxGpsData.state_position < sizeof(ubloxInit)) {
                    if (auxGpsData.state_position < sizeof(ubloxAirborne)) {
                        if (auxGpsConfig()->gps_ublox_mode == UBLOX_AIRBORNE) {
                            serialWrite(auxGpsPort, ubloxAirborne[auxGpsData.state_position]);
                        } else {
                            serialWrite(auxGpsPort, ubloxInit[auxGpsData.state_position]);
                        }
                    } else {
                        serialWrite(auxGpsPort, ubloxInit[auxGpsData.state_position]);
                    }
                    auxGpsData.state_position++;
                } else {
                    auxGpsData.state_position = 0;
                    auxGpsData.messageState++;
                    auxGpsData.ackState = UBLOX_ACK_IDLE;
                }
            }

            if (auxGpsData.messageState == GPS_MESSAGE_STATE_SBAS) {
                switch (auxGpsData.ackState) {
                    case UBLOX_ACK_IDLE:
                        {
                            ubx_message tx_buffer;
                            tx_buffer.header.preamble1 = 0xB5;
                            tx_buffer.header.preamble2 = 0x62;
                            tx_buffer.header.msg_class = 0x06;
                            tx_buffer.header.msg_id = 0x16;
                            tx_buffer.header.length = 8;

                            //NOTE: default ublox config for sbas mode is: UBLOX_MODE_ENABLED, test is disabled
                            tx_buffer.payload.sbas.mode = UBLOX_MODE_TEST;
                            if (auxGpsConfig()->sbasMode != SBAS_NONE) {
                                tx_buffer.payload.sbas.mode |= UBLOX_MODE_ENABLED;
                            }

                            //NOTE: default ublox config for sbas mode is: UBLOX_USAGE_RANGE | UBLOX_USAGE_DIFFCORR, integrity is disabled
                            tx_buffer.payload.sbas.usage = UBLOX_USAGE_RANGE | UBLOX_USAGE_DIFFCORR;
                            if (auxGpsConfig()->sbas_integrity) {
                                tx_buffer.payload.sbas.usage |= UBLOX_USAGE_INTEGRITY;
                            }

                            tx_buffer.payload.sbas.maxSBAS = 3;
                            tx_buffer.payload.sbas.scanmode2 = 0;
                            switch (auxGpsConfig()->sbasMode) {
                                case SBAS_AUTO:
                                    tx_buffer.payload.sbas.scanmode1 = 0; 
                                    break;
                                case SBAS_EGNOS:
                                    tx_buffer.payload.sbas.scanmode1 = 0x00010048; //PRN123, PRN126, PRN136
                                    break;
                                case SBAS_WAAS:
                                    tx_buffer.payload.sbas.scanmode1 = 0x0004A800; //PRN131, PRN133, PRN135, PRN138
                                    break;
                                case SBAS_MSAS:
                                    tx_buffer.payload.sbas.scanmode1 = 0x00020200; //PRN129, PRN137
                                    break;
                                case SBAS_GAGAN:
                                    tx_buffer.payload.sbas.scanmode1 = 0x00001180; //PRN127, PRN128, PRN132
                                    break;
                                default:
                                    tx_buffer.payload.sbas.scanmode1 = 0; 
                                    break;
                            }
                            ubloxSendConfigMessage((const uint8_t *) &tx_buffer, UBLOX_SBAS_MESSAGE_LENGTH);
                        }
                        break;
                    case UBLOX_ACK_WAITING:
                        if ((++auxGpsData.ackTimeoutCounter) == UBLOX_ACK_TIMEOUT_MAX_COUNT) {
                            auxGpsData.ackState = UBLOX_ACK_GOT_TIMEOUT;
                        }
                        break;
                    case UBLOX_ACK_GOT_TIMEOUT:
                    case UBLOX_ACK_GOT_NACK:
                    case UBLOX_ACK_GOT_ACK:
                        auxGpsData.state_position = 0;
                        auxGpsData.ackState = UBLOX_ACK_IDLE;
                        auxGpsData.messageState++;
                        break;
                    default:
                        break;
                }
            }

            if (auxGpsData.messageState == GPS_MESSAGE_STATE_GNSS) {
                switch (auxGpsData.ackState) {
                    case UBLOX_ACK_IDLE:
                        {
                            ubx_message tx_buffer;
                            tx_buffer.header.preamble1 = 0xB5;
                            tx_buffer.header.preamble2 = 0x62;
                            tx_buffer.header.msg_class = 0x06;
                            tx_buffer.header.msg_id = 0x3E;
                            tx_buffer.header.length = 60;

                            tx_buffer.payload.gnss.msgVer = 0;
                            tx_buffer.payload.gnss.numTrkChHw = 32;
                            tx_buffer.payload.gnss.numTrkChUse = 32;
                            tx_buffer.payload.gnss.numConfigBlocks = 7;

                            tx_buffer.payload.gnss.configblocks[0].gnssId = 0;    //GPS
                            tx_buffer.payload.gnss.configblocks[0].resTrkCh = 8;  //min channels
                            tx_buffer.payload.gnss.configblocks[0].maxTrkCh = 16; //max channels
                            tx_buffer.payload.gnss.configblocks[0].reserved1 = 0;
                            tx_buffer.payload.gnss.configblocks[0].flags = UBLOX_GNSS_ENABLE | UBLOX_GNSS_DEFAULT_SIGCFGMASK | 0x01000000; //last number is undocumented and was captured from uCenter

                            tx_buffer.payload.gnss.configblocks[1].gnssId = 1;    //SBAS
                            tx_buffer.payload.gnss.configblocks[1].resTrkCh = 1;  //min channels
                            tx_buffer.payload.gnss.configblocks[1].maxTrkCh = 3;  //max channels
                            tx_buffer.payload.gnss.configblocks[1].reserved1 = 0;
                            tx_buffer.payload.gnss.configblocks[1].flags = UBLOX_GNSS_DEFAULT_SIGCFGMASK | 0x01000000; //last number is undocumented and was captured from uCenter
                            if (auxGpsConfig()->sbasMode != SBAS_NONE) {
                                tx_buffer.payload.gnss.configblocks[1].flags |= UBLOX_GNSS_ENABLE;
                            }

                            tx_buffer.payload.gnss.configblocks[2].gnssId = 2;    //Galileo
                            tx_buffer.payload.gnss.configblocks[2].resTrkCh = 4;  //min channels
                            tx_buffer.payload.gnss.configblocks[2].maxTrkCh = 8;  //max channels
                            tx_buffer.payload.gnss.configblocks[2].reserved1 = 0;
                            tx_buffer.payload.gnss.configblocks[2].flags = UBLOX_GNSS_DEFAULT_SIGCFGMASK | 0x01000000; //last number is undocumented and was captured from uCenter
                            if (auxGpsConfig()->gps_ublox_use_galileo) {
                                tx_buffer.payload.gnss.configblocks[2].flags |= UBLOX_GNSS_ENABLE;
                            }

                            tx_buffer.payload.gnss.configblocks[3].gnssId = 3;    //BeiDou
                            tx_buffer.payload.gnss.configblocks[3].resTrkCh = 8;  //min channels
                            tx_buffer.payload.gnss.configblocks[3].maxTrkCh = 16;  //max channels
                            tx_buffer.payload.gnss.configblocks[3].reserved1 = 0;
                            tx_buffer.payload.gnss.configblocks[3].flags = UBLOX_GNSS_DEFAULT_SIGCFGMASK | 0x01000000; //last number is undocumented and was captured from uCenter

                            tx_buffer.payload.gnss.configblocks[4].gnssId = 4;    //IMES
                            tx_buffer.payload.gnss.configblocks[4].resTrkCh = 0;  //min channels
                            tx_buffer.payload.gnss.configblocks[4].maxTrkCh = 8;  //max channels
                            tx_buffer.payload.gnss.configblocks[4].reserved1 = 0;
                            tx_buffer.payload.gnss.configblocks[4].flags = UBLOX_GNSS_DEFAULT_SIGCFGMASK | 0x03000000; //last number is undocumented and was captured from uCenter

                            tx_buffer.payload.gnss.configblocks[5].gnssId = 5;    //QZSS
                            tx_buffer.payload.gnss.configblocks[5].resTrkCh = 0;  //min channels
                            tx_buffer.payload.gnss.configblocks[5].maxTrkCh = 3;  //max channels
                            tx_buffer.payload.gnss.configblocks[5].reserved1 = 0;
                            tx_buffer.payload.gnss.configblocks[5].flags = UBLOX_GNSS_DEFAULT_SIGCFGMASK | 0x05000000; //last number is undocumented and was captured from uCenter
                            if (!auxGpsConfig()->gps_ublox_use_galileo) {
                                tx_buffer.payload.gnss.configblocks[5].flags |= UBLOX_GNSS_ENABLE;
                            }

                            tx_buffer.payload.gnss.configblocks[6].gnssId = 6;    //GLONASS
                            tx_buffer.payload.gnss.configblocks[6].resTrkCh = 8;  //min channels
                            tx_buffer.payload.gnss.configblocks[6].maxTrkCh = 14; //max channels
                            tx_buffer.payload.gnss.configblocks[6].reserved1 = 0;
                            tx_buffer.payload.gnss.configblocks[6].flags = UBLOX_GNSS_ENABLE | UBLOX_GNSS_DEFAULT_SIGCFGMASK | 0x01000000; //last number is undocumented and was captured from uCenter
                            
                            ubloxSendConfigMessage((const uint8_t *) &tx_buffer, UBLOX_GNSS_MESSAGE_LENGTH);
                        }
                        break;
                    case UBLOX_ACK_WAITING:
                        if ((++auxGpsData.ackTimeoutCounter) == UBLOX_ACK_TIMEOUT_MAX_COUNT) {
                            auxGpsData.ackState = UBLOX_ACK_GOT_TIMEOUT;
                        }
                        break;
                    case UBLOX_ACK_GOT_TIMEOUT:
                    case UBLOX_ACK_GOT_NACK:
                    case UBLOX_ACK_GOT_ACK:
                        auxGpsData.state_position = 0;
                        auxGpsData.ackState = UBLOX_ACK_IDLE;
                        auxGpsData.messageState++;
                        break;
                    default:
                        break;
                }
            }

            if (auxGpsData.messageState >= GPS_MESSAGE_STATE_INITIALIZED) {
                // ublox should be initialised, try receiving
                auxGpsSetState(GPS_RECEIVING_DATA);
            }
            break;
    }
}
#endif // USE_GPS_UBLOX

void auxGpsInitHardware(void)
{
    switch (auxGpsConfig()->provider) {
    case GPS_NMEA:
#ifdef USE_GPS_NMEA
        auxGpsInitNmea();
#endif
        break;

    case GPS_UBLOX:
#ifdef USE_GPS_UBLOX
        auxGpsInitUblox();
#endif
        break;
    default:
        break;
    }
}

static void updateAuxGpsIndicator(timeUs_t currentTimeUs)
{
    static uint32_t GPSLEDTime;
    if ((int32_t)(currentTimeUs - GPSLEDTime) >= 0 && (auxGpsSol.numSat >= 5)) {
        GPSLEDTime = currentTimeUs + 150000;
        LED2_TOGGLE;
    }
}

void auxGpsUpdate(timeUs_t currentTimeUs)
{
    // read out available GPS bytes
    if (auxGpsPort) {
        while (serialRxBytesWaiting(auxGpsPort))
            auxGpsNewData(serialRead(auxGpsPort));
    } else if (aux_GPS_update & GPS_MSP_UPDATE) { // GPS data received via MSP
        auxGpsSetState(GPS_RECEIVING_DATA);
        auxGpsData.lastMessage = millis();
        sensorsSet(SENSOR_AUX_GPS);
        onAuxGpsNewData();
        aux_GPS_update &= ~GPS_MSP_UPDATE;
    }

    switch (auxGpsData.state) {
        case GPS_UNKNOWN:
        case GPS_INITIALIZED:
            break;

        case GPS_INITIALIZING:
        case GPS_CHANGE_BAUD:
        case GPS_CONFIGURE:
            auxGpsInitHardware();
            break;

        case GPS_LOST_COMMUNICATION:
            auxGpsData.timeouts++;
            if (auxGpsConfig()->autoBaud) {
                // try another rate
                auxGpsData.baudrateIndex++;
                auxGpsData.baudrateIndex %= GPS_INIT_ENTRIES;
            }
            auxGpsData.lastMessage = millis();
            auxGpsSol.numSat = 0;
            DISABLE_STATE(AUX_GPS_FIX);
            auxGpsSetState(GPS_INITIALIZING);
            break;

        case GPS_RECEIVING_DATA:
            // check for no data/gps timeout/cable disconnection etc
            if (millis() - auxGpsData.lastMessage > GPS_TIMEOUT) {
                // remove GPS from capability
                sensorsClear(SENSOR_AUX_GPS);
                auxGpsSetState(GPS_LOST_COMMUNICATION);
#ifdef USE_GPS_UBLOX
            } else {
                if (auxGpsConfig()->autoConfig == GPS_AUTOCONFIG_ON) { // Only if autoconfig is enabled
                    if ((auxGpsData.messageState == GPS_MESSAGE_STATE_INITIALIZED) && STATE(AUX_GPS_FIX) && (auxGpsConfig()->gps_ublox_mode == UBLOX_DYNAMIC)) {
                        auxGpsData.messageState = GPS_MESSAGE_STATE_PEDESTRIAN_TO_AIRBORNE;
                        auxGpsData.state_position = 0;
                    }
                    if (auxGpsData.messageState == GPS_MESSAGE_STATE_PEDESTRIAN_TO_AIRBORNE) {
                        if (auxGpsData.state_position < sizeof(ubloxAirborne)) {
                            if (isSerialTransmitBufferEmpty(auxGpsPort)) {
                                serialWrite(auxGpsPort, ubloxAirborne[auxGpsData.state_position]);
                                auxGpsData.state_position++;
                            }
                        } else {
                            auxGpsData.messageState = GPS_MESSAGE_STATE_ENTRY_COUNT;
                        }
                    }
                }
#endif //USE_GPS_UBLOX
            }
            break;
    }
    if (sensors(SENSOR_AUX_GPS)) {
        updateAuxGpsIndicator(currentTimeUs);
    }
    if (!ARMING_FLAG(ARMED) && !auxGpsConfig()->gps_set_home_point_once) {
        DISABLE_STATE(AUX_GPS_FIX_HOME);
    }
// #if defined(USE_GPS_RESCUE)
//     if (gpsRescueIsConfigured()) {
//         updateGPSRescueState();
//     }
// #endif
#if defined(USE_GPS_FOLLOW)
    if (gpsFollowIsConfigured()) {
        updateGPSFollowState();
    }
#endif
}

static void auxGpsNewData(uint16_t c)
{
    if (!auxGpsNewFrame(c)) {
        return;
    }

    // new data received and parsed, we're in business
    auxGpsData.lastLastMessage = auxGpsData.lastMessage;
    auxGpsData.lastMessage = millis();
    sensorsSet(SENSOR_AUX_GPS);

    aux_GPS_update ^= GPS_DIRECT_TICK;

#if 0
    debug[3] = aux_GPS_update;
#endif

    onAuxGpsNewData();
}

bool auxGpsNewFrame(uint8_t c)
{
    switch (auxGpsConfig()->provider) {
    case GPS_NMEA:          // NMEA
#ifdef USE_GPS_NMEA
        return auxGpsNewFrameNMEA(c);
#endif
        break;
    case GPS_UBLOX:         // UBX binary
#ifdef USE_GPS_UBLOX
        return auxGpsNewFrameUBLOX(c);
#endif
        break;
    default:
        break;
    }
    return false;
}

// Check for healthy communications
bool auxGpsIsHealthy()
{
    return (auxGpsData.state == GPS_RECEIVING_DATA);
}

/* This is a light implementation of a GPS frame decoding
   This should work with most of modern GPS devices configured to output 5 frames.
   It assumes there are some NMEA GGA frames to decode on the serial bus
   Now verifies checksum correctly before applying data

   Here we use only the following data :
     - latitude
     - longitude
     - GPS fix is/is not ok
     - GPS num sat (4 is enough to be +/- reliable)
     // added by Mis
     - GPS altitude (for OSD displaying)
     - GPS speed (for OSD displaying)
*/

#define NO_FRAME   0
#define FRAME_GGA  1
#define FRAME_RMC  2
#define FRAME_GSV  3


// This code is used for parsing NMEA data

/* Alex optimization
  The latitude or longitude is coded this way in NMEA frames
  dm.f   coded as degrees + minutes + minute decimal
  Where:
    - d can be 1 or more char long. generally: 2 char long for latitude, 3 char long for longitude
    - m is always 2 char long
    - f can be 1 or more char long
  This function converts this format in a unique unsigned long where 1 degree = 10 000 000

  EOS increased the precision here, even if we think that the gps is not precise enough, with 10e5 precision it has 76cm resolution
  with 10e7 it's around 1 cm now. Increasing it further is irrelevant, since even 1cm resolution is unrealistic, however increased
  resolution also increased precision of nav calculations
static uint32_t GPS_coord_to_degrees(char *coordinateString)
{
    char *p = s, *d = s;
    uint8_t min, deg = 0;
    uint16_t frac = 0, mult = 10000;

    while (*p) {                // parse the string until its end
        if (d != s) {
            frac += (*p - '0') * mult;  // calculate only fractional part on up to 5 digits  (d != s condition is true when the . is located)
            mult /= 10;
        }
        if (*p == '.')
            d = p;              // locate '.' char in the string
        p++;
    }
    if (p == s)
        return 0;
    while (s < d - 2) {
        deg *= 10;              // convert degrees : all chars before minutes ; for the first iteration, deg = 0
        deg += *(s++) - '0';
    }
    min = *(d - 1) - '0' + (*(d - 2) - '0') * 10;       // convert minutes : 2 previous char before '.'
    return deg * 10000000UL + (min * 100000UL + frac) * 10UL / 6;
}
*/

// helper functions
#ifdef USE_GPS_NMEA
static uint32_t grab_fields(char *src, uint8_t mult)
{                               // convert string to uint32
    uint32_t i;
    uint32_t tmp = 0;
    int isneg = 0;
    for (i = 0; src[i] != 0; i++) {
        if ((i == 0) && (src[0] == '-')) { // detect negative sign
            isneg = 1;
            continue; // jump to next character if the first one was a negative sign
        }
        if (src[i] == '.') {
            i++;
            if (mult == 0) {
                break;
            } else {
                src[i + mult] = 0;
            }
        }
        tmp *= 10;
        if (src[i] >= '0' && src[i] <= '9') {
            tmp += src[i] - '0';
        }
        if (i >= 15) {
            return 0; // out of bounds
        }
    }
    return isneg ? -tmp : tmp;    // handle negative altitudes
}

typedef struct auxGpsDataNmea_s {
    int32_t latitude;
    int32_t longitude;
    uint8_t numSat;
    int32_t altitudeCm;
    uint16_t speed;
    uint16_t hdop;
    uint16_t ground_course;
    uint32_t time;
    uint32_t date;
} auxGpsDataNmea_t;

static bool auxGpsNewFrameNMEA(char c)
{
    static auxGpsDataNmea_t gps_Msg;

    uint8_t frameOK = 0;
    static uint8_t param = 0, offset = 0, parity = 0;
    static char string[15];
    static uint8_t checksum_param, gps_frame = NO_FRAME;
    static uint8_t svMessageNum = 0;
    uint8_t svSatNum = 0, svPacketIdx = 0, svSatParam = 0;

    switch (c) {
        case '$':
            param = 0;
            offset = 0;
            parity = 0;
            break;
        case ',':
        case '*':
            string[offset] = 0;
            if (param == 0) {       //frame identification
                gps_frame = NO_FRAME;
                if (0 == strcmp(string, "GPGGA") || 0 == strcmp(string, "GNGGA")) {
                    gps_frame = FRAME_GGA;
                } else if (0 == strcmp(string, "GPRMC") || 0 == strcmp(string, "GNRMC")) {
                    gps_frame = FRAME_RMC;
                } else if (0 == strcmp(string, "GPGSV")) {
                    gps_frame = FRAME_GSV;
                }
            }

            switch (gps_frame) {
                case FRAME_GGA:        //************* GPGGA FRAME parsing
                    switch (param) {
            //          case 1:             // Time information
            //              break;
                        case 2:
                            gps_Msg.latitude = GPS_coord_to_degrees(string);
                            break;
                        case 3:
                            if (string[0] == 'S')
                                gps_Msg.latitude *= -1;
                            break;
                        case 4:
                            gps_Msg.longitude = GPS_coord_to_degrees(string);
                            break;
                        case 5:
                            if (string[0] == 'W')
                                gps_Msg.longitude *= -1;
                            break;
                        case 6:
                            if (string[0] > '0') {
                                ENABLE_STATE(AUX_GPS_FIX);
                            } else {
                                DISABLE_STATE(AUX_GPS_FIX);
                            }
                            break;
                        case 7:
                            gps_Msg.numSat = grab_fields(string, 0);
                            break;
                        case 8:
                            gps_Msg.hdop = grab_fields(string, 1) * 100;          // hdop
                            break;
                        case 9:
                            gps_Msg.altitudeCm = grab_fields(string, 1) * 10;     // altitude in centimeters. Note: NMEA delivers altitude with 1 or 3 decimals. It's safer to cut at 0.1m and multiply by 10
                            break;
                    }
                    break;
                case FRAME_RMC:        //************* GPRMC FRAME parsing
                    switch (param) {
                        case 1:
                            gps_Msg.time = grab_fields(string, 2); // UTC time hhmmss.ss
                            break;
                        case 7:
                            gps_Msg.speed = ((grab_fields(string, 1) * 5144L) / 1000L);    // speed in cm/s added by Mis
                            break;
                        case 8:
                            gps_Msg.ground_course = (grab_fields(string, 1));      // ground course deg * 10
                            break;
                        case 9:
                            gps_Msg.date = grab_fields(string, 0); // date dd/mm/yy
                            break;
                    }
                    break;
                case FRAME_GSV:
                    switch (param) {
                      /*case 1:
                            // Total number of messages of this type in this cycle
                            break; */
                        case 2:
                            // Message number
                            svMessageNum = grab_fields(string, 0);
                            break;
                        case 3:
                            // Total number of SVs visible
                            aux_GPS_numCh = grab_fields(string, 0);
                            break;
                    }
                    if (param < 4)
                        break;

                    svPacketIdx = (param - 4) / 4 + 1; // satellite number in packet, 1-4
                    svSatNum    = svPacketIdx + (4 * (svMessageNum - 1)); // global satellite number
                    svSatParam  = param - 3 - (4 * (svPacketIdx - 1)); // parameter number for satellite

                    if (svSatNum > GPS_SV_MAXSATS)
                        break;

                    switch (svSatParam) {
                        case 1:
                            // SV PRN number
                            aux_GPS_svinfo_chn[svSatNum - 1]  = svSatNum;
                            aux_GPS_svinfo_svid[svSatNum - 1] = grab_fields(string, 0);
                            break;
                      /*case 2:
                            // Elevation, in degrees, 90 maximum
                            break;
                        case 3:
                            // Azimuth, degrees from True North, 000 through 359
                            break; */
                        case 4:
                            // SNR, 00 through 99 dB (null when not tracking)
                            aux_GPS_svinfo_cno[svSatNum - 1] = grab_fields(string, 0);
                            aux_GPS_svinfo_quality[svSatNum - 1] = 0; // only used by ublox
                            break;
                    }

                    aux_GPS_svInfoReceivedCount++;

                    break;
            }

            param++;
            offset = 0;
            if (c == '*')
                checksum_param = 1;
            else
                parity ^= c;
            break;
        case '\r':
        case '\n':
            if (checksum_param) {   //parity checksum
                shiftPacketLog();
                uint8_t checksum = 16 * ((string[0] >= 'A') ? string[0] - 'A' + 10 : string[0] - '0') + ((string[1] >= 'A') ? string[1] - 'A' + 10 : string[1] - '0');
                if (checksum == parity) {
                    *auxGpsPacketLogChar = LOG_IGNORED;
                    aux_GPS_packetCount++;
                    switch (gps_frame) {
                    case FRAME_GGA:
                      *auxGpsPacketLogChar = LOG_NMEA_GGA;
                      frameOK = 1;
                      if (STATE(AUX_GPS_FIX)) {
                            auxGpsSol.llh.lat = gps_Msg.latitude;
                            auxGpsSol.llh.lon = gps_Msg.longitude;
                            auxGpsSol.numSat = gps_Msg.numSat;
                            auxGpsSol.llh.altCm = gps_Msg.altitudeCm;
                            auxGpsSol.hdop = gps_Msg.hdop;
                        }
                        break;
                    case FRAME_RMC:
                        *auxGpsPacketLogChar = LOG_NMEA_RMC;
                        auxGpsSol.groundSpeed = gps_Msg.speed;
                        auxGpsSol.groundCourse = gps_Msg.ground_course;
#ifdef USE_RTC_TIME
                        // This check will miss 00:00:00.00, but we shouldn't care - next report will be valid
                        if(!rtcHasTime() && gps_Msg.date != 0 && gps_Msg.time != 0) {
                            dateTime_t temp_time;
                            temp_time.year = (gps_Msg.date % 100) + 2000;
                            temp_time.month = (gps_Msg.date / 100) % 100;
                            temp_time.day = (gps_Msg.date / 10000) % 100;
                            temp_time.hours = (gps_Msg.time / 1000000) % 100;
                            temp_time.minutes = (gps_Msg.time / 10000) % 100;
                            temp_time.seconds = (gps_Msg.time / 100) % 100;
                            temp_time.millis = (gps_Msg.time & 100) * 10;
                            rtcSetDateTime(&temp_time);
                        }
#endif
                        break;
                    } // end switch
                } else {
                    *auxGpsPacketLogChar = LOG_ERROR;
                }
            }
            checksum_param = 0;
            break;
        default:
            if (offset < 15)
                string[offset++] = c;
            if (!checksum_param)
                parity ^= c;
    }
    return frameOK;
}
#endif // USE_GPS_NMEA

#ifdef USE_GPS_UBLOX
// UBX support
typedef struct {
    uint32_t time;              // GPS msToW
    int32_t longitude;
    int32_t latitude;
    int32_t altitude_ellipsoid;
    int32_t altitudeMslMm;
    uint32_t horizontal_accuracy;
    uint32_t vertical_accuracy;
} ubx_nav_posllh;

typedef struct {
    uint32_t time;              // GPS msToW
    uint8_t fix_type;
    uint8_t fix_status;
    uint8_t differential_status;
    uint8_t res;
    uint32_t time_to_first_fix;
    uint32_t uptime;            // milliseconds
} ubx_nav_status;

typedef struct {
    uint32_t time;
    int32_t time_nsec;
    int16_t week;
    uint8_t fix_type;
    uint8_t fix_status;
    int32_t ecef_x;
    int32_t ecef_y;
    int32_t ecef_z;
    uint32_t position_accuracy_3d;
    int32_t ecef_x_velocity;
    int32_t ecef_y_velocity;
    int32_t ecef_z_velocity;
    uint32_t speed_accuracy;
    uint16_t position_DOP;
    uint8_t res;
    uint8_t satellites;
    uint32_t res2;
} ubx_nav_solution;

typedef struct {
    uint32_t time;              // GPS msToW
    int32_t ned_north;
    int32_t ned_east;
    int32_t ned_down;
    uint32_t speed_3d;
    uint32_t speed_2d;
    int32_t heading_2d;
    uint32_t speed_accuracy;
    uint32_t heading_accuracy;
} ubx_nav_velned;

typedef struct {
    uint8_t chn;                // Channel number, 255 for SVx not assigned to channel
    uint8_t svid;               // Satellite ID
    uint8_t flags;              // Bitmask
    uint8_t quality;            // Bitfield
    uint8_t cno;                // Carrier to Noise Ratio (Signal Strength) // dbHz, 0-55.
    uint8_t elev;               // Elevation in integer degrees
    int16_t azim;               // Azimuth in integer degrees
    int32_t prRes;              // Pseudo range residual in centimetres
} ubx_nav_svinfo_channel;

typedef struct {
    uint32_t time;              // GPS Millisecond time of week
    uint8_t numCh;              // Number of channels
    uint8_t globalFlags;        // Bitmask, Chip hardware generation 0:Antaris, 1:u-blox 5, 2:u-blox 6
    uint16_t reserved2;         // Reserved
    ubx_nav_svinfo_channel channel[16];         // 16 satellites * 12 byte
} ubx_nav_svinfo;

typedef struct {
    uint8_t clsId;               // Class ID of the acknowledged message 
    uint8_t msgId;               // Message ID of the acknowledged message
} ubx_ack;

enum {
    PREAMBLE1 = 0xb5,
    PREAMBLE2 = 0x62,
    CLASS_NAV = 0x01,
    CLASS_ACK = 0x05,
    CLASS_CFG = 0x06,
    MSG_ACK_NACK = 0x00,
    MSG_ACK_ACK = 0x01,
    MSG_POSLLH = 0x2,
    MSG_STATUS = 0x3,
    MSG_SOL = 0x6,
    MSG_VELNED = 0x12,
    MSG_SVINFO = 0x30,
    MSG_CFG_PRT = 0x00,
    MSG_CFG_RATE = 0x08,
    MSG_CFG_SET_RATE = 0x01,
    MSG_CFG_NAV_SETTINGS = 0x24
} aux_ubx_protocol_bytes;

enum {
    FIX_NONE = 0,
    FIX_DEAD_RECKONING = 1,
    FIX_2D = 2,
    FIX_3D = 3,
    FIX_GPS_DEAD_RECKONING = 4,
    FIX_TIME = 5
} aux_ubs_nav_fix_type;

enum {
    NAV_STATUS_FIX_VALID = 1,
    NAV_STATUS_TIME_WEEK_VALID = 4,
    NAV_STATUS_TIME_SECOND_VALID = 8
} aux_ubx_nav_status_bits;

// Packet checksum accumulators
static uint8_t _ck_a;
static uint8_t _ck_b;

// State machine state
static bool _skip_packet;
static uint8_t _step;
static uint8_t _msg_id;
static uint16_t _payload_length;
static uint16_t _payload_counter;

static bool next_fix;
static uint8_t _class;

// do we have new position information?
static bool _new_position;

// do we have new speed information?
static bool _new_speed;

// Example packet sizes from UBlox u-center from a Glonass capable GPS receiver.
//15:17:55  R -> UBX NAV-STATUS,  Size  24,  'Navigation Status'
//15:17:55  R -> UBX NAV-POSLLH,  Size  36,  'Geodetic Position'
//15:17:55  R -> UBX NAV-VELNED,  Size  44,  'Velocity in WGS 84'
//15:17:55  R -> UBX NAV-CLOCK,  Size  28,  'Clock Status'
//15:17:55  R -> UBX NAV-AOPSTATUS,  Size  24,  'AOP Status'
//15:17:55  R -> UBX 03-09,  Size 208,  'Unknown'
//15:17:55  R -> UBX 03-10,  Size 336,  'Unknown'
//15:17:55  R -> UBX NAV-SOL,  Size  60,  'Navigation Solution'
//15:17:55  R -> UBX NAV,  Size 100,  'Navigation'
//15:17:55  R -> UBX NAV-SVINFO,  Size 328,  'Satellite Status and Information'

// from the UBlox6 document, the largest payout we receive i the NAV-SVINFO and the payload size
// is calculated as 8 + 12*numCh.  numCh in the case of a Glonass receiver is 28.
#define UBLOX_PAYLOAD_SIZE 344


// Receive buffer
static union {
    ubx_nav_posllh posllh;
    ubx_nav_status status;
    ubx_nav_solution solution;
    ubx_nav_velned velned;
    ubx_nav_svinfo svinfo;
    ubx_ack ack;
    uint8_t bytes[UBLOX_PAYLOAD_SIZE];
} _buffer;

void _aux_update_checksum(uint8_t *data, uint8_t len, uint8_t *ck_a, uint8_t *ck_b)
{
    while (len--) {
        *ck_a += *data;
        *ck_b += *ck_a;
        data++;
    }
}


static bool UBLOX_parse_gps(void)
{
    uint32_t i;

    *auxGpsPacketLogChar = LOG_IGNORED;

    switch (_msg_id) {
    case MSG_POSLLH:
        *auxGpsPacketLogChar = LOG_UBLOX_POSLLH;
        //i2c_dataset.time                = _buffer.posllh.time;
        auxGpsSol.llh.lon = _buffer.posllh.longitude;
        auxGpsSol.llh.lat = _buffer.posllh.latitude;
        auxGpsSol.llh.altCm = _buffer.posllh.altitudeMslMm / 10;  //alt in cm
        if (next_fix) {
            ENABLE_STATE(AUX_GPS_FIX);
        } else {
            DISABLE_STATE(AUX_GPS_FIX);
        }
        _new_position = true;
        break;
    case MSG_STATUS:
        *auxGpsPacketLogChar = LOG_UBLOX_STATUS;
        next_fix = (_buffer.status.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.status.fix_type == FIX_3D);
        if (!next_fix)
            DISABLE_STATE(AUX_GPS_FIX);
        break;
    case MSG_SOL:
        *auxGpsPacketLogChar = LOG_UBLOX_SOL;
        next_fix = (_buffer.solution.fix_status & NAV_STATUS_FIX_VALID) && (_buffer.solution.fix_type == FIX_3D);
        if (!next_fix)
            DISABLE_STATE(AUX_GPS_FIX);
        auxGpsSol.numSat = _buffer.solution.satellites;
        auxGpsSol.hdop = _buffer.solution.position_DOP;
#ifdef USE_RTC_TIME
        //set clock, when gps time is available
        if(!rtcHasTime() && (_buffer.solution.fix_status & NAV_STATUS_TIME_SECOND_VALID) && (_buffer.solution.fix_status & NAV_STATUS_TIME_WEEK_VALID)) {
            //calculate rtctime: week number * ms in a week + ms of week + fractions of second + offset to UNIX reference year - 18 leap seconds
            rtcTime_t temp_time = (((int64_t) _buffer.solution.week)*7*24*60*60*1000) + _buffer.solution.time + (_buffer.solution.time_nsec/1000000) + 315964800000LL - 18000;
            rtcSet(&temp_time);
        }
#endif
        break;
    case MSG_VELNED:
        *auxGpsPacketLogChar = LOG_UBLOX_VELNED;
        auxGpsSol.speed3d = _buffer.velned.speed_3d;       // cm/s
        auxGpsSol.groundSpeed = _buffer.velned.speed_2d;    // cm/s
        auxGpsSol.groundCourse = (uint16_t) (_buffer.velned.heading_2d / 10000);     // Heading 2D deg * 100000 rescaled to deg * 10
        _new_speed = true;
        break;
    case MSG_SVINFO:
        *auxGpsPacketLogChar = LOG_UBLOX_SVINFO;
        aux_GPS_numCh = _buffer.svinfo.numCh;
        if (aux_GPS_numCh > 16)
            aux_GPS_numCh = 16;
        for (i = 0; i < aux_GPS_numCh; i++) {
            aux_GPS_svinfo_chn[i] = _buffer.svinfo.channel[i].chn;
            aux_GPS_svinfo_svid[i] = _buffer.svinfo.channel[i].svid;
            aux_GPS_svinfo_quality[i] =_buffer.svinfo.channel[i].quality;
            aux_GPS_svinfo_cno[i] = _buffer.svinfo.channel[i].cno;
        }
        for (i = aux_GPS_numCh; i < 16; i++) {
            aux_GPS_svinfo_chn[i] = 0;
            aux_GPS_svinfo_svid[i] = 0;
            aux_GPS_svinfo_quality[i] = 0;
            aux_GPS_svinfo_cno[i] = 0;
        }
        aux_GPS_svInfoReceivedCount++;
        break;
    case MSG_ACK_ACK:
        if ((auxGpsData.ackState == UBLOX_ACK_WAITING) && (_buffer.ack.msgId == auxGpsData.ackWaitingMsgId)) {
            auxGpsData.ackState = UBLOX_ACK_GOT_ACK;
        }
        break;
    case MSG_ACK_NACK:
        if ((auxGpsData.ackState == UBLOX_ACK_WAITING) && (_buffer.ack.msgId == auxGpsData.ackWaitingMsgId)) {
            auxGpsData.ackState = UBLOX_ACK_GOT_NACK;
        }
        break;
    default:
        return false;
    }

    // we only return true when we get new position and speed data
    // this ensures we don't use stale data
    if (_new_position && _new_speed) {
        _new_speed = _new_position = false;
        return true;
    }
    return false;
}

static bool auxGpsNewFrameUBLOX(uint8_t data)
{
    bool parsed = false;

    switch (_step) {
        case 0: // Sync char 1 (0xB5)
            if (PREAMBLE1 == data) {
                _skip_packet = false;
                _step++;
            }
            break;
        case 1: // Sync char 2 (0x62)
            if (PREAMBLE2 != data) {
                _step = 0;
                break;
            }
            _step++;
            break;
        case 2: // Class
            _step++;
            _class = data;
            _ck_b = _ck_a = data;   // reset the checksum accumulators
            break;
        case 3: // Id
            _step++;
            _ck_b += (_ck_a += data);       // checksum byte
            _msg_id = data;
            break;
        case 4: // Payload length (part 1)
            _step++;
            _ck_b += (_ck_a += data);       // checksum byte
            _payload_length = data; // payload length low byte
            break;
        case 5: // Payload length (part 2)
            _step++;
            _ck_b += (_ck_a += data);       // checksum byte
            _payload_length += (uint16_t)(data << 8);
            if (_payload_length > UBLOX_PAYLOAD_SIZE) {
                _skip_packet = true;
            }
            _payload_counter = 0;   // prepare to receive payload
            if (_payload_length == 0) {
                _step = 7;
            }
            break;
        case 6:
            _ck_b += (_ck_a += data);       // checksum byte
            if (_payload_counter < UBLOX_PAYLOAD_SIZE) {
                _buffer.bytes[_payload_counter] = data;
            }
            if (++_payload_counter >= _payload_length) {
                _step++;
            }
            break;
        case 7:
            _step++;
            if (_ck_a != data) {
                _skip_packet = true;          // bad checksum
                auxGpsData.errors++;
            }
            break;
        case 8:
            _step = 0;

            shiftPacketLog();

            if (_ck_b != data) {
                *auxGpsPacketLogChar = LOG_ERROR;
                auxGpsData.errors++;
                break;              // bad checksum
            }

            aux_GPS_packetCount++;

            if (_skip_packet) {
                *auxGpsPacketLogChar = LOG_SKIPPED;
                break;
            }

            if (UBLOX_parse_gps()) {
                parsed = true;
            }
    }
    return parsed;
}
#endif // USE_GPS_UBLOX

static void auxGpsHandlePassthrough(uint8_t data)
{
     auxGpsNewData(data);
 #ifdef USE_DASHBOARD
     if (featureIsEnabled(FEATURE_DASHBOARD)) {
         dashboardUpdate(micros());
     }
 #endif

 }

void auxGpsEnablePassthrough(serialPort_t *gpsPassthroughPort)
{
    waitForSerialPortToFinishTransmitting(auxGpsPort);
    waitForSerialPortToFinishTransmitting(gpsPassthroughPort);

    if (!(auxGpsPort->mode & MODE_TX))
        serialSetMode(auxGpsPort, auxGpsPort->mode | MODE_TX);

#ifdef USE_DASHBOARD
    if (featureIsEnabled(FEATURE_DASHBOARD)) {
        dashboardShowFixedPage(PAGE_GPS);
    }
#endif

    serialPassthrough(auxGpsPort, gpsPassthroughPort, &auxGpsHandlePassthrough, NULL);
}

float aux_GPS_scaleLonDown = 1.0f;  // this is used to offset the shrinking longitude as we go towards the poles

void aux_GPS_calc_longitude_scaling(int32_t lat)
{
    float rads = (fabsf((float)lat) / 10000000.0f) * 0.0174532925f;
    aux_GPS_scaleLonDown = cos_approx(rads);
}

////////////////////////////////////////////////////////////////////////////////////
// Calculate the distance flown and vertical speed from gps position data
//
static void aux_GPS_calculateDistanceTraveledVerticalSpeed(bool initialize)
{
    static int32_t lastCoord[2] = { 0, 0 };
    static int32_t lastAlt;
    static int32_t lastMillis;

    int currentMillis = millis();

    if (initialize) {
        aux_GPS_distanceTraveledInCm = 0;
        aux_GPS_verticalSpeedInCmS = 0;
    } else {
        if (STATE(AUX_GPS_FIX_HOME) && ARMING_FLAG(ARMED)) {
            uint16_t speed = auxGpsConfig()->gps_use_3d_speed ? auxGpsSol.speed3d : auxGpsSol.groundSpeed;
            // Only add up movement when speed is faster than minimum threshold
            if (speed > GPS_DISTANCE_FLOWN_MIN_SPEED_THRESHOLD_CM_S) {
                uint32_t dist;
                int32_t dir;
                aux_GPS_distance_cm_bearing(&auxGpsSol.llh.lat, &auxGpsSol.llh.lon, &lastCoord[LAT], &lastCoord[LON], &dist, &dir);
                if (auxGpsConfig()->gps_use_3d_speed) {
                    dist = sqrtf(powf(auxGpsSol.llh.altCm - lastAlt, 2.0f) + powf(dist, 2.0f));
                }
                aux_GPS_distanceTraveledInCm += dist;
            }
        }
        aux_GPS_verticalSpeedInCmS = (auxGpsSol.llh.altCm - lastAlt) * 1000 / (currentMillis - lastMillis);
        aux_GPS_verticalSpeedInCmS = constrain(aux_GPS_verticalSpeedInCmS, -1500, 1500);
    }
    lastCoord[LON] = auxGpsSol.llh.lon;
    lastCoord[LAT] = auxGpsSol.llh.lat;
    lastAlt = auxGpsSol.llh.altCm;
    lastMillis = currentMillis;
}

void aux_GPS_reset_home_position(void)
{
    if (!STATE(AUX_GPS_FIX_HOME) || !auxGpsConfig()->gps_set_home_point_once) {
        if (STATE(AUX_GPS_FIX) && auxGpsSol.numSat >= 5) {
            aux_GPS_home[LAT] = auxGpsSol.llh.lat;
            aux_GPS_home[LON] = auxGpsSol.llh.lon;
            aux_GPS_calc_longitude_scaling(auxGpsSol.llh.lat); // need an initial value for distance and bearing calc
            // Set ground altitude
            ENABLE_STATE(AUX_GPS_FIX_HOME);
        }
    }
    aux_GPS_calculateDistanceTraveledVerticalSpeed(true); //Initialize
}

////////////////////////////////////////////////////////////////////////////////////
#define DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR_IN_HUNDREDS_OF_KILOMETERS 1.113195f
#define TAN_89_99_DEGREES 5729.57795f
// Get distance between two points in cm
// Get bearing from pos1 to pos2, returns an 1deg = 100 precision
void aux_GPS_distance_cm_bearing(int32_t *currentLat1, int32_t *currentLon1, int32_t *destinationLat2, int32_t *destinationLon2, uint32_t *dist, int32_t *bearing)
{
    float dLat = *destinationLat2 - *currentLat1; // difference of latitude in 1/10 000 000 degrees
    float dLon = (float)(*destinationLon2 - *currentLon1) * aux_GPS_scaleLonDown;
    *dist = sqrtf(sq(dLat) + sq(dLon)) * DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR_IN_HUNDREDS_OF_KILOMETERS;

    *bearing = 9000.0f + atan2_approx(-dLat, dLon) * TAN_89_99_DEGREES;      // Convert the output radians to 100xdeg
    if (*bearing < 0)
        *bearing += 36000;
}

void aux_GPS_calculateDistanceAndDirectionToHome(void)
{
    if (STATE(GPS_FIX_HOME)) {      // If we don't have home set, do not display anything
        uint32_t dist;
        int32_t dir;
        aux_GPS_distance_cm_bearing(&auxGpsSol.llh.lat, &auxGpsSol.llh.lon, &GPS_home[LAT], &GPS_home[LON], &dist, &dir);
        aux_GPS_distanceToHome = dist / 100;
        aux_GPS_directionToHome = dir / 100;
    } else {
        aux_GPS_distanceToHome = 0;
        aux_GPS_directionToHome = 0;
    }
}

void onAuxGpsNewData(void)
{
    if (!(STATE(AUX_GPS_FIX) && auxGpsSol.numSat >= 5)) {
        return;
    }

    //
    // Calculate time delta for navigation loop, range 0-1.0f, in seconds
    //
    // Time for calculating x,y speed and navigation pids
    static uint32_t nav_loopTimer;
    aux_dTnav = (float)(millis() - nav_loopTimer) / 1000.0f;
    nav_loopTimer = millis();
    // prevent runup from bad GPS
    aux_dTnav = MIN(aux_dTnav, 1.0f);

    aux_GPS_calculateDistanceAndDirectionToHome();
    if (ARMING_FLAG(ARMED)) {
        aux_GPS_calculateDistanceTraveledVerticalSpeed(false);
    }

// #ifdef USE_GPS_RESCUE
//     rescueNewGpsData();
// #endif
#ifdef USE_GPS_FOLLOW
    GPS_calculateDistanceAndDirectionToTarget();
    followNewGpsData();
#endif
}

#endif
