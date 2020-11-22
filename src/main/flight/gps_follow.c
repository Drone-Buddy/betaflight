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

#include <stdint.h>
#include <math.h>

#include "platform.h"

#ifdef USE_GPS_FOLLOW

#include "build/debug.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/utils.h"

#include "drivers/time.h"

#include "io/gps.h"
#include "io/aux_gps.h"

#include "config/config.h"

#include "fc/core.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/imu.h"
#include "flight/pid.h"
#include "flight/position.h"

#include "pg/pg.h"
#include "pg/pg_ids.h"

#include "rx/rx.h"

#include "sensors/acceleration.h"

#include "gps_follow.h"

typedef enum {
    FOLLOW_IDLE,                // We have not yet started the GPS_FOLLOW mode
    FOLLOW_INITIALIZE,          // Initialize the GPS_FOLLOW mode if conditions are satisfied
    FOLLOW_ATTAIN_ALT,          // Attain altitude to fly to target
    FOLLOW_CROSSTRACK,          // Fly to target
    FOLLOW_APPROACH,            // Slowdown when we get close to the target
    FOLLOW_FOLLOW,              // Follow the target
    FOLLOW_ABORT,               // Abort GPS_FOLLOW mode
    FOLLOW_COMPLETE             // GPS_FOLLOW mode has reached a termination point
} followPhase_e;

typedef enum {
    FOLLOW_HEALTHY,
    FOLLOW_FLYAWAY,
    FOLLOW_GPSLOST,
    FOLLOW_AUXGPSLOST,
    FOLLOW_LOWSATS,
    FOLLOW_CRASH_FLIP_DETECTED,
    FOLLOW_STALLED,
    FOLLOW_TOO_CLOSE
} followFailureState_e;

typedef struct {
    int32_t targetAltitudeCm;
    int32_t targetGroundSpeed;
    uint8_t minAngleDeg;
    uint8_t maxAngleDeg;
    bool crosstrack;
} followIntent_s;

typedef struct {
    int32_t currentAltitudeCm;
    int32_t currentAltitudeTargetCm;
    uint16_t distanceToTargetM;
    uint16_t maxDistanceToTargetM;
    int16_t directionToTarget;
    uint16_t groundSpeed;
    uint16_t targetGroundSpeed;
    uint8_t numSat;
    uint8_t numSatTarget;
    float zVelocity;
    float zVelocityAvg;
    float accMagnitude;
    float accMagnitudeAvg;
    bool healthy;
} followSensorData_s;

typedef struct {
    bool bumpDetection;
    bool convergenceDetection;
} followSanityFlags;

typedef struct {
    followPhase_e phase;
    followFailureState_e failure;
    followSensorData_s sensor;
    followIntent_s intent;
    bool isAvailable;
} followState_s;

typedef enum {          // This will probably need to be changed
    FIXED_ALT,
    CURRENT_ALT 
} altitudeMode_e;

typedef struct {
    float Kp;
    float Ki;
    float Kd;
} throttle_s;

#define GPS_FOLLOW_MAX_YAW_RATE             180     // deg/sec max yaw rate
#define GPS_FOLLOW_RATE_SCALE_DEGREES        45     // scale the command yaw rate when the error is less than this angle
#define GPS_FOLLOW_SLOWDOWN_DISTANCE_M       10     // distance from target to start decreasing speed
#define GPS_FOLLOW_ZVELOCITY_THRESHOLD      300     // altitude threshold to start decreasing z velocity
#define GPS_FOLLOW_APPROACH_ZVELOCITY        50     // descend velocity for approach phase
#define GPS_FOLLOW_ITERM_WINDUP             100     // reset I term after Z velocity error of this amount in cm/s
#define GPS_FOLLOW_MAX_ITERM_ACC            250.0f  // max allowed iterm value
#define GPS_FOLLOW_SLOWDOWN_ALT             500     // the altitude after which the quad begins to slowdown the descent velocity
#define GPS_FOLLOW_MINIMUM_ZVELOCITY         50     // minimum speed for final attainment phase
#define GPS_FOLLOW_APPROACH_ZVELOCITY        50     // descent speed after reaching target

#define GPS_FOLLOW_THROTTLE_P_SCALE     0.0003125f  // pid scale for P term
#define GPS_FOLLOW_THROTTLE_I_SCALE     0.1f        // pid scale for I term
#define GPS_FOLLOW_THROTTLE_D_SCALE     0.0003125f  // pid scale for D term

#ifdef USE_MAG
    #define GPS_FOLLOW_USE_MAG      true
#else
    #define GPS_FOLLOW_USE_MAG      false
#endif

PG_REGISTER_WITH_RESET_TEMPLATE(
    gpsFollowConfig_t, gpsFollowConfig, 
    PG_GPS_FOLLOW, 1
);

PG_RESET_TEMPLATE(gpsFollowConfig_t, gpsFollowConfig,
    .angle = 32,
    .initialAltitudeM = 4,
    .descentDistanceM = 4,
    .crosstrackGroundSpeed = 200,
    .followGroundSpeed = 200,
    .throttleP = 150,
    .throttleI = 20,
    .throttleD = 50,
    .velP = 80,
    .velI = 20,
    .velD = 15,
    .yawP = 40,
    .throttleMin = 1100,
    .throttleMax = 1600,
    .throttleHover = 1280,
    .sanityChecks = true,
    .minSats = 8,
    .minSatsAux = 8,
    .minFollowDistance = 5,         // actually don't need this -> just use targetFollowDistanceM
    .allowArmingWithoutFix = false,
    .useMag = GPS_FOLLOW_USE_MAG,
    .targetFollowAltitudeM = 5,  // this is the altitude at which we end approach
    .targetFollowDistanceM = 10, // this is the distance at which we end approach
    .altitudeMode = FIXED_ALT,
    .ascendRate = 50,
    .descendRate = 50,
);

static uint16_t followThrottle;
static float    followYaw;

int32_t   gpsFollowAngle[ANGLE_INDEX_COUNT] = { 0, 0 };
float     alitudeError = 0.0;

#ifndef USE_GPS_RESCUE
throttle_s throttle;
uint16_t   hoverThrottle = 0;
float    averageThrottle = 0.0;
uint32_t throttleSamples = 0;
bool     magForceDisable = false;
#else
extern throttle_s      throttle;
extern uint16_t   hoverThrottle;
extern float    averageThrottle;
extern uint32_t throttleSamples;
extern bool     magForceDisable;
#endif

static bool newGPSData = false;

followState_s followState;

// If we have new GPS data, update target heading if possible.
void followNewGpsData(void)
{
    newGPSData = true;
}

static void followStart()
{
    followState.phase = FOLLOW_INITIALIZE;
}

static void followStop()
{
    followState.phase = FOLLOW_IDLE;
}

// Things that need to run regardless if GPS FOLLOW mode is enabled or not
static void idleTasks()
{
    // Do not calculate any of the idle task values when we are not flying
    if (!ARMING_FLAG(ARMED)) {
        followState.sensor.maxDistanceToTargetM = 0;
        return;
    }

    // Don't update any follow flight statistics if we haven't applied a 
    // proper altitude offset yet
    if (!isAltitudeOffset()) {
        return;
    } 
 
    gpsFollowAngle[AI_PITCH] = 0;
    gpsFollowAngle[AI_ROLL] = 0;

    // Store the max distance to the target during normal flight to know if a
    // flyaway is happening
    followState.sensor.maxDistanceToTargetM = MAX(
        followState.sensor.distanceToTargetM,
        followState.sensor.maxDistanceToTargetM
    );

    // Get current throttle from RC to initialize followThrottle
    followThrottle = rcCommand[THROTTLE];

    // Take average throttle as throttle required to hover
    // FIXME: only take samples when acceleration is low
    //
    const float ct = getCosTiltAngle();
    const float accZ = ABS(acc.accADC[Z]) * acc.dev.acc_1G_rec;
    if (accZ < 0.1                  // acc in Z axis is less than 0.1G
        && ct > 0.5 && ct < 1.00    // 0 to 45 degrees zenith angle
        && throttleSamples < 1E6
        && followThrottle > 1070
    ) {
        uint16_t adjustedThrottle = 
            1000 + (followThrottle - PWM_RANGE_MIN) * ct;

        if (throttleSamples == 0) {
            averageThrottle = adjustedThrottle;
        } else {
            averageThrottle += 
                (adjustedThrottle - averageThrottle) / (throttleSamples + 1);
        }
        hoverThrottle = lrintf(averageThrottle);
        throttleSamples++;
    }
}

static void setBearing(int16_t desiredHeading)
{
    float errorAngle = (attitude.values.yaw / 10.0f) - desiredHeading;

    // Constrain angle in range -180 to +180
    if (errorAngle <= -180) {
        errorAngle += 360;
    } else if (errorAngle > 180) {
        errorAngle -= 360;
    }

    // Since we set TAER in terms of RC values we have to account for reversed 
    // yaw controls by negating our angle 
    errorAngle *= -GET_DIRECTION(rcControlsConfig()->yaw_control_reversed);

    followYaw = -constrainf(
        errorAngle / GPS_FOLLOW_RATE_SCALE_DEGREES * GPS_FOLLOW_MAX_YAW_RATE,
        -GPS_FOLLOW_MAX_YAW_RATE,
        GPS_FOLLOW_MAX_YAW_RATE
    );

}

static void followAttainPosition()
{
    // Speed and altitude controller internal variables
    static float previousSpeedError = 0;
    static int16_t speedIntegral = 0;
    int zVelocityError;
    static int previousZVelocityError = 0;
    static float zVelocityIntegral = 0;
    static float scalingRate = 0;
    static int16_t altitudeAdjustment = 0;

    // Initialize internal variables each time GPS Follow is started
    if (followState.phase == FOLLOW_INITIALIZE) {
        previousSpeedError = 0;
        speedIntegral = 0;
        previousZVelocityError = 0;
        zVelocityIntegral = 0;
        altitudeAdjustment = 0;
    }

    // Point to target if that is in our intent
    if (followState.intent.crosstrack) {
        setBearing(followState.sensor.directionToTarget);
    }


    DEBUG_SET(DEBUG_FOLLOW, 3, followState.failure);

    // If we have no new data to base decisions on, skip
    if (!newGPSData) {
        return;
    }

    /** 
     * Speed Controller
     */
    const int16_t speedError = (
        followState.intent.targetGroundSpeed 
        - followState.sensor.groundSpeed
    ) / 100;
    const int16_t speedDerivative = speedError - previousSpeedError;

    speedIntegral = constrain(speedIntegral + speedError, -100, 100);

    previousSpeedError = speedError;

    const int16_t angleAdjustment = 
        gpsFollowConfig()->velP * speedError
        + (gpsFollowConfig()->velI * speedIntegral) / 100
        + gpsFollowConfig()->velD * speedDerivative;

    // FIXME: define constant for maxAngleAdjustment
    gpsFollowAngle[AI_PITCH] = constrain(
        gpsFollowAngle[AI_PITCH] + MIN(angleAdjustment, 80),
        followState.intent.minAngleDeg * 100,
        followState.intent.maxAngleDeg * 100
    );

    const float ct = 
        cos(DECIDEGREES_TO_RADIANS(gpsFollowAngle[AI_PITCH] / 10));

    /**
     * Altitude Controller
     */
    const int16_t altitudeError = 
        followState.intent.targetAltitudeCm 
        - followState.sensor.currentAltitudeCm;
    
    // P component
    // Slow down as we get closer to the desired altitude
    if (ABS(altitudeError) > 0 
        && ABS(altitudeError) < GPS_FOLLOW_ZVELOCITY_THRESHOLD
    ) {
        scalingRate = (float) altitudeError / GPS_FOLLOW_ZVELOCITY_THRESHOLD;
    } else {
        scalingRate = 1;
    }

    if (altitudeError > 0) {
        zVelocityError = 
            gpsFollowConfig()->ascendRate * scalingRate 
            - followState.sensor.zVelocity;
    } else if (altitudeError < 0) {
        if (followState.sensor.currentAltitudeCm <= GPS_FOLLOW_SLOWDOWN_ALT) {
            const int16_t followApproachDescendVel = MAX(
                GPS_FOLLOW_APPROACH_ZVELOCITY 
                  * followState.sensor.currentAltitudeCm 
                  / GPS_FOLLOW_SLOWDOWN_ALT,
                GPS_FOLLOW_MINIMUM_ZVELOCITY
            );

            zVelocityError = 
                -followApproachDescendVel - followState.sensor.zVelocity;
        } else {
            zVelocityError = 
                -gpsFollowConfig()->descendRate * scalingRate 
                - followState.sensor.zVelocity;
        }
    } else {
        zVelocityError = 0;
    }

    // I component
    if (ABS(zVelocityError) < GPS_FOLLOW_ITERM_WINDUP) {
        zVelocityIntegral = constrainf(
            zVelocityIntegral + zVelocityError / 100.0f,
            -GPS_FOLLOW_MAX_ITERM_ACC,
            GPS_FOLLOW_MAX_ITERM_ACC
        );
    } else {
        zVelocityIntegral = 0;
    }

    // D component
    const int zVelocityDerivative = zVelocityError - previousZVelocityError;
    previousZVelocityError = zVelocityError;

    const int16_t hoverAdjustment = (hoverThrottle - 1000) / ct;
    altitudeAdjustment = constrain(
        altitudeAdjustment 
        + (throttle.Kp * zVelocityError 
           + throttle.Ki * zVelocityIntegral 
           + throttle.Kd * zVelocityDerivative),
        gpsFollowConfig()->throttleMin - 1000 - hoverAdjustment,
        gpsFollowConfig()->throttleMax - 1000 - hoverAdjustment
    );

    followThrottle = constrain(
        1000 + altitudeAdjustment + hoverAdjustment,
        gpsFollowConfig()->throttleMin,
        gpsFollowConfig()->throttleMax
    );

    DEBUG_SET(DEBUG_FOLLOW, 0, followThrottle);
    DEBUG_SET(DEBUG_FOLLOW, 1, gpsFollowAngle[AI_PITCH]);
    DEBUG_SET(DEBUG_FOLLOW, 2, altitudeAdjustment);

    DEBUG_SET(DEBUG_GPS_FOLLOW_THROTTLE_PID, 0, throttle.Kp * zVelocityError);
    DEBUG_SET(DEBUG_GPS_FOLLOW_THROTTLE_PID, 1, throttle.Ki * zVelocityIntegral);
    DEBUG_SET(DEBUG_GPS_FOLLOW_THROTTLE_PID, 2, throttle.Kd * zVelocityDerivative);
    DEBUG_SET(DEBUG_GPS_FOLLOW_THROTTLE_PID, 3, followState.sensor.zVelocity);
}

static void performSanityChecks()
{
    static uint32_t previousTimeUs = 0;         // last time checked
    static int8_t secondsStalled = 0;           // stalled movement (wind)
    static uint16_t lastDistanceToTargetM = 0;  // fly-away detection
    static int8_t secondsFlyingAway = 0;
    static int8_t secondsLowSats = 0;           // low sat coverage

    const uint32_t currentTimeUs = micros();

    if (followState.phase == FOLLOW_IDLE) {
        followState.failure = FOLLOW_HEALTHY;
        return;
    } else if (followState.phase == FOLLOW_INITIALIZE) {
        previousTimeUs = currentTimeUs;     // initialize internal variables
        secondsStalled = 10;                // don't stall right at the start
        lastDistanceToTargetM = followState.sensor.distanceToTargetM;
        secondsFlyingAway = 0;
        secondsLowSats = 5;                 // make sure we have sats at start
        return;
    }

    // Abort if previous sanityCheck failed
    if (followState.failure != FOLLOW_HEALTHY) {
        if (gpsFollowConfig()->sanityChecks) {
            followState.phase = FOLLOW_ABORT;
        }
    }

    // Check if we crashed
    if (crashRecoveryModeActive()) {
        followState.failure = FOLLOW_CRASH_FLIP_DETECTED;
    }

    // Check if GPS comms are healthy
    if (!followState.sensor.healthy) {
        followState.failure = FOLLOW_GPSLOST;
    }

    // Skip the rest if it hasn't been a second since we last checked
    const uint32_t dTime = currentTimeUs - previousTimeUs;
    if (dTime < 1E6) { return; }

    previousTimeUs = currentTimeUs;

    if (followState.phase == FOLLOW_CROSSTRACK) {
        // If we're moving at less than 150 cm/s and the target 
        // is moving faster than 150 cm/s then we're stalled
        bool targetMoving = followState.sensor.targetGroundSpeed > 50;
        bool craftMoving = followState.sensor.groundSpeed > 50;
        bool isStalled = targetMoving && !craftMoving;
        secondsStalled = constrain(
            secondsStalled + (isStalled ? 1 : -1),
            0,
            20
        );

        if (secondsStalled == 20) {
            followState.failure = FOLLOW_STALLED;
        }

        // If we're moving away from the target and not within a grace 
        // range of minFollowDistance then we're flying away
        bool isWithinGrace = 
            followState.sensor.distanceToTargetM 
            > gpsFollowConfig()->minFollowDistance;
        bool isFlyingAway = 
            lastDistanceToTargetM < followState.sensor.distanceToTargetM;
        secondsFlyingAway = constrain(
            secondsFlyingAway + ((isFlyingAway && !isWithinGrace) ? 1 : -1),
            0,
            10
        );
        lastDistanceToTargetM = followState.sensor.distanceToTargetM;

        if (secondsFlyingAway == 10) {
#ifdef USE_MAG
            if (sensors(SENSOR_MAG) 
                && gpsFollowConfig()->useMag 
                && !magForceDisable
            ) {
                // Try again with magnetometer disabled just in case
                magForceDisable = true;
                secondsFlyingAway = 0;
            } else
#endif
            {
                followState.failure = FOLLOW_FLYAWAY;
            }
        }
    }

    bool areSatsLow = 
        followState.sensor.numSat < gpsFollowConfig()->minSats
        || followState.sensor.numSatTarget < gpsFollowConfig()->minSatsAux;
    secondsLowSats = constrain(secondsLowSats + (areSatsLow ? 1 : -1), 0, 10);

    if (secondsLowSats == 10) {
        followState.failure = FOLLOW_LOWSATS;
    }
}

static void sensorUpdate()
{
    followState.sensor.currentAltitudeCm = getEstimatedAltitudeCm();
    followState.sensor.currentAltitudeTargetCm = auxGpsSol.llh.altCm;
    followState.sensor.healthy = gpsIsHealthy() && auxGpsIsHealthy();

    // Calculate altitude velocity
    static uint32_t previousTimeUs;
    static int32_t previousAltitudeCm;

    const uint32_t currentTimeUs = micros();
    const float dTime = currentTimeUs - previousTimeUs;

    if (newGPSData) {
        followState.sensor.distanceToTargetM = GPS_distanceToTarget;
        followState.sensor.directionToTarget = GPS_directionToTarget;
        followState.sensor.numSat = gpsSol.numSat;
        followState.sensor.numSatTarget = auxGpsSol.numSat;
        followState.sensor.groundSpeed = gpsSol.groundSpeed;
        followState.sensor.targetGroundSpeed = auxGpsSol.groundSpeed;

        followState.sensor.zVelocity = 
            (followState.sensor.currentAltitudeCm - previousAltitudeCm) 
            * 1E6 / dTime;
        followState.sensor.zVelocityAvg = 
            0.8f * followState.sensor.zVelocityAvg 
            + followState.sensor.zVelocity * 0.2f;

        followState.sensor.accMagnitude = (float) 
            sqrtf(
                sq(acc.accADC[Z]) + sq(acc.accADC[X]) + sq(acc.accADC[Y])
            ) * acc.dev.acc_1G_rec;
        followState.sensor.accMagnitudeAvg = 
            (followState.sensor.accMagnitudeAvg * 0.8f)
            + (followState.sensor.accMagnitude * 0.2f);
        
        previousAltitudeCm = followState.sensor.currentAltitudeCm;
        previousTimeUs = currentTimeUs;
    }

}

// Checks for the following conditions:
// 1. GPS data is being received
// 2. AUX GPS data is being received
// 3. GPS has a fix
// 4. AUX GPS has a fix
// 5. Configured minimum number of GPS satellites is satisfied
// 6. Configured minimum number AUX GPS satellites is satisfied
static bool checkGPSFollowIsAvailable(void)
{
    static uint32_t previousTimeUs = 0;
    const uint32_t currentTimeUs = micros();
    static int8_t secondsLowSats = 0;
    static bool lowSats = false;
    static bool lowSatsAux = false;
    static bool noGPSFix = false;
    static bool noAuxGPSFix = false;
    bool result = true;

    if(!gpsIsHealthy() || !auxGpsIsHealthy()) {
        return false;
    }

    // Only run checks at 1 Hz
    const uint32_t dTime = currentTimeUs - previousTimeUs;
    if (dTime < 1E6) {
        if (noGPSFix || noAuxGPSFix || lowSats || lowSatsAux) {
            result = false;
        }
        return result;
    }

    previousTimeUs = currentTimeUs;
    
    if (!STATE(GPS_FIX)) {
        result = false;
        noGPSFix = true;
    } else if (!STATE(AUX_GPS_FIX)) {
        result = false;
        noAuxGPSFix = true;
    }

    bool areSatsLow = gpsSol.numSat < gpsFollowConfig()->minSats;
    bool areAuxSatsLow = auxGpsSol.numSat < gpsFollowConfig()->minSatsAux;
    secondsLowSats = constrain(
        secondsLowSats + ((areSatsLow || areAuxSatsLow) ? 1 : -1),
        0,
        2
    );
    if (secondsLowSats == 2) {
        lowSats = true;
        result = false;
    } else {
        lowSats = false;
    }

    return result;
}

void updateGPSFollowState(void)
{
    static uint16_t newDescentDistanceM;
    static float_t lineSlope;
    static float_t lineOffsetM;
    static int32_t newSpeed;
    static int32_t newAltitude;

    if (!FLIGHT_MODE(GPS_FOLLOW_MODE)) {
        followStop();
    } 
    else if (
        FLIGHT_MODE(GPS_FOLLOW_MODE) 
        && followState.phase == FOLLOW_IDLE
    ) {
        followStart();
        followAttainPosition();         // Initialize
        performSanityChecks();          // Initialize
    }

    sensorUpdate();
    followState.isAvailable = checkGPSFollowIsAvailable();

    switch (followState.phase) {
        case FOLLOW_IDLE:
            idleTasks();
            break;
        case FOLLOW_INITIALIZE:
            if (hoverThrottle == 0) {
                hoverThrottle = gpsFollowConfig()->throttleHover;
            }

            throttle.Kp = 
                gpsFollowConfig()->throttleP * GPS_FOLLOW_THROTTLE_P_SCALE;
            throttle.Ki = 
                gpsFollowConfig()->throttleI * GPS_FOLLOW_THROTTLE_I_SCALE;
            throttle.Kd = 
                gpsFollowConfig()->throttleD * GPS_FOLLOW_THROTTLE_D_SCALE;

            if (!STATE(AUX_GPS_FIX)) {
                setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
                disarm(DISARM_REASON_GPS_FOLLOW);
            }

            newSpeed = gpsFollowConfig()->crosstrackGroundSpeed;
            // set new descent distance if actual distance to target is lower
            if (followState.sensor.distanceToTargetM 
                < gpsFollowConfig()->descentDistanceM
            ) {
                newDescentDistanceM = 
                    MAX(followState.sensor.distanceToTargetM - 5, 1);
            } else {
                newDescentDistanceM = gpsFollowConfig()->descentDistanceM;
            }

            switch (gpsFollowConfig()->altitudeMode) {
                case FIXED_ALT:
                    newAltitude = gpsFollowConfig()->initialAltitudeM * 100;
                    break;
                case CURRENT_ALT:
                default:
                    newAltitude = followState.sensor.currentAltitudeCm;
                    break;
            }

            lineSlope = 
                ((float) gpsFollowConfig()->initialAltitudeM 
                    - gpsFollowConfig()->targetFollowAltitudeM)
                / (newDescentDistanceM 
                    - gpsFollowConfig()->targetFollowDistanceM);
            lineOffsetM = gpsFollowConfig()->initialAltitudeM 
                - lineSlope * newDescentDistanceM;

            followState.phase = FOLLOW_ATTAIN_ALT;
            FALLTHROUGH;
        case FOLLOW_ATTAIN_ALT:
            // Get to a safe altitude at a low velocity ASAP
            if (ABS(followState.intent.targetAltitudeCm 
                - followState.sensor.currentAltitudeCm) < 100
            ) {
                followState.phase = FOLLOW_CROSSTRACK;
            }

            followState.intent.targetGroundSpeed = 10;
            followState.intent.targetAltitudeCm = newAltitude;
            followState.intent.crosstrack = true;
            followState.intent.minAngleDeg = 0;
            followState.intent.maxAngleDeg = 15;
            break;
        case FOLLOW_CROSSTRACK:
            if (followState.sensor.distanceToTargetM <= newDescentDistanceM) {
                followState.phase = FOLLOW_APPROACH;
            }

            followState.intent.targetGroundSpeed = 
                gpsFollowConfig()->crosstrackGroundSpeed;
            followState.intent.targetAltitudeCm = newAltitude;
            followState.intent.crosstrack = true;
            followState.intent.minAngleDeg = 0;
            followState.intent.maxAngleDeg = gpsFollowConfig()->angle;
            break;
        case FOLLOW_APPROACH:
            if (followState.sensor.distanceToTargetM
                    <= gpsFollowConfig()->targetFollowDistanceM
                && followState.sensor.currentAltitudeCm 
                    <= gpsFollowConfig()->targetFollowAltitudeM * 100
            ) {
                followState.phase = FOLLOW_FOLLOW;
            }

            // Prevent parabolic movement when approaching the target
            // Only allow altitudes to be lower than the current 
            const int32_t newAlt = MAX(
                (lineSlope * followState.sensor.distanceToTargetM 
                    + lineOffsetM) * 100,
                0
            );

            // Start to decrease speed when distance to home is less than or
            // equal to GPS_FOLLOW_SLOWDOWN_DISTANCE_M
            if (followState.sensor.distanceToTargetM 
                <= GPS_FOLLOW_SLOWDOWN_DISTANCE_M
            ) {
                newSpeed = gpsFollowConfig()->crosstrackGroundSpeed 
                    * followState.sensor.distanceToTargetM 
                    / GPS_FOLLOW_SLOWDOWN_DISTANCE_M;
            }

            followState.intent.targetAltitudeCm = 
                constrain(newAlt, 300, followState.intent.targetAltitudeCm);
            followState.intent.targetGroundSpeed =
                constrain(newSpeed, 10, followState.intent.targetGroundSpeed);
            followState.intent.crosstrack = true;
            followState.intent.minAngleDeg = 0;
            followState.intent.maxAngleDeg = gpsFollowConfig()->angle;
            break;
        case FOLLOW_FOLLOW:
            // We are now following the target.
            followState.intent.targetAltitudeCm = 
                gpsFollowConfig()->targetFollowAltitudeM * 100;
            followState.intent.targetGroundSpeed =
                followState.sensor.targetGroundSpeed;
            followState.intent.crosstrack = true;
            followState.intent.minAngleDeg = 0;
            followState.intent.maxAngleDeg = 15;
            break;
        case FOLLOW_COMPLETE:
            // This currently no way to reach this state.
            // If GPS_FOLLOW was repurposed for following a predetermined
            // path this could be useful.
            followStop();
            break;
        case FOLLOW_ABORT:
            setArmingDisabled(ARMING_DISABLED_ARM_SWITCH);
            disarm(DISARM_REASON_GPS_FOLLOW);
            followStop();
            break;
        default:
            break;
    }

    performSanityChecks();

    if (followState.phase != FOLLOW_IDLE) {
        followAttainPosition();
    }

    newGPSData = false;
}

float gpsFollowGetYawRate(void)
{
    return followYaw;
}

float gpsFollowGetThrottle(void)
{
    float commandedThrottle = scaleRangef(
        followThrottle,
        MAX(rxConfig()->mincheck,PWM_RANGE_MIN), PWM_RANGE_MAX,
        0.0f, 1.0f
    );
    commandedThrottle = constrainf(commandedThrottle, 0.0f, 1.0f);

    return commandedThrottle;
}

bool gpsFollowIsConfigured(void)
{
    return isModeActivationConditionPresent(BOXGPSFOLLOW);
}

bool gpsFollowIsAvailable(void)
{
    return followState.isAvailable;
}

bool gpsFollowIsDisabled(void)
{
    return (!STATE(GPS_FIX) && !STATE(AUX_GPS_FIX));
}

#ifdef USE_MAG
bool gpsFollowDisableMag(void)
{
    // Determine if magnetometer has been disabled by GPS_FOLLOW
    return (
        (!gpsFollowConfig()->useMag || magForceDisable)
        && (followState.phase >= FOLLOW_INITIALIZE)
        && (followState.phase <= FOLLOW_COMPLETE)
    );
}
#endif

#endif
