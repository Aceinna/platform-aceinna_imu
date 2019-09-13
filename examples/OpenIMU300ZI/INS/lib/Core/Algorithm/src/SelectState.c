/*
 * File:   SelectState.c
 * Author: joemotyka
 *
 * Created on May 8, 2016, 12:35 AM
 */

#include <stdio.h>
#include <string.h>   // memset
#include <math.h>     // fabs

#include "GlobalConstants.h"
#include "platformAPI.h"  

#include "algorithm.h"        // gAlgorithm
#include "algorithmAPI.h"
#include "Indices.h"      // IND
#include "StateIndices.h" // STATE_IND
#include "AlgorithmLimits.h"       // LIMIT
#include "VectorMath.h"          // VectorNormalize
#include "QuaternionMath.h"      // EulerAnglesToQuaternion, QuaternionToEulerAngles
#include "TransformationMath.h"  // FieldVectorsToEulerAngles
#include "EKF_Algorithm.h"
#include "PredictFunctions.h"
#include "TimingVars.h"

#ifndef INS_OFFLINE
#ifdef DISPLAY_DIAGNOSTIC_MSG
#include "debug.h"
#endif
#endif


static real accumulatedAccelVector[3];
static real accumulatedGyroVector[3];
static real accumulatedMagVector[3];
static real averagedAccelVector[3];
static real averagedGyroVector[3];
static real averagedMagVector[3];

// Local functions
static BOOL _AccumulateFieldVectors(void);
static BOOL _AverageFieldVectors(uint16_t pointsToAverage);

static void _DropToHighGainAHRS(void);
static void _ResetAlgorithm(void);

static uint8_t _InitINSFilter(real* leverArmN);

// StabilizeSystem: Run for a prescribed period to let the sensors settle.
void StabilizeSystem(void)
{
    // Decrement timer (initial value is set based on the calling frequency of
    //   the EKF)
    gAlgorithm.stateTimer = gAlgorithm.stateTimer - 1;

    /* Upon timeout prepare for transition to the next stage of the EKF
     * (initialization) by resetting the state and state-timer and
     * initializing the accumulation vectors.
     */
    if (gAlgorithm.stateTimer == 0) {
        #ifdef INS_OFFLINE
        printf("To ini att. %u\n", gEKFInputData.itow);
        #else
#ifdef DISPLAY_DIAGNOSTIC_MSG
        DebugPrintString("To ini att. ");
        DebugPrintInt("", gEKFInputData.itow);
        DebugPrintEndline();
#endif
        #endif
        // Set new state and timer
        gAlgorithm.state      = INITIALIZE_ATTITUDE;
        gAlgorithm.stateTimer = gAlgorithm.Duration.Initialize_Attitude;

        // Initialize the vectors
        accumulatedAccelVector[X_AXIS] = (real)0.0;
        accumulatedAccelVector[Y_AXIS] = (real)0.0;
        accumulatedAccelVector[Z_AXIS] = (real)0.0;

        accumulatedGyroVector[X_AXIS] = (real)0.0;
        accumulatedGyroVector[Y_AXIS] = (real)0.0;
        accumulatedGyroVector[Z_AXIS] = (real)0.0;

        accumulatedMagVector[X_AXIS] = (real)0.0;
        accumulatedMagVector[Y_AXIS] = (real)0.0;
        accumulatedMagVector[Z_AXIS] = (real)0.0;

#ifdef DISPLAY_DIAGNOSTIC_MSG
        TimingVars_DiagnosticMsg("Transitioning to initialization mode");
#endif
    }

    // Set the bit to indicate initialization
    gAlgoStatus.bit.algorithmInit  = TRUE;
}


/* InitializeAttitude: Initialize the algorithm by collecting sensor data for
 * a prescribed period and averaging it.
 */
void InitializeAttitude(void)
{
    // Decrement timer
    gAlgorithm.stateTimer = gAlgorithm.stateTimer - 1;

    /* Sum the acceleration and magnetic-field vectors (from the end of the
     * initialization stage)
     */
    _AccumulateFieldVectors();

    /* Quasi-static check: check for motion over threshold. If detected, reset
     * the accumulation variables and restart initialization phase.
     */
    if ((fabs(gEKFInputData.angRate_B[X_AXIS]) > LIMIT_QUASI_STATIC_STARTUP_RATE) ||
        (fabs(gEKFInputData.angRate_B[Y_AXIS]) > LIMIT_QUASI_STATIC_STARTUP_RATE) ||
        (fabs(gEKFInputData.angRate_B[Z_AXIS]) > LIMIT_QUASI_STATIC_STARTUP_RATE))
    {
        accumulatedAccelVector[X_AXIS] = (real)0.0;
        accumulatedAccelVector[Y_AXIS] = (real)0.0;
        accumulatedAccelVector[Z_AXIS] = (real)0.0;

        accumulatedGyroVector[X_AXIS] = (real)0.0;
        accumulatedGyroVector[Y_AXIS] = (real)0.0;
        accumulatedGyroVector[Z_AXIS] = (real)0.0;

        accumulatedMagVector[X_AXIS] = (real)0.0;
        accumulatedMagVector[Y_AXIS] = (real)0.0;
        accumulatedMagVector[Z_AXIS] = (real)0.0;

        gAlgorithm.stateTimer = gAlgorithm.Duration.Initialize_Attitude;
    }

    /* Timeout...  Prepare for the transition to the next stage of the EKF
     * (High-Gain AHRS) then determine the system's Initial Conditions by
     * averaging the accumulated vectors.
     */
    if (gAlgorithm.stateTimer == 0) {
        #ifdef INS_OFFLINE
        printf("To HG. %u\n", gEKFInputData.itow);
        #else
#ifdef DISPLAY_DIAGNOSTIC_MSG
        DebugPrintString("To HG. ");
        DebugPrintInt("", gEKFInputData.itow);
        DebugPrintEndline();
#endif
        #endif
#ifdef DISPLAY_DIAGNOSTIC_MSG
        if (magUsedInAlgorithm()) {
            TimingVars_DiagnosticMsg("Transitioning to high-gain AHRS mode");
        } else {
            TimingVars_DiagnosticMsg("Transitioning to high-gain VG mode");
        }
#endif

        // Set new state and timer
        gAlgorithm.state      = HIGH_GAIN_AHRS;
        gAlgorithm.stateTimer = gAlgorithm.Duration.High_Gain_AHRS;

        /* Average acceleration and magnetic field-vectors to determine the
         * initial attitude of the system.
         */
        _AverageFieldVectors(gAlgorithm.Duration.Initialize_Attitude);

        /* Compute the measured Euler Angles and associated quaternion
         * Euler angles are computed from averaged field vectors 
         * (correct for hard/soft-iron effects)
         */
        // Unit gravity vector in the body frame
        real unitGravityVector[3] = {0.0f};
        UnitGravity( averagedAccelVector, unitGravityVector );
        // Roll and pitch
        UnitGravityToEulerAngles( unitGravityVector, gKalmanFilter.measuredEulerAngles );
        // Yaw
        if ( magUsedInAlgorithm() )
        {
            gKalmanFilter.measuredEulerAngles[YAW] = UnitGravityAndMagToYaw( unitGravityVector,
                                                                             averagedMagVector );
        }
        else
        {
            gKalmanFilter.measuredEulerAngles[YAW] = 0.0f;  // start from 0
        }

        /* Initial attitude quaternion is generated using Euler angles from
         * averaged gravity and magnetic fields. (DEBUG: This is used to
         * initialize the EKF state)
         */
        EulerAnglesToQuaternion( gKalmanFilter.measuredEulerAngles,
                                 gKalmanFilter.quaternion );
        // Euler angles from the initial measurement (DEBUG: initial output of the system)
        gKalmanFilter.eulerAngles[ROLL] = gKalmanFilter.measuredEulerAngles[ROLL];
        gKalmanFilter.eulerAngles[PITCH] = gKalmanFilter.measuredEulerAngles[PITCH];
        gKalmanFilter.eulerAngles[YAW] = gKalmanFilter.measuredEulerAngles[YAW];

        // Initialize the Kalman filter variables
        _ResetAlgorithm();

        // Set linear-acceleration switch variables
        gAlgorithm.linAccelSwitchCntr = 0;

        /// Update the system status
        gAlgoStatus.bit.algorithmInit         = FALSE;
        gAlgoStatus.bit.highGain              = TRUE;
        gAlgoStatus.bit.attitudeOnlyAlgorithm = TRUE;
    }
}


/* HG_To_LG_Transition_Test: Transition from high-gain to low-gain.  Only check
 * is that the bias isn't greater than 10 deg/sec (this is probably not a good check).
 */
void HG_To_LG_Transition_Test(void)
{
    /* Decrement timer if 'dynamicMotion' TRUE (setting FALSE will cause the
     * system to revert to high - gain mode once out of high - gain mode -- need
     * to set flag high to transition out of high - gain mode once this is done)
     */
    /* dynamic-motion flag switch from high-gain to low-gain AHRS. if not set
     * timer will not decrement the transition to LG AHRS will not occur.
     * set at system configuration or (Nav-View) interface
     */
    if (gAlgorithm.Behavior.bit.dynamicMotion) {
        gAlgorithm.stateTimer--;
    }

    /* Startup check (if the estimated bias is large the software never
     * transitions to the LG AHRS mode. NOTE: this seems incorrect, instead
     * the SW should check if the bias has converged, not if it is above a
     * threshold -- it is possible that the system could have a large bias.)
     *  However, this seems wrong too
     */
    if ((fabs(gKalmanFilter.rateBias_B[X_AXIS]) > TEN_DEGREES_IN_RAD) &&
        (fabs(gKalmanFilter.rateBias_B[Y_AXIS]) > TEN_DEGREES_IN_RAD) &&
        (fabs(gKalmanFilter.rateBias_B[Z_AXIS]) > TEN_DEGREES_IN_RAD))
    {
        gAlgorithm.stateTimer = gAlgorithm.Duration.High_Gain_AHRS;
    }

    /* Timeout...  Prepare for the transition to the next stage of the EKF
     * (Low-Gain AHRS) and populate the values in Q that do not change with
     * each iteration.
     */
    if (gAlgorithm.stateTimer == 0) {
        #ifdef INS_OFFLINE
        printf("To LG. %u\n", gEKFInputData.itow);
        #else
#ifdef DISPLAY_DIAGNOSTIC_MSG
        DebugPrintString("To LG. ");
        DebugPrintInt("", gEKFInputData.itow);
        DebugPrintEndline();
#endif
        #endif
#ifdef DISPLAY_DIAGNOSTIC_MSG
        if (magUsedInAlgorithm()) {
            TimingVars_DiagnosticMsg("Transitioning to low-gain AHRS mode");
        } else {
            TimingVars_DiagnosticMsg("Transitioning to low-gain VG mode");
        }
#endif

        gAlgorithm.state      = LOW_GAIN_AHRS;
        gAlgorithm.stateTimer = gAlgorithm.Duration.Low_Gain_AHRS;

        gAlgoStatus.bit.highGain = FALSE;
    }
}


/* This logic is only called upon transition to INS from LG_AHRS then it is
 * not called unless the algorithm reverts back to HG_AHRS, which will
 * cause the system to pass through LG_AHRS on its way to INS.
 */
void LG_To_INS_Transition_Test(void)
{
#ifdef DISPLAY_DIAGNOSTIC_MSG
    // Display the diagnostic message once upon transition (DEBUG: Remove in firmware)
    static int oneTime = TRUE;
#endif

    if (gAlgorithm.stateTimer > 0) {
        // Stay in LG mode until timeout occurs then begin check for INS transition
        gAlgorithm.stateTimer = gAlgorithm.stateTimer - 1;
    } else {
        // Upon timeout, begin check for INS transition (remove msg in firmware)
#ifdef DISPLAY_DIAGNOSTIC_MSG
        if (oneTime) {
            TimingVars_DiagnosticMsg("Begin check for INS transition");
            oneTime = FALSE;
        }
#endif

        /* If GPS output is valid (GPS providing data with a good signal lock)
         * then transit to INS mode.
         */
        if ( gpsUsedInAlgorithm() && gEKFInputData.gpsUpdate ) 
        {
            #ifdef INS_OFFLINE
            printf("To INS. %u\n", gEKFInputData.itow);
            #else
#ifdef DISPLAY_DIAGNOSTIC_MSG
            DebugPrintString("To INS. ");
            DebugPrintInt("", gEKFInputData.itow);
            DebugPrintEndline();
#endif
            #endif
#ifdef DISPLAY_DIAGNOSTIC_MSG
            TimingVars_DiagnosticMsg("Transitioning to INS mode");
#endif

            // Transit to INS solution
            gAlgorithm.state = INS_SOLUTION;

            // Sync the algorithm and GPS ITOW
            gAlgorithm.itow = gEKFInputData.itow;
            
            /* We have a good GPS reading now - set this variable so we
             * don't drop into INS right away
             */
            gAlgorithm.timeOfLastGoodGPSReading = gEKFInputData.itow;

            /* Prepare for INS.
			 * R_NinE and rGPS_E are used later. rGPS_E is used in _InitINSFilter to initialize rGPS0_E.
			 * R_NinE is in UpdateFunctions.c to convert position to ECEF. R_NinE is also calculated in
			 * UpdateFunctions.c. But it is necessary here because it is needed to calculate position
			 * after switching to INS and before the next GNSS measurement to trigger a Kalman update.
			 */
            LLA_To_Base(&gEKFInputData.llaRad[0],
                        &gAlgorithm.rGPS0_E[0],
                        &gAlgorithm.rGPS_N[0],
                        &gAlgorithm.R_NinE[0][0],
                        &gAlgorithm.rGPS_E[0]);

            /* Lever-arm is antenna position w.r.t to IMU in body. rGPS_N is antenna positive
             * w.r.t to initial point in NED. IMU positive w.r.t initial point is
             * rGPS_N - R_b_to_N * lever-arm
             */
            float leverArmN[3];
            leverArmN[0] = gKalmanFilter.R_BinN[0][0] * gAlgorithm.leverArmB[0] +
                gKalmanFilter.R_BinN[0][1] * gAlgorithm.leverArmB[1] +
                gKalmanFilter.R_BinN[0][2] * gAlgorithm.leverArmB[2];
            leverArmN[1] = gKalmanFilter.R_BinN[1][0] * gAlgorithm.leverArmB[0] +
                gKalmanFilter.R_BinN[1][1] * gAlgorithm.leverArmB[1] +
                gKalmanFilter.R_BinN[1][2] * gAlgorithm.leverArmB[2];
            leverArmN[2] = gKalmanFilter.R_BinN[2][0] * gAlgorithm.leverArmB[0] +
                gKalmanFilter.R_BinN[2][1] * gAlgorithm.leverArmB[1] +
                gKalmanFilter.R_BinN[2][2] * gAlgorithm.leverArmB[2];
            
            // Initialize the algorithm with GNSS position and lever arm
            _InitINSFilter(leverArmN);

            // Set linear-acceleration switch variables
            gAlgorithm.linAccelSwitchCntr = 0;
        }
    }
}




//
static uint8_t _InitINSFilter(real* leverArmN)
{
    real tmp[7][7];
    int rowNum, colNum;

    gAlgorithm.insFirstTime = FALSE;

    /* Upon the first entry into INS, save off the base position and reset the
     *   Kalman filter variables.
     */
    // Save off the base ECEF location
    gAlgorithm.rGPS0_E[X_AXIS] = gAlgorithm.rGPS_E[X_AXIS];
    gAlgorithm.rGPS0_E[Y_AXIS] = gAlgorithm.rGPS_E[Y_AXIS];
    gAlgorithm.rGPS0_E[Z_AXIS] = gAlgorithm.rGPS_E[Z_AXIS];

    /* Reset the gps position (as position is relative to starting location)
     * rGPS_N is the IMU position, while starting location is the antenna position.
     * rGPS_N is reset to lever-arm.
     */
    gAlgorithm.rGPS_N[X_AXIS] = -leverArmN[0];
    gAlgorithm.rGPS_N[Y_AXIS] = -leverArmN[1];
    gAlgorithm.rGPS_N[Z_AXIS] = -leverArmN[2];

    // Reset prediction values. Position_N is also IMU position.
    gKalmanFilter.Position_N[X_AXIS] = (real)-leverArmN[0];
    gKalmanFilter.Position_N[Y_AXIS] = (real)-leverArmN[1];
    gKalmanFilter.Position_N[Z_AXIS] = (real)-leverArmN[2];

    gKalmanFilter.Velocity_N[X_AXIS] = (real)gEKFInputData.vNed[X_AXIS];
    gKalmanFilter.Velocity_N[Y_AXIS] = (real)gEKFInputData.vNed[Y_AXIS];
    gKalmanFilter.Velocity_N[Z_AXIS] = (real)gEKFInputData.vNed[Z_AXIS];

    gKalmanFilter.accelBias_B[X_AXIS] = (real)0.0;
    gKalmanFilter.accelBias_B[Y_AXIS] = (real)0.0;
    gKalmanFilter.accelBias_B[Z_AXIS] = (real)0.0;

    /* Extract the Quaternion and rate-bias values from the matrix before
     * resetting
     */
    // Save off the quaternion and rate-bias covariance values
    for (rowNum = Q0; rowNum <= Q3 + Z_AXIS + 1; rowNum++) 
    {
        for (colNum = Q0; colNum <= Q3 + Z_AXIS + 1; colNum++) 
        {
            tmp[rowNum][colNum] = gKalmanFilter.P[rowNum + STATE_Q0][colNum + STATE_Q0];
        }
    }

    // Reset P
    memset(gKalmanFilter.P, 0, sizeof(gKalmanFilter.P));
    for (rowNum = 0; rowNum < NUMBER_OF_EKF_STATES; rowNum++) 
    {
        gKalmanFilter.P[rowNum][rowNum] = (real)INIT_P_INS;
    }

    // Repopulate the P matrix with the quaternion and rate-bias values
    for (rowNum = Q0; rowNum <= Q3 + Z_AXIS + 1; rowNum++) 
    {
        for (colNum = Q0; colNum <= Q3 + Z_AXIS + 1; colNum++) 
        {
            gKalmanFilter.P[rowNum + STATE_Q0][colNum + STATE_Q0] = tmp[rowNum][colNum];
        }
    }

    /* Use the GPS-provided horizontal and vertical accuracy values to populate
     *   the covariance values.
     */
    gKalmanFilter.P[STATE_RX][STATE_RX] = gEKFInputData.GPSHorizAcc * gEKFInputData.GPSHorizAcc;
    gKalmanFilter.P[STATE_RY][STATE_RY] = gKalmanFilter.P[STATE_RX][STATE_RX];
    gKalmanFilter.P[STATE_RZ][STATE_RZ] = gEKFInputData.GPSVertAcc * gEKFInputData.GPSVertAcc;

    /* Scale the best velocity error by HDOP then multiply by the z-axis angular
     * rate PLUS one (to prevent the number from being zero) so the velocity
     * update during high-rate turns is reduced.
     */
    float temp = (real)0.0625 * gEKFInputData.HDOP;  // 0.0625 = 0.05 / 0.8
    real absFilteredYawRate = (real)fabs(gAlgorithm.filteredYawRate);
    if (absFilteredYawRate > TEN_DEGREES_IN_RAD)
    {
        temp *= (1.0f + absFilteredYawRate);
    }
    gKalmanFilter.P[STATE_VX][STATE_VX] = temp;// *((real)1.0 + fabs(gAlgorithm.filteredYawRate) * (real)RAD_TO_DEG);
    gKalmanFilter.P[STATE_VX][STATE_VX] = gKalmanFilter.P[STATE_VX][STATE_VX] * gKalmanFilter.P[STATE_VX][STATE_VX];
    gKalmanFilter.P[STATE_VY][STATE_VY] = gKalmanFilter.P[STATE_VX][STATE_VX];

    // z-axis velocity isn't really a function of yaw-rate and hdop
    //gKalmanFilter.R[STATE_VZ][STATE_VZ] = gKalmanFilter.R[STATE_VX][STATE_VX];
    gKalmanFilter.P[STATE_VZ][STATE_VZ] = (float)(0.1 * 0.1);

    return 1;
}


/* INS_To_AHRS_Transition_Test:  Drop back to LG AHRS operation if...
 *   1) GPS drops out for more than 3 seconds
 *   2) magnetometer data not available AND at rest too long
 *   3) magnetic alignment being performed
 */
void INS_To_AHRS_Transition_Test(void)
{
    // Record last GPS velocity large enough to give a good heading measurement
    if (gEKFInputData.rawGroundSpeed >= LIMIT_MIN_GPS_VELOCITY_HEADING)
    {
        gAlgorithm.timeOfLastSufficientGPSVelocity = (int32_t)gEKFInputData.itow;
    }
    /* Determine the length of time it has been since the system 'moved' --
     * only linear motion considered (rotations ignored).
     */
    int32_t timeSinceRestBegan = (int32_t)gEKFInputData.itow - gAlgorithm.timeOfLastSufficientGPSVelocity;
    if (timeSinceRestBegan < 0)
    {
        timeSinceRestBegan = timeSinceRestBegan + MAX_ITOW;
    }
    if (timeSinceRestBegan > LIMIT_MAX_REST_TIME_BEFORE_HEADING_INVALID)
    {
        gAlgorithm.gnssHeadingFirstTime = TRUE;
    }

    // compute time since the last good GPS reading
    int32_t timeSinceLastGoodGPSReading = (int32_t)gAlgorithm.itow - gAlgorithm.timeOfLastGoodGPSReading;
    if (timeSinceLastGoodGPSReading < 0) {
        timeSinceLastGoodGPSReading = timeSinceLastGoodGPSReading + MAX_ITOW;
    }

    if ( timeSinceLastGoodGPSReading > gAlgorithm.Limit.Max_GPS_Drop_Time )
    {
        // Currently in INS mode but requiring a transition to AHRS / VG
        gAlgorithm.insFirstTime = TRUE;
        gAlgorithm.gnssHeadingFirstTime = TRUE;

        /* The transition from INS to AHRS and back to INS does not seem to
         * generate a stable solution if we transition to LG AHRS for only 30
         * seconds.The interval needs to be longer(~1min).However, to
         * mitigate any unforseen issues with the transition, HG AHRS with the
         * nominal timing(1 min in HG, 30 seconds in LG) will be selected.
         */
        gAlgorithm.state      = LOW_GAIN_AHRS;            // HIGH_GAIN_AHRS;
        gAlgorithm.stateTimer = gAlgorithm.Duration.Low_Gain_AHRS;   // gAlgorithm.Duration.High_Gain_AHRS;

        // Set linear-acceleration switch variables
        gAlgorithm.linAccelSwitchCntr = 0;

#ifdef DISPLAY_DIAGNOSTIC_MSG
        if (magUsedInAlgorithm()) {
            TimingVars_DiagnosticMsg("Transitioning to low-gain AHRS mode");
        } else {
            TimingVars_DiagnosticMsg("Transitioning to low-gain VG mode");
        }
#endif

        gAlgoStatus.bit.highGain              = ( gAlgorithm.state == HIGH_GAIN_AHRS );
        gAlgoStatus.bit.attitudeOnlyAlgorithm = TRUE;
    }

}


/* Dynamic motion logic:
 *   0) When dynamicMotion is FALSE, remain in high-gain AHRS (do not decrement
 *      counter in 'HG_To_LG_Transition_Test')
 *   1) If dynamicMotion is selected then proceed to other filter states upon
 *      timeout (else, stay in HG mode)
 *   2) When in LG or INS mode... if dynamicMotion is set FALSE then transition
 *      to HG AHRS
 *   3) Once dynamicMotion is reset TRUE (by user), the system should begin
 *      transition to LG AHRS as if beginning from nominal startup
 */
void DynamicMotion(void)
{
    static BOOL enterStatic = FALSE;   // should this be true or false?

    /* If dynamicMotion is FALSE then transition to high-gain AHRS. The
     * system stays in HG until dynamicMotion is set high.
     */
    if (gAlgorithm.state > HIGH_GAIN_AHRS) {
        if (gAlgorithm.Behavior.bit.dynamicMotion) {
            enterStatic = FALSE;
        } else {
            if (enterStatic == FALSE) {
                enterStatic = TRUE;
                _DropToHighGainAHRS();

#ifdef DISPLAY_DIAGNOSTIC_MSG
                /* Question: what if DM flag is set (by user) at the moment of
                 * transition?  Should the FW initialize enterStatic
                 * in HG_To_LG_Transition_Test?
                 */
                TimingVars_DiagnosticMsg("Transitioning to High-Gain AHRS -- dynamic-motion flag set");
#endif
            }
        }
    }
}


//
static void _DropToHighGainAHRS(void)
{
    gAlgorithm.state      = HIGH_GAIN_AHRS;
    gAlgorithm.stateTimer = gAlgorithm.Duration.High_Gain_AHRS;

    gAlgoStatus.bit.highGain              = TRUE;
    gAlgoStatus.bit.attitudeOnlyAlgorithm = TRUE;

    // Reset flag in case drop is from INS
    gAlgorithm.insFirstTime = TRUE;
}


/******************************************************************************
* @name: _AccumulateFieldVectors Update the running sum of the acceleration and
* @brief magnetic field vectors (the accumulation variables are 64-bits long).
*        The total is averaged to form the system ICs.
*
* @brief called in main.cpp and processUpdateAlgorithm() in algorithm.cpp
* TRACE:
*
* @param N/A
* @retval 1 if magnetometers are used, otherwise it returns a zero.
******************************************************************************/
static BOOL _AccumulateFieldVectors(void)
{
    // Accumulate the acceleration vector readings (accels in g's)
    accumulatedAccelVector[X_AXIS] += (real)gEKFInputData.accel_B[X_AXIS];
    accumulatedAccelVector[Y_AXIS] += (real)gEKFInputData.accel_B[Y_AXIS];
    accumulatedAccelVector[Z_AXIS] += (real)gEKFInputData.accel_B[Z_AXIS];

    // Accumulate the gyroscope vector readings (accels in rad/s)
    accumulatedGyroVector[X_AXIS] += gEKFInputData.angRate_B[X_AXIS];
    accumulatedGyroVector[Y_AXIS] += gEKFInputData.angRate_B[Y_AXIS];
    accumulatedGyroVector[Z_AXIS] += gEKFInputData.angRate_B[Z_AXIS];

    // Accumulate the magnetic-field vector readings (or set to zero if the
    //   product does not have magnetometers)
    if (magUsedInAlgorithm() )
    {
        accumulatedMagVector[X_AXIS] += (real)gEKFInputData.magField_B[X_AXIS];
        accumulatedMagVector[Y_AXIS] += (real)gEKFInputData.magField_B[Y_AXIS];
        accumulatedMagVector[Z_AXIS] += (real)gEKFInputData.magField_B[Z_AXIS];
    } else {
        accumulatedMagVector[X_AXIS] = (real)0.0;
        accumulatedMagVector[Y_AXIS] = (real)0.0;
        accumulatedMagVector[Z_AXIS] = (real)0.0;
    }

   return(magUsedInAlgorithm());		
}


/******************************************************************************
* @name: _AverageFieldVectors Average the accumulated field vectors by shifting
*        the sum to the right
*        Note: the number of samples that are summed must be a multiple of 2:
*       Number of points accumulated, N = 2^bitsToShift
*
* TRACE:
*
* @param [in] bitsToShift
* @brief global data structure changes:
* Input:  gKalmanFilter.AccumulatedAccelVector
*         gKalmanFilter.AccumulatedMagVector
* Output: gKalmanFilter.AveragedAccelVector
*         gKalmanFilter.AveragedMagVector
* @retval 1 if magnetometers are used, otherwise it returns a zero.
******************************************************************************/
static BOOL _AverageFieldVectors(uint16_t pointsToAverage)
{
    real mult = (real)(1.0 / (real)pointsToAverage);

    // Average the accumulated acceleration vector
    averagedAccelVector[X_AXIS] = accumulatedAccelVector[X_AXIS] * mult;
    averagedAccelVector[Y_AXIS] = accumulatedAccelVector[Y_AXIS] * mult;
    averagedAccelVector[Z_AXIS] = accumulatedAccelVector[Z_AXIS] * mult;

    // Average the accumulated angular rate vector
    averagedGyroVector[X_AXIS] = accumulatedGyroVector[X_AXIS] * mult;
    averagedGyroVector[Y_AXIS] = accumulatedGyroVector[Y_AXIS] * mult;
    averagedGyroVector[Z_AXIS] = accumulatedGyroVector[Z_AXIS] * mult;

    /* Average the accumulated magnetic-field vector (or set to zero if
     * magnetometer is not in use.)
     */
    if (magUsedInAlgorithm())
    {
        averagedMagVector[X_AXIS] = accumulatedMagVector[X_AXIS] * mult;
        averagedMagVector[Y_AXIS] = accumulatedMagVector[Y_AXIS] * mult;
        averagedMagVector[Z_AXIS] = accumulatedMagVector[Z_AXIS] * mult;
    }
    else
    {
        averagedMagVector[X_AXIS] = (real)0.0;
        averagedMagVector[Y_AXIS] = (real)0.0;
        averagedMagVector[Z_AXIS] = (real)0.0;
    }

    return (magUsedInAlgorithm());
}


//
static void _ResetAlgorithm(void)
{
    int elemNum;

    // Reset P
    memset(gKalmanFilter.P, 0, sizeof(gKalmanFilter.P));
    float s1 = 1.0f;
    float s2 = 0.01f; // s1=0.01, s2=0.1, HG=160sec can make ini_pitch=40 stable ,DXG
    gKalmanFilter.P[STATE_Q0][STATE_Q0] = s1 * (real)INIT_P_Q;
    gKalmanFilter.P[STATE_Q1][STATE_Q1] = s1 * (real)INIT_P_Q;
    gKalmanFilter.P[STATE_Q2][STATE_Q2] = s1 * (real)INIT_P_Q;
    gKalmanFilter.P[STATE_Q3][STATE_Q3] = s1 * (real)INIT_P_Q;

    gKalmanFilter.P[STATE_WBX][STATE_WBX] = s2 * (real)INIT_P_WB;
    gKalmanFilter.P[STATE_WBY][STATE_WBY] = s2 * (real)INIT_P_WB;
    gKalmanFilter.P[STATE_WBZ][STATE_WBZ] = s2 * (real)INIT_P_WB;


    // Reset the rate-bias and corrected-rate variables (in the body-frame)
    for (elemNum = X_AXIS; elemNum <= Z_AXIS; elemNum++)
    {
        /* Initialize gyro rate bias with averaged gyro output
         * If averaged gyro output is above 1deg/s, this means there should be rotation
         * during initializatoin and it cannot be considered as bias. A default zero bias
         * is used instead.
         */
        if (fabs(averagedGyroVector[elemNum]) < ONE_DEGREE_IN_RAD)
        {
            gKalmanFilter.rateBias_B[elemNum] = (real)averagedGyroVector[elemNum];
        }
        else
        {
            gKalmanFilter.rateBias_B[elemNum] = 0.0;
        }

        gKalmanFilter.correctedRate_B[elemNum] = (real)0.0;
    }


    GenerateProcessJacobian();
    GenerateProcessCovariance();
}

