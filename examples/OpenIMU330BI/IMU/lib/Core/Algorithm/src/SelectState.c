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
#include "TimingVars.h"

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
        printf("To ini att. %u\n", gEKFInput.itow);
        #else
#ifdef DISPLAY_DIAGNOSTIC_MSG
        DebugPrintString("To ini att. ");
        DebugPrintInt("", gEKFInput.itow);
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
    if ((fabs(gEKFInput.angRate_B[X_AXIS]) > LIMIT_QUASI_STATIC_STARTUP_RATE) ||
        (fabs(gEKFInput.angRate_B[Y_AXIS]) > LIMIT_QUASI_STATIC_STARTUP_RATE) ||
        (fabs(gEKFInput.angRate_B[Z_AXIS]) > LIMIT_QUASI_STATIC_STARTUP_RATE))
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
        printf("To HG. %u\n", gEKFInput.itow);
        #else
#ifdef DISPLAY_DIAGNOSTIC_MSG
        DebugPrintString("To HG. ");
        DebugPrintInt("", gEKFInput.itow);
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
        gKalmanFilter.quaternion_Past[0] = gKalmanFilter.quaternion[0];
        gKalmanFilter.quaternion_Past[1] = gKalmanFilter.quaternion[1];
        gKalmanFilter.quaternion_Past[2] = gKalmanFilter.quaternion[2];
        gKalmanFilter.quaternion_Past[3] = gKalmanFilter.quaternion[3];
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
        printf("To LG. %u\n", gEKFInput.itow);
        #else
#ifdef DISPLAY_DIAGNOSTIC_MSG
        DebugPrintString("To LG. ");
        DebugPrintInt("", gEKFInput.itow);
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
        if ( gpsUsedInAlgorithm() && gEKFInput.gpsUpdate && gEKFInput.gpsFixType ) 
        {
            #ifdef INS_OFFLINE
            printf("To INS. %u\n", gEKFInput.itow);
            #else
#ifdef DISPLAY_DIAGNOSTIC_MSG
            DebugPrintString("To INS. ");
            DebugPrintInt("", gEKFInput.itow);
            DebugPrintEndline();
#endif
            #endif
#ifdef DISPLAY_DIAGNOSTIC_MSG
            TimingVars_DiagnosticMsg("Transitioning to INS mode");
#endif

            // Transit to INS solution
            gAlgorithm.state = INS_SOLUTION;
            
            // Initialize the algorithm with GNSS position and lever arm
            InitINSFilter();

            // Set linear-acceleration switch variables
            gAlgorithm.linAccelSwitchCntr = 0;
        }
    }
}

/* INS_To_AHRS_Transition_Test:  Drop back to LG AHRS operation if...
 *   1) GPS drops out for more than 3 seconds
 *   2) magnetometer data not available AND at rest too long
 *   3) magnetic alignment being performed
 */
void INS_To_AHRS_Transition_Test(void)
{
    // Record last GPS velocity large enough to give a good heading measurement
    if (gEKFInput.rawGroundSpeed >= LIMIT_MIN_GPS_VELOCITY_HEADING)
    {
        gAlgorithm.timeOfLastSufficientGPSVelocity = (int32_t)gEKFInput.itow;
    }
    /* Determine the length of time it has been since the system 'moved' --
     * only linear motion considered (rotations ignored).
     */
    int32_t timeSinceRestBegan = (int32_t)gEKFInput.itow - gAlgorithm.timeOfLastSufficientGPSVelocity;
    if (timeSinceRestBegan < 0)
    {
        timeSinceRestBegan = timeSinceRestBegan + MAX_ITOW;
    }
    if (timeSinceRestBegan > LIMIT_MAX_REST_TIME_BEFORE_HEADING_INVALID && gAlgorithm.headingIni != HEADING_UNINITIALIZED)
    {
        gAlgorithm.headingIni = HEADING_GNSS_LOW;
#ifdef DISPLAY_DIAGNOSTIC_MSG
        DebugPrintString("Rest for too long.");
        DebugPrintEndline();
#endif
    }

    // compute time since the last good GPS reading
    int32_t timeSinceLastGoodGPSReading = (int32_t)gAlgorithm.itow - gAlgorithm.timeOfLastGoodGPSReading;
    if (timeSinceLastGoodGPSReading < 0) {
        timeSinceLastGoodGPSReading = timeSinceLastGoodGPSReading + MAX_ITOW;
    }

    if ( timeSinceLastGoodGPSReading > gAlgorithm.Limit.maxGpsDropTime )
    {
#ifdef INS_OFFLINE
        printf("GPS outage too long\n");
#endif // INS_OFFLINE

        // Currently in INS mode but requiring a transition to AHRS / VG
        gAlgorithm.insFirstTime = TRUE;
        gAlgorithm.headingIni = HEADING_UNINITIALIZED;

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
    accumulatedAccelVector[X_AXIS] += (real)gEKFInput.accel_B[X_AXIS];
    accumulatedAccelVector[Y_AXIS] += (real)gEKFInput.accel_B[Y_AXIS];
    accumulatedAccelVector[Z_AXIS] += (real)gEKFInput.accel_B[Z_AXIS];

    // Accumulate the gyroscope vector readings (accels in rad/s)
    accumulatedGyroVector[X_AXIS] += gEKFInput.angRate_B[X_AXIS];
    accumulatedGyroVector[Y_AXIS] += gEKFInput.angRate_B[Y_AXIS];
    accumulatedGyroVector[Z_AXIS] += gEKFInput.angRate_B[Z_AXIS];

    // Accumulate the magnetic-field vector readings (or set to zero if the
    //   product does not have magnetometers)
    if (magUsedInAlgorithm() )
    {
        accumulatedMagVector[X_AXIS] += (real)gEKFInput.magField_B[X_AXIS];
        accumulatedMagVector[Y_AXIS] += (real)gEKFInput.magField_B[Y_AXIS];
        accumulatedMagVector[Z_AXIS] += (real)gEKFInput.magField_B[Z_AXIS];
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

