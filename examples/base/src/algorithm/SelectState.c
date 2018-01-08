/*
 * File:   SelectState.cpp
 * Author: joemotyka
 *
 * Created on May 8, 2016, 12:35 AM
 */

#include "GlobalConstants.h"

#include <string.h>   // memset
#include <math.h>     // fabs

#include "xbowsp_algorithm.h"        // gAlgorithm
#include "xbowsp_generaldrivers.h"   // gCalibration

#include "Indices.h"      // IND
#include "StateIndices.h" // STATE_IND
#include "AlgorithmLimits.h"       // LIMIT
#include "TimingVars.h"   // timer

#include "VectorMath.h"          // VectorNormalize
#include "QuaternionMath.h"      // EulerAnglesToQuaternion, QuaternionToEulerAngles
#include "TransformationMath.h"  // FieldVectorsToEulerAngles

#include "EKF_Algorithm.h"
#include "PredictFunctions.h"

#include "MagAlign.h"
#include "GpsData.h"  // gGpsData

#include "WorldMagneticModel.h"

// Local functions
static int _AccumulateFieldVectors(void);
static int _AverageFieldVectors(uint16_t pointsToAverage);

static void _DropToHighGainAHRS(void);
static void _ResetAlgorithm(void);

static uint8_t _InitINSFilter(void);

// StabilizeSystem.m
//
// Run for a prescribed period to let the sensors settle. Probably not
//   needed but has been carried over from the 440 algorithm.
void StabilizeSystem(void)
{
    // Decrement timer (initial value is set to the calling frequency of the EKF)
    gAlgorithm.stateTimer = gAlgorithm.stateTimer - 1;

    // Upon timeout prepare for transition to the next stage of the EKF
    //   (initialization) by resetting the state and state-timer and
    //   initializing the accumulation vectors.
    if (gAlgorithm.stateTimer == 0) {
        // Set new state and timer
        gAlgorithm.state      = INITIALIZE_ATTITUDE;
        gAlgorithm.stateTimer = gAlgorithm.Duration.Initialize_Attitude;

        // Initialize the vectors
        gAlgorithm.accumulatedAccelVector[X_AXIS] = 0.0;
        gAlgorithm.accumulatedAccelVector[Y_AXIS] = 0.0;
        gAlgorithm.accumulatedAccelVector[Z_AXIS] = 0.0;

        gAlgorithm.accumulatedMagVector[X_AXIS] = 0.0;
        gAlgorithm.accumulatedMagVector[Y_AXIS] = 0.0;
        gAlgorithm.accumulatedMagVector[Z_AXIS] = 0.0;

#ifdef DISPLAY_DIAGNOSTIC_MSG
        TimingVars_DiagnosticMsg("Transitioning to initialization mode");
#endif
    }

    // Set the bit to indicate initialization
    gAlgorithm.bitStatus.swAlgBIT.bit.initialization = TRUE;
    gAlgorithm.bitStatus.swStatus.bit.algorithmInit  = TRUE;
}


//
void InitializeAttitude(void)
{
    // Decrement timer at 100 Hz
    gAlgorithm.stateTimer = gAlgorithm.stateTimer - 1;

    /// Sum the acceleration and magnetic-field vectors (from the end of the
    ///   initialization stage)
    _AccumulateFieldVectors();

    // Quasi-static check: check for motion over threshold. If detected, reset the
    //                     accumulation variables and restart initialization phase
    if ((fabs(gAlgorithm.scaledSensors[XRATE]) > LIMIT_QUASI_STATIC_STARTUP_RATE) ||
        (fabs(gAlgorithm.scaledSensors[YRATE]) > LIMIT_QUASI_STATIC_STARTUP_RATE) ||
        (fabs(gAlgorithm.scaledSensors[ZRATE]) > LIMIT_QUASI_STATIC_STARTUP_RATE))
    {
        gAlgorithm.accumulatedAccelVector[X_AXIS] = 0.0;
        gAlgorithm.accumulatedAccelVector[Y_AXIS] = 0.0;
        gAlgorithm.accumulatedAccelVector[Z_AXIS] = 0.0;

        gAlgorithm.accumulatedMagVector[X_AXIS] = 0.0;
        gAlgorithm.accumulatedMagVector[Y_AXIS] = 0.0;
        gAlgorithm.accumulatedMagVector[Z_AXIS] = 0.0;

        gAlgorithm.stateTimer = gAlgorithm.Duration.Initialize_Attitude;
    }

    // Timeout...  Prepare for the transition to the next stage of the EKF
    //   (High-Gain AHRS) then determine the system's Initial Conditions by
    //   averaging the accumulated vectors.
    if (gAlgorithm.stateTimer == 0) {
#ifdef DISPLAY_DIAGNOSTIC_MSG
        if (gConfiguration.userBehavior.bit.useMags) {
            TimingVars_DiagnosticMsg("Transitioning to high-gain AHRS mode");
        } else {
            TimingVars_DiagnosticMsg("Transitioning to high-gain VG mode");
        }
#endif

        // Set new state and timer
        gAlgorithm.state      = HIGH_GAIN_AHRS;
        gAlgorithm.stateTimer = gAlgorithm.Duration.High_Gain_AHRS;

        // Average acceleration and magnetic field-vectors to determine the
        //   initial attitude of the system.
        _AverageFieldVectors(gAlgorithm.Duration.Initialize_Attitude);

// ************ Compute the measured Euler Angles and associated quaternion ************
        // Compute Euler angles from averaged field vectors (correct for hard/soft-iron effects)
        //   'usePredFlag' is set FALSE below indicating that the measured Euler angles will be
        //   used to 'level' the magnetometer readings.
        FieldVectorsToEulerAngles( &gAlgorithm.averagedAccelVector[0],
                                   gAlgorithm.averagedMagVector,
                                   FALSE,
                                   gKalmanFilter.measuredEulerAngles );

        // Initial attitude quaternion is generated using Euler angles from
        //   averaged gravity and magnetic fields. (DEBUG: This is used to
        //   initialize the EKF state)
        EulerAnglesToQuaternion( gKalmanFilter.measuredEulerAngles,
                                 gKalmanFilter.quaternion );

        // Euler angles from the initial measurement (DEBUG: initial output of
        //   the system)
        QuaternionToEulerAngles( gKalmanFilter.eulerAngles,
                                 gKalmanFilter.quaternion );
// ************ Compute the measured Euler Angles and associated quaternion ************

        // Initialize the Kalman filter variables
        _ResetAlgorithm();

        // Set linear-acceleration switch variables
        gAlgorithm.linAccelSwitchCntr = 0;
//        gAlgorithm.aMagThreshold      = (real)0.008;

        /// Update the system status
        gAlgorithm.bitStatus.swAlgBIT.bit.initialization        = FALSE;
        gAlgorithm.bitStatus.swStatus.bit.algorithmInit         = FALSE;
        gAlgorithm.bitStatus.swStatus.bit.highGain              = TRUE;
        gAlgorithm.bitStatus.swStatus.bit.attitudeOnlyAlgorithm = TRUE;
    }
}


//
void HG_To_LG_Transition_Test(void)
{
    // Decrement timer if 'dynamicMotion' TRUE (setting FALSE will cause the
    //   system to revert to high - gain mode once out of high - gain mode -- need
    //   to set flag high to transition out of high - gain mode once this is done)

    // dynamic-motion flag switch from high-gain to low-gain AHRS. if not set
    //   timer will not decrement the transition to LG AHRS will not occur.
    //   set at system configuration or (Nav-View) interface
    if (gConfiguration.userBehavior.bit.dynamicMotion) {
        gAlgorithm.stateTimer--;
    }

    // Startup check (if the estimated bias is large the software never
    //   transitions to the LG AHRS mode. NOTE: this seems incorrect, instead
    //   the SW should check if the bias has converged, not if it is above a
    //   threshold -- it is possible that the system could have a large bias.)
    //    However, this seems wrong too
    if ((fabs(gKalmanFilter.rateBias_B[X_AXIS]) > TEN_DEGREES_IN_RAD) &&
        (fabs(gKalmanFilter.rateBias_B[Y_AXIS]) > TEN_DEGREES_IN_RAD) &&
        (fabs(gKalmanFilter.rateBias_B[Z_AXIS]) > TEN_DEGREES_IN_RAD))
    {
        gAlgorithm.stateTimer = gAlgorithm.Duration.High_Gain_AHRS;
    }

    // Timeout...  Prepare for the transition to the next stage of the EKF
    //   (Low-Gain AHRS) and populate the values in Q that do not change with
    //   each iteration.
    if (gAlgorithm.stateTimer == 0) {
#ifdef DISPLAY_DIAGNOSTIC_MSG
        if (gConfiguration.userBehavior.bit.useMags) {
            TimingVars_DiagnosticMsg("Transitioning to low-gain AHRS mode");
        } else {
            TimingVars_DiagnosticMsg("Transitioning to low-gain VG mode");
        }
#endif

        gAlgorithm.state      = LOW_GAIN_AHRS;
        gAlgorithm.stateTimer = gAlgorithm.Duration.Low_Gain_AHRS;

        gAlgorithm.bitStatus.swStatus.bit.highGain = FALSE;

        // Populate Q (constant values) for the low-gain state (only a few variables changed)
        GenerateProcessCovariance();
    }
}


// This logic is only called until transition to INS from LG_AHRS then it is
//   not called unless the algorithm reverts back to HG_AHRS, which will
//   cause the system to pass through LG_AHRS on its way to INS.
void LG_To_INS_Transition_Test(void)
{
#ifdef DISPLAY_DIAGNOSTIC_MSG
    // Display the diagnostic message once upon transition (DEBUG: Remove in
    //   firmware)
    static int oneTime = TRUE;
#endif

    int velGoodForTrans;

    if (gAlgorithm.stateTimer > 0) {
        // Stay in LG mode until timeout occurs then begin check for INS
        //   transition
        gAlgorithm.stateTimer = gAlgorithm.stateTimer - 1;
    } else {
        // Upon timeout, begin check for INS transition (remove msg in firmware)
#ifdef DISPLAY_DIAGNOSTIC_MSG
        if (oneTime) {
            TimingVars_DiagnosticMsg("Begin check for INS transition");
            oneTime = FALSE;
        }
#endif

        // Can have external gps so "hasGps" is not a valid check
        if (gConfiguration.userBehavior.bit.useGPS) {
            // Check for valid GPS:
            //   1) fix/DOP/sync
            //   2) hasGPS
            //   if( (gGpsDataPtr->GPSFix == 0)          && (gGpsDataPtr->GPSProtocol == SIRF_BINARY) || // SiRF 0 = fix non-zero less than ideal fix
            //       (gGpsDataPtr->HDOP < hdopThreshold) && (gGpsDataPtr->GPSProtocol == NMEA_TEXT)   ||
            //       (gGpsDataPtr->GPSFix == 0)          && (gGpsDataPtr->GPSProtocol == NOVATEL_BINARY) ) {  // NMEA > 0 = fix, zero no fix
            // getPpsFlag() == false


            // If GPS output is valid (GPS providing data with a good signal lock)
            //   then perform calculations to determine if transition is possible
            if (gGpsDataPtr->gpsValid) {
                // Sync the algorithm and GPS ITOW
                gAlgorithm.itow = gGpsDataPtr->itow;

                // Calculate the in-plane velocity from GPS and determine if it
                //   exceeds a threshold
                real speedSq = (real)gGpsDataPtr->vNed[X_AXIS] * (real)gGpsDataPtr->vNed[X_AXIS] +
                               (real)gGpsDataPtr->vNed[Y_AXIS] * (real)gGpsDataPtr->vNed[Y_AXIS];

                if (speedSq >= LIMIT_GPS_VELOCITY_SQ) {
                    velGoodForTrans = TRUE;   // Able to get heading information from GPS data
                    gAlgorithm.timeOfLastSufficientGPSVelocity = (int32_t)gGpsDataPtr->itow;
                } else {
                    velGoodForTrans = FALSE;
                }

                // Mag readings are available if the system has mags and the useMags
                //   switch is set.
                int magReadingsPossible = gCalibration.productConfiguration.bit.hasMags &&
                                           gConfiguration.userBehavior.bit.useMags;

                // The system is able to calculate heading if either:
                //   1) magnetic field readings are available
                //   2) GPS velocity is sufficiently large
                int canCalcHeading = magReadingsPossible || velGoodForTrans;

                // Declination measurement is needed when using the magnetometer
                //   with the GPS.  If using GPS without magnetometers then the
                //   declination calculation is not needed.
                // ITOW is count in milliseconds
                //
                // Check for declination
                //   1) A current declination requires solution from the WMM and
                //      a valid GPS
                //   2) declination is not required when ...
                int32_t dt;
                if( gWorldMagModel.validSoln ) {
                    dt = (long)(gGpsDataPtr->itow - gWorldMagModel.timeOfLastSoln );
                    if (dt < 0) {
                        dt = dt + MAX_ITOW;
                    }
                } else {
                    // Soln invalid; set time to a large value
                    dt = LIMIT_DECL_EXPIRATION_TIME + 1;
                }
                int declMeasCurrent = ( dt <= LIMIT_DECL_EXPIRATION_TIME );
// ) && gWorldMagModel.validSoln;
                int needDeclMeas = magReadingsPossible &&
                                    gConfiguration.userBehavior.bit.useGPS;   // <-- check is done above

                // Can transition to INS if:
                //   1) GPS is being used
                //   2) the GPS signal is valid (according to ...)
                //   3) the system is able to compute heading from mags or GPS
                //   4) the system is not freely integrating
                //   5) the system is not performing a magnetic-alignment
                //      procedure
                if ( canCalcHeading &&
                     ( declMeasCurrent || needDeclMeas == FALSE) &&
                     !gConfiguration.userBehavior.bit.freeIntegrate &&
                     ( gAlgorithm.calState == MAG_ALIGN_STATUS_TERMINATION ||
                       gAlgorithm.calState == MAG_ALIGN_STATUS_IDLE ) )
                {
#ifdef DISPLAY_DIAGNOSTIC_MSG
                    TimingVars_DiagnosticMsg("Transitioning to INS mode");
#endif
                    gAlgorithm.state = INS_SOLUTION;

                    // We have a good GPS reading now - set this variable so we don't drop INS right away
                    gAlgorithm.timeOfLastGoodGPSReading = gGpsDataPtr->itow;

                    // Prepare for INS
                    _InitINSFilter();

                    // Set linear-acceleration switch variables
                    gAlgorithm.linAccelSwitchCntr = 0;

                    // Save off vel time
                    //gpsVelTime         = gGpsDataPtr->itow;
                    //lastGoodGPSVelTime = gGpsDataPtr->itow;
                }
            }
        }
    }
}


#define  INIT_P_Q    1.0e-5;
#define  INIT_P_WB   1.0e-5;
#define  INIT_P_INS  1.0e-3;

//
static uint8_t _InitINSFilter(void)
{
    // Local variables for _InitINSFilter() - pulling these variables from inside
    // the funtion to here, prevents the Hard Fault Exception seen in stm32f2xx_it.c - WHY?
//    real tmpQ[4][4], tmpWB[3][3];
    real tmp[7][7];

    // Calculate position from GPS data
  //gGpsDataPtr->Position[X_AXIS];

    gKalmanFilter.Position_N[X_AXIS] = 0.0;  // GPS position goes here?
    gKalmanFilter.Position_N[Y_AXIS] = 0.0;
    gKalmanFilter.Position_N[Z_AXIS] = 0.0;

    gKalmanFilter.Velocity_N[X_AXIS] = (real)gGpsDataPtr->vNed[X_AXIS];
    gKalmanFilter.Velocity_N[Y_AXIS] = (real)gGpsDataPtr->vNed[Y_AXIS];
    gKalmanFilter.Velocity_N[Z_AXIS] = (real)gGpsDataPtr->vNed[Z_AXIS];

    gKalmanFilter.accelBias_B[X_AXIS] = 0.0;
    gKalmanFilter.accelBias_B[Y_AXIS] = 0.0;
    gKalmanFilter.accelBias_B[Z_AXIS] = 0.0;

    // Extract the Quaternion and rate-bias values from the matrix before
    //   resetting
    int rowNum, colNum;

    // Save off the quaternion and rate-bias covariance values
    for (rowNum = Q0; rowNum <= Q3 + Z_AXIS + 1; rowNum++) {
        for (colNum = Q0; colNum <= Q3 + Z_AXIS + 1; colNum++) {
            tmp[rowNum][colNum] = gKalmanFilter.P[rowNum + STATE_Q0][colNum + STATE_Q0];
        }
    }

//    // Save off the quaternion covariance values
//    for (rowNum = Q0; rowNum <= Q3 + Z_AXIS; rowNum++) {
//        for (colNum = Q0; colNum <= Q3 + Z_AXIS; colNum++) {
//            tmpQ[rowNum][colNum] = gKalmanFilter.P[rowNum + STATE_Q0][colNum + STATE_Q0];
//        }
//    }
//
//    // Save off the rate-bias covariance values
//    for (rowNum = X_AXIS; rowNum <= Z_AXIS; rowNum++) {
//        for (colNum = X_AXIS; colNum <= Z_AXIS; colNum++) {
//            tmpWB[rowNum][colNum] = gKalmanFilter.P[rowNum + STATE_WBX][colNum + STATE_WBX];
//        }
//    }

    // Reset P
    memset(gKalmanFilter.P, 0, sizeof(gKalmanFilter.P));
    for (rowNum = 0; rowNum < NUMBER_OF_EKF_STATES; rowNum++) {
        gKalmanFilter.P[rowNum][rowNum] = (real)INIT_P_INS;
    }


    // Repopulate the P matrix with the quaternion and rate-bias values
    for (rowNum = Q0; rowNum <= Q3 + Z_AXIS + 1; rowNum++) {
        for (colNum = Q0; colNum <= Q3 + Z_AXIS + 1; colNum++) {
            gKalmanFilter.P[rowNum + STATE_Q0][colNum + STATE_Q0] = tmp[rowNum][colNum];
        }
    }

//    // Repopulate the P matrix with the quaternion and rate-bias values
//    for (rowNum = Q0; rowNum <= Q3; rowNum++) {
//        for (colNum = Q0; colNum <= Q3; colNum++) {
//            gKalmanFilter.P[rowNum + STATE_Q0][colNum + STATE_Q0] = tmpQ[rowNum][colNum];
//        }
//    }
//
//    for (rowNum = X_AXIS; rowNum <= Z_AXIS; rowNum++) {
//        for (colNum = X_AXIS; colNum <= Z_AXIS; colNum++) {
//            gKalmanFilter.P[rowNum + STATE_WBX][colNum + STATE_WBX] = tmpWB[rowNum][colNum];
//        }
//    }

return 1;

}


//INS_To_AHRS_Transition_Test.m
//
// DROP TO AHRS IF ...
//   1) GPS drops out for more than 3 seconds
//   2) magnetometer data not available AND at rest too long
//   3) magnetic alignment being performed
void INS_To_AHRS_Transition_Test(void)
{
    int magReadingsPossible = gCalibration.productConfiguration.bit.hasMags &&
                               gConfiguration.userBehavior.bit.useMags;

    // Determine the length of time it has been since the system 'moved' --
    //   only linear motion considered (rotations ignored).
    int32_t timeSinceRestBegan = (int32_t)gAlgorithm.itow - gAlgorithm.timeOfLastSufficientGPSVelocity;
    if (timeSinceRestBegan < 0) {
        timeSinceRestBegan = timeSinceRestBegan + MAX_ITOW;
    }

    // compute time since the last good GPS reading
    int32_t timeSinceLastGoodGPSReading = (int32_t)gAlgorithm.itow - gAlgorithm.timeOfLastGoodGPSReading;
    if (timeSinceLastGoodGPSReading < 0) {
        timeSinceLastGoodGPSReading = timeSinceLastGoodGPSReading + MAX_ITOW;
    }

    if ( ( timeSinceLastGoodGPSReading > gAlgorithm.Limit.Max_GPS_Drop_Time ) ||
         ( !magReadingsPossible && (timeSinceRestBegan > LIMIT_MAX_REST_TIME_BEFORE_DROP_TO_AHRS)) ||
         !( gAlgorithm.calState == MAG_ALIGN_STATUS_TERMINATION ||
            gAlgorithm.calState == MAG_ALIGN_STATUS_IDLE ) )
    {
        // Currently in INS mode but requiring a transition to AHRS / VG
        gAlgorithm.insFirstTime = TRUE;

        // The transition from INS to AHRS and back to INS does not seem to
        //   generate a stable solution if we transition to LG AHRS for only 30
        //   seconds.The interval needs to be longer(~1min).However, to
        //   mitigate any unforseen issues with the transition, HG AHRS with the
        //   nominal timing(1 min in HG, 30 seconds in LG) will be selected.
        gAlgorithm.state      = LOW_GAIN_AHRS;            // HIGH_GAIN_AHRS;
        gAlgorithm.stateTimer = gAlgorithm.Duration.Low_Gain_AHRS;   // gAlgorithm.Duration.High_Gain_AHRS;

        // Set linear-acceleration switch variables
        gAlgorithm.linAccelSwitchCntr = 0;

#ifdef DISPLAY_DIAGNOSTIC_MSG
        if (gConfiguration.userBehavior.bit.useMags) {
            TimingVars_DiagnosticMsg("Transitioning to high-gain AHRS mode");
        } else {
            TimingVars_DiagnosticMsg("Transitioning to high-gain VG mode");
        }
#endif

        gAlgorithm.bitStatus.swStatus.bit.highGain              = ( gAlgorithm.state == HIGH_GAIN_AHRS );
        gAlgorithm.bitStatus.swStatus.bit.attitudeOnlyAlgorithm = TRUE;
    }

}


// Dynamic motion logic:
//   0) When dynamicMotion is FALSE, remain in high-gain AHRS (do not decrement
//      counter in 'HG_To_LG_Transition_Test')
//   1) If dynamicMotion is selected then proceed to other filter states upon
//      timeout (else, stay in HG mode)
//   2) When in LG or INS mode... if dynamicMotion is set FALSE then transition
//      to HG AHRS
//   3) Once dynamicMotion is reset TRUE (by user), the system should begin
//      transition to LG AHRS as if beginning from nominal startup
void DynamicMotion(void)
{
    static int enterStatic = FALSE;   // should this be true or false?

    // If dynamicMotion is FALSE then transition to high-gain AHRS. The
    //   system stays in HG until dynamicMotion is set high.
    if (gAlgorithm.state > HIGH_GAIN_AHRS) {
        if (gConfiguration.userBehavior.bit.dynamicMotion) {
            enterStatic = FALSE;
        } else {
            if (enterStatic == FALSE) {
                enterStatic = TRUE;
                _DropToHighGainAHRS();

#ifdef DISPLAY_DIAGNOSTIC_MSG
                // Question: what if DM flag is set (by user) at the moment of
                //           transition?  Should the FW initialize enterStatic
                //           in HG_To_LG_Transition_Test?
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

    gAlgorithm.bitStatus.swStatus.bit.highGain              = TRUE;
    gAlgorithm.bitStatus.swStatus.bit.attitudeOnlyAlgorithm = TRUE;

    // Reset flag in case drop is from INS
    gAlgorithm.insFirstTime = TRUE;
}


/** ****************************************************************************
* @name: _AccumulateFieldVectors Update the running sum of the acceleration and
* @brief magnetic field vectors (the accumulation variables are 64-bits long).
*        The total is averaged to form the system ICs.
*
* @brief called in main.cpp and processUpdatAlgorithm() in algorithm.cpp
* TRACE:
*
* @param N/A
* @retval 1 if magnetometers are used, otherwise it returns a zero.
******************************************************************************/
static int _AccumulateFieldVectors(void)
{
    // Accumulate the acceleration vector readings (accels in g's)
    gAlgorithm.accumulatedAccelVector[X_AXIS] += (real)gAlgorithm.scaledSensors[XACCEL];
    gAlgorithm.accumulatedAccelVector[Y_AXIS] += (real)gAlgorithm.scaledSensors[YACCEL];
    gAlgorithm.accumulatedAccelVector[Z_AXIS] += (real)gAlgorithm.scaledSensors[ZACCEL];

    // Accumulate the magnetic-field vector readings (or set to zero if the
    //   product does not have magnetometers)
    if ( gCalibration.productConfiguration.bit.hasMags &&
         gConfiguration.userBehavior.bit.useMags )
    {
        gAlgorithm.accumulatedMagVector[X_AXIS] += (real)gAlgorithm.scaledSensors[XMAG];
        gAlgorithm.accumulatedMagVector[Y_AXIS] += (real)gAlgorithm.scaledSensors[YMAG];
        gAlgorithm.accumulatedMagVector[Z_AXIS] += (real)gAlgorithm.scaledSensors[ZMAG];
    } else {
        gAlgorithm.accumulatedMagVector[X_AXIS] = (real)0.0;
        gAlgorithm.accumulatedMagVector[Y_AXIS] = (real)0.0;
        gAlgorithm.accumulatedMagVector[Z_AXIS] = (real)0.0;
    }

    // Return a bit describing the magnetometer's existence
    return(gCalibration.productConfiguration.bit.hasMags);
}


/** ****************************************************************************
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
static int _AverageFieldVectors(uint16_t pointsToAverage)
{
    real mult = (real)(1.0 / (real)pointsToAverage);

    // Average the accumulated acceleration vector
    gAlgorithm.averagedAccelVector[X_AXIS] = gAlgorithm.accumulatedAccelVector[X_AXIS] * mult;
    gAlgorithm.averagedAccelVector[Y_AXIS] = gAlgorithm.accumulatedAccelVector[Y_AXIS] * mult;
    gAlgorithm.averagedAccelVector[Z_AXIS] = gAlgorithm.accumulatedAccelVector[Z_AXIS] * mult;

    // Average the accumulated magnetic-field vector (or set to zero if the
    //   product does not have magnetometers)
    if ( gCalibration.productConfiguration.bit.hasMags &&
         gConfiguration.userBehavior.bit.useMags )
    {
        gAlgorithm.averagedMagVector[X_AXIS] = gAlgorithm.accumulatedMagVector[X_AXIS] * mult;
        gAlgorithm.averagedMagVector[Y_AXIS] = gAlgorithm.accumulatedMagVector[Y_AXIS] * mult;
        gAlgorithm.averagedMagVector[Z_AXIS] = gAlgorithm.accumulatedMagVector[Z_AXIS] * mult;
    } else {
        gAlgorithm.averagedMagVector[X_AXIS] = 0.0;
        gAlgorithm.averagedMagVector[Y_AXIS] = 0.0;
        gAlgorithm.averagedMagVector[Z_AXIS] = 0.0;
    }

    // Return a bit describing the magnetometer's existence
    return(gCalibration.productConfiguration.bit.hasMags);
}


static void _ResetAlgorithm(void)
{
    int elemNum;
//    uint8_t endIndex;

//    if( gCalibration.productConfiguration.bit.hasMags &&
//       gConfiguration.userBehavior.bit.useMags )
//    {
//        endIndex = STATE_WBZ;
//    } else {
//        endIndex = STATE_WBZ - 1;
//    }

    // Reset P
    memset( &gKalmanFilter.P[0][0], 0, sizeof(gKalmanFilter.P) );
    gKalmanFilter.P[STATE_Q0][STATE_Q0] = (real)INIT_P_Q;
    gKalmanFilter.P[STATE_Q1][STATE_Q1] = (real)INIT_P_Q;
    gKalmanFilter.P[STATE_Q2][STATE_Q2] = (real)INIT_P_Q;
    gKalmanFilter.P[STATE_Q3][STATE_Q3] = (real)INIT_P_Q;

    gKalmanFilter.P[STATE_WBX][STATE_WBX] = (real)INIT_P_WB;
    gKalmanFilter.P[STATE_WBY][STATE_WBY] = (real)INIT_P_WB;
    gKalmanFilter.P[STATE_WBZ][STATE_WBZ] = (real)INIT_P_WB;

    // Reset the rate-bias and corrected-rate variables (in the body-frame)
    for (elemNum = X_AXIS; elemNum <= Z_AXIS; elemNum++) {
        gKalmanFilter.rateBias_B[elemNum]      = 0.0;
        gKalmanFilter.correctedRate_B[elemNum] = 0.0;
    }

    GenerateProcessJacobian();
    GenerateProcessCovariance();
}

