/*
 * File:   UpdateFunctions.cpp
 * Author: joemotyka
 *
 * Created on May 8, 2016, 12:35 AM
 */

#include <math.h>
#include <string.h>

#include "UpdateMatrixSizing.h"  // used to specify the size of the update vectors

#include "GlobalConstants.h"   // TRUE, FALSE, NUMBER_OF_EKF_STATES, ...
#include "UpdateFunctions.h"

#include "xbowsp_algorithm.h"
#include "xbowsp_generaldrivers.h"

#include "TimingVars.h"
#include "Indices.h"
#include "AlgorithmLimits.h"

#include "EKF_Algorithm.h"

#include "VectorMath.h"
#include "MatrixMath.h"

#include "QuaternionMath.h"
#include "TransformationMath.h"
#include "StateIndices.h"

#include "SensorNoiseParameters.h"

#include "GpsData.h"
#include "WorldMagneticModel.h"

#include "ucb_packet.h"   // for UcbGetSysRange()

#ifdef INS_OFFLINE
#include "c:\Projects\software\sim\INS380_Offline\INS380_Offline\SimulationParameters.h"
#endif

//#include "RunLengthEncoding.h"
// H is sparse and has elements in the following locations...
uint8_t RLE_H[ROWS_IN_H][2] = { {  6, 9 },
                                {  6, 9 },
                                {  6, 9 } };

// KxH is sparse with elements only in cols 6 through 9
uint8_t RLE_KxH[16][2] = { { 6, 9 }, { 6, 9 }, { 6, 9 },
                           { 6, 9 }, { 6, 9 }, { 6, 9 },
                           { 6, 9 }, { 6, 9 }, { 6, 9 }, { 6, 9 },
                           { 6, 9 }, { 6, 9 }, { 6, 9 },
                           { 6, 9 }, { 6, 9 }, { 6, 9 } };

// Local functions
static void _TurnSwitch_Q(void);
static void _TurnSwitch_EA(void);

static real _UnwrapAttitudeError( real attitudeError );
static real _LimitValue( real value, real limit );
static void _GPS_PosVel_To_NED(void);  // doesn't actually operate on velocity

// Functions to split the INS update across multiple iterations, so the update can
// complete in the required 10 ms
static void updateInsVel(void);
static void updateInsPos(void);
static void updateInsAtt(void);

// EKF_UpdateStage.m
void EKF_UpdateStage(void)
{
    // Perform a VG/AHRS update, regardless of GPS availability or health,
    //   when the state is HG AHRS or LG AHRS.  Once GPS becomes healthy
    //   (and the right conditions are met) perform an INS or reduced-
    //   order GPS update.
    if( gAlgorithm.state <= LOW_GAIN_AHRS )
    {
        // Update the AHRS solution at a 10 Hz update rate
        if( (timer.subFrameCntr == 0) && (timer.oneHundredHertzFlag == 1) ) {
            // The AHRS/VG solution is handled inside FieldVectorsToEulerAngles (called from the
            //   prediction function EKF_PredictionStage)

            // nu = z - h(xPred)
            ComputeSystemInnovation_AHRS_EA();
            Update_AHRS_EA();
        }
    } else {
        // GPS-type Updates (with magnetometers: true-heading = mag-heading + mag-decl)
        if ( gConfiguration.userBehavior.bit.useMags &&
             gCalibration.productConfiguration.bit.hasMags )
        {
            // The following variable enables the update to be broken up into two sequential
            //   calculations in two sucessive 100 Hz periods
            static int runInsUpdate = 0;

            // Perform the EKF update at 10 Hz (split nine mag-only updates for
            //   for every GPS/mag update)
            if( (timer.tenHertzCntr == 0) && (timer.subFrameCntr == 0) &&
                (timer.oneHundredHertzFlag == 1) && (gGpsDataPtr->gpsValid) )
            {   // Start INS calcs at 10 Hz mark 9
                // At 1 Hz mark, update when GPS data is valid, else do an AHRS-update
                runInsUpdate = 1;

                // Sync the algorithm itow to the GPS value (may place this elsewhere)
                gAlgorithm.itow = gGpsDataPtr->itow;

                // reset the "last good reading" time
                gAlgorithm.timeOfLastGoodGPSReading = gGpsDataPtr->itow;

                // Save off the Lat/Lon in radians and altitude in meters
                gAlgorithm.llaRad[X_AXIS] = ((double)gGpsDataPtr->latSign * gGpsDataPtr->lat) * D2R; // high precision from GPS
                gAlgorithm.llaRad[Y_AXIS] = ((double)gGpsDataPtr->lonSign * gGpsDataPtr->lon) * D2R;
                gAlgorithm.llaRad[Z_AXIS] = gGpsDataPtr->alt;

                // Extract what's common between the following function and the routines
                //   below so we aren't repeating calculations
                _GPS_PosVel_To_NED();

                real vSq = (real)(gGpsDataPtr->vNed[X_AXIS] * gGpsDataPtr->vNed[X_AXIS] +
                                  gGpsDataPtr->vNed[Y_AXIS] * gGpsDataPtr->vNed[Y_AXIS]);

                // Set in LG_To_INS_Transition_Test for AHRS operation
                if (vSq >= LIMIT_GPS_VELOCITY_SQ) {
                    gAlgorithm.timeOfLastSufficientGPSVelocity = (int32_t)gGpsDataPtr->itow;
                }

                // This Sequential-Filter (three-stage approach) is nearly as good as the full
                //   implementation -
                // we also can split it across multiple iterations to not exceed 10 ms execution
                // on the embedded 380
#if 1
                ComputeSystemInnovation_INS();  // z = meas, h = pred = q, nu - z - h
                updateInsPos();
                updateInsVel();
            }  // comment out (this and next line) to run all update functions in one loop
            else if ( (timer.tenHertzCntr == 0) && (timer.subFrameCntr == 1) && runInsUpdate ) {
                updateInsAtt();
                runInsUpdate = 0;  // set up for next pass
#else
                ComputeSystemInnovation_AHRS_Q();
                Update_AHRS_Q();
#endif
            } else if (timer.subFrameCntr == 0) { // AHRS at all the other 10 Hz marks
                // AHRS-like update
                ComputeSystemInnovation_AHRS_Q();
                Update_AHRS_Q();
            }

            // FIXME: MOVE THIS TO THE EKF CALL, AFTER THE UPDATE, SO THE LAT/LON
            //        IS COMPUTED FROM INTEGRATED DATA IF AN UPDATE IS NOT PERFORMED
            // Update LLA at 100/200 Hz
            if ( gGpsDataPtr->gpsValid && ( gAlgorithm.insFirstTime == FALSE ) ) {
                //r_E = Base_To_ECEF( &gKalmanFilter.Position_N[0], &gAlgorithm.rGPS0_E[0], &R_NinE[0][0] );    //
                double r_E[3];
                PosNED_To_PosECEF( &gKalmanFilter.Position_N[0], &gAlgorithm.rGPS0_E[0], &gAlgorithm.R_NinE[0][0], &r_E[0] );
                //                 100 Hz                        generated once          1 Hz                      100 Hz

                //gKalmanFilter.llaDeg[LAT_IDX] = ECEF2LLA( r_E );   // output variable (ned used for anything else); this is in [ deg, deg, m ]
                ECEF_To_LLA(&gKalmanFilter.llaDeg[0], &r_E[0]);
                //          100 Hz                    100 Hz
            }

    // Don't have this logic in the current code base
    //
    // //currently in navmode and need to make update
    // if ( (algorithm.timerITOW -lastEnoughGPSVelTime)>LONGER_STOP_FREE_HEADING_TIME &&
    //      !(calibration.productConfiguration.bit.mags && configuration.userBehavior.bit.useMags) &&
    //      velPacket &&
    //      speedSquared>YAW_TRACK_SPEED_SQ)
    //          initNavFilter(); // if stop period is a bit long and not mag is used,
    //                           //alogrithm heading needs to be re-initialized with GPS
    //                           // when getting engough GPS velocity

        } else {
            // Magnetometers are unavailable or unused

        }
//        % Magnetometers not available or unused
//
//        % Check for valid GPS signal
//        if( gGpsDataPtr.GPSValid ),
//            % Compute vSq and respond accordingly
//            vSq = gGpsDataPtr.vel(X_AXIS) * gGpsDataPtr.vel(X_AXIS) + ...
//                  gGpsDataPtr.vel(Y_AXIS) * gGpsDataPtr.vel(Y_AXIS);
//
//            % Update the GPS-Stuff at 1Hz
//            if( ( timing.subFrameCntr == 0 ) && ( timing.tenHertzCntr == 0 ) ),
//                GPSPosVelToNED;
//            end
//
//            if( vSq >= LIMIT.GPS_VELOCITY_SQ ),
//                % Sufficient GPS velocity, can use velocity to compute
//                %   heading (heading is also part of message)
//                timeOfLastSufficientGPSVelocity = itow;
//                yawLock_1stTimeFlag = TRUE;
//
//                % Update at the rate of GPS data availability (presently
//                %   1Hz)
//                if( ( timing.subFrameCntr == 0 ) && ( timing.tenHertzCntr == 0 ) ),
//                    % 1 Hz update -- Virtually no difference between the
//                    %   first two solutions
//                    if( 1 ),
//                        % Use GPS position, velocity, and yaw from track
//                        ComputeSystemInnovation_PosVelYaw;
//                        Update_PosVelYaw;
//                    elseif( 0 ),
//                        % Use GPS position, velocity, yaw from track, and
//                        %   accelerometer readings
//                        ComputeSystemInnovation_PosVelTiltYaw;
//                        Update_PosVelTiltYaw; % Need to use yaw from track!
//                    elseif( 0 ),
//                        % Doesn't seem to work
//                        ComputeSystemInnovation_Yaw;
//                        Update_Yaw; % Use yaw from GPS track
//                    elseif( 0 ),
//                        % Not promising either
//                        ComputeSystemInnovation_PosVel;
//                        Update_PosVel; % Need to use yaw from track!
//                    else
//                        ComputeSystemInnovation_Vel;
//                        nu = zeros(3,1);
//                        Update_Vel; % Need to use yaw from track!
//
//%                       ComputeSystemInnovation_Yaw;
//%                       Update_Yaw; % Use yaw from GPS track
//                    end
//                end
//            else
//                % GPS velocity is below threshold to perform update
//                yawRateSq = ( gKF.correctedRate_B(YAW) )^2;
//
//                if( gConfiguration.userBehavior.bit.stationaryYawLock && ...
//                    yawRateSq < LIMIT.YAW_RATE_SQ ),
//                    if( yawLock_1stTimeFlag ),
//                        yawLock_1stTimeFlag = FALSE;
//                        psi_yawLock = gKF.predictedEulerAngles(YAW);
//                    else
//                        % Perform a 10 Hz update split between Pos/Vel and
//                        %   Yaw
//                        if( timing.subFrameCntr == 0 ),
//                            if( ( timing.tenHertzCntr == 0 ) && ( gGpsDataPtr.GPSValid ) ),   % 1 Hz mark with valid GPS
//                                % 1 Hz update
//                                ComputeSystemInnovation_PosVel;
//                                Update_PosVel;
//                            else
//                                % 10 Hz update
//                                ComputeSystemInnovation_Yaw;
//                                Update_Yaw;  % need to use psi_yawLock
//                            end
//                        end
//                    end
//                else
//                    % Yaw-lock not selected OR Yaw-Rate sufficiently high
//                    %   (not stopped).  Perform a 1 Hz update (when GPS
//                    %   updated).
//                    if( ( timing.subFrameCntr == 0 ) && ( timing.tenHertzCntr == 0 ) ),
//                        ComputeSystemInnovation_PosVel;
//                        Update_PosVel;
//                    end
//
//                    yawLock_1stTimeFlag = 1;
//                end
//            end
//
//            % Update at what rate (100 Hz?)
//            tmpLLA = [ gGpsDataPtr.pos(X_AXIS) * D2R;
//                       gGpsDataPtr.pos(Y_AXIS) * D2R;
//                       gGpsDataPtr.pos(Z_AXIS) ];
//            RNinE = InvRneFromLLA( tmpLLA );   % matches the inverse calculation
//            r_E = Base2ECEF( gKF.Position_N, rGPS0_E, RNinE );    %
//            lla = ECEF2LLA( r_E );   % output variable (ned used for anything else)
//        else
//            % GPS is invalid (if GPS is invalid for longer than a database
//            %   limit (~3 seconds) then drop to AHRS, see
//            %   INS_To_AHRS_Transition_Test.m)
//            if( gConfiguration.userBehavior.bit.stationaryYawLock ),
//                % if stationaryYawLock === TRUE then 'hold' heading;
//                if( yawLock_1stTimeFlag ),
//                    % rationale: it doesn't make sense to do an update the
//                    %            first time as the error will be zero
//                    yawLock_1stTimeFlag = 0;
//                    psi_yawLock = gKF.predictedEulerAngles(YAW);
//                else
//                    % Update yaw at ten hertz (GPS readings unavailable)
//                    if( timing.subFrameCntr == 0 ),
//                        %ComputeSystemInnovation_Yaw;  % <-- uses gps velocity, wrong!
//                        % Compute the innovation (based on angle prior to yaw-lock)
//                        z = psi_yawLock;
//                        h = gKF.predictedEulerAngles(YAW);
//                        nu = z - h;
//
//                        Update_Yaw;
//                    end
//                end
//            else
//                % Yaw-Lock not enabled, let yaw drift until GPS recovers or
//                %   transition to AHRS occurs
//                yawLock_1stTimeFlag = 1;
//            end
//        end
//    end
    }
}


// ComputeSystemInnovation_AHRS.m
void ComputeSystemInnovation_AHRS_EA(void)
{
    // Compute the innovation, nu, between measured and predicted attitude.
    //   Correct for wrap-around. Then limit the error.
    // ----- Roll -----
    gKalmanFilter.nu[STATE_ROLL]  = gKalmanFilter.measuredEulerAngles[ROLL] -
                                    gKalmanFilter.eulerAngles[ROLL];
    gKalmanFilter.nu[STATE_ROLL]  = _UnwrapAttitudeError(gKalmanFilter.nu[STATE_ROLL]);
    gKalmanFilter.nu[STATE_ROLL]  = _LimitValue(gKalmanFilter.nu[STATE_ROLL],  (real)TEN_DEGREES_IN_RAD);

    // ----- Pitch -----
    gKalmanFilter.nu[STATE_PITCH] = gKalmanFilter.measuredEulerAngles[PITCH] -
                                    gKalmanFilter.eulerAngles[PITCH];
    gKalmanFilter.nu[STATE_PITCH] = _UnwrapAttitudeError(gKalmanFilter.nu[STATE_PITCH]);
    gKalmanFilter.nu[STATE_PITCH] = _LimitValue(gKalmanFilter.nu[STATE_PITCH], (real)TEN_DEGREES_IN_RAD);

    // ----- Yaw -----
    // Separating the two was meant to save time but it doesn't seem to.  However, keep for now.
    if( gConfiguration.userBehavior.bit.useMags ) {
        gKalmanFilter.nu[STATE_YAW]   = gKalmanFilter.measuredEulerAngles[YAW] -
                                        gKalmanFilter.eulerAngles[YAW];
        gKalmanFilter.nu[STATE_YAW]   = _UnwrapAttitudeError(gKalmanFilter.nu[STATE_YAW]);
        gKalmanFilter.nu[STATE_YAW]   = _LimitValue(gKalmanFilter.nu[STATE_YAW],   (real)TEN_DEGREES_IN_RAD);
    } else {
        gKalmanFilter.nu[STATE_YAW] = 0.0;
    }

    // When the filtered yaw-rate is above certain thresholds then reduce the
    //   attitude-errors used to update roll and pitch.
    _TurnSwitch_EA();
}


/** ****************************************************************************
* @name: _GenerateObservationJacobian_RollAndPitch roll and pitch elements of
*        the measurement Jacobian (H)
* @brief
* TRACE:
* @param N/A
* @retval 1
******************************************************************************/
uint8_t _GenerateObservationJacobian_AHRS_EA(void)
{
    real u = 0.0;
    real uNum = 0.0;
    real uDen = 0.0;
    real denom = 1.0;
    real multiplier = 0.0;

    // Set the values in DP to zero
    static int initH = TRUE;
    if( initH ) {
        initH = FALSE;
        memset(&gKalmanFilter.H[0][0], 0, sizeof(gKalmanFilter.H));
    }

    /// Note: H is 3x7
    /// Roll
    uNum = (real)2.0 * ( gKalmanFilter.quaternion[Q2] * gKalmanFilter.quaternion[Q3] +
                         gKalmanFilter.quaternion[Q0] * gKalmanFilter.quaternion[Q1] );
    uDen =  gKalmanFilter.quaternion[Q0] * gKalmanFilter.quaternion[Q0] + 
           -gKalmanFilter.quaternion[Q1] * gKalmanFilter.quaternion[Q1] +
           -gKalmanFilter.quaternion[Q2] * gKalmanFilter.quaternion[Q2] +
            gKalmanFilter.quaternion[Q3] * gKalmanFilter.quaternion[Q3];

    denom = uNum*uNum + uDen*uDen;
    if (denom < 1e-3) {
        denom = (real)1e-3;
    }
    multiplier = (real)2.0 / denom;

    /// Derivative of the roll-angle wrt quaternions
    gKalmanFilter.H[ROLL][STATE_Q0] = multiplier * ( uDen*gKalmanFilter.quaternion[Q1] +
                                                    -uNum*gKalmanFilter.quaternion[Q0]);
    gKalmanFilter.H[ROLL][STATE_Q1] = multiplier * ( uDen*gKalmanFilter.quaternion[Q0] + 
                                                     uNum*gKalmanFilter.quaternion[Q1]);
    gKalmanFilter.H[ROLL][STATE_Q2] = multiplier * ( uDen*gKalmanFilter.quaternion[Q3] + 
                                                     uNum*gKalmanFilter.quaternion[Q2]);
    gKalmanFilter.H[ROLL][STATE_Q3] = multiplier * ( uDen*gKalmanFilter.quaternion[Q2] +
                                                    -uNum*gKalmanFilter.quaternion[Q3]);

    /// Pitch
    u = (real)2.0 * ( gKalmanFilter.quaternion[Q1] * gKalmanFilter.quaternion[Q3] -
                      gKalmanFilter.quaternion[Q0] * gKalmanFilter.quaternion[Q2] );

    denom = sqrt((real)1.0 - u*u);
    if (denom < 1e-3) {
        denom = (real)1e-3;
    }
    multiplier = (real)2.0 / denom;

    gKalmanFilter.H[PITCH][STATE_Q0] = multiplier * ( gKalmanFilter.quaternion[Q2]);
    gKalmanFilter.H[PITCH][STATE_Q1] = multiplier * (-gKalmanFilter.quaternion[Q3]);
    gKalmanFilter.H[PITCH][STATE_Q2] = multiplier * ( gKalmanFilter.quaternion[Q0]);
    gKalmanFilter.H[PITCH][STATE_Q3] = multiplier * (-gKalmanFilter.quaternion[Q1]);

    /// Yaw
    uNum = (real)2.0 * ( gKalmanFilter.quaternion[Q1] * gKalmanFilter.quaternion[Q2] +
                         gKalmanFilter.quaternion[Q0] * gKalmanFilter.quaternion[Q3] );
    uDen = (real)1.0 - (real)2.0 * ( gKalmanFilter.quaternion[Q2] * gKalmanFilter.quaternion[Q2] +
                                     gKalmanFilter.quaternion[Q3] * gKalmanFilter.quaternion[Q3] );
    denom = uNum*uNum + uDen*uDen;
    if (denom < 1e-3) {
        denom = (real)1e-3;
    }
    multiplier = (real)2.0 / denom;

    /// Derivative of the yaw-angle wrt quaternions
    gKalmanFilter.H[YAW][STATE_Q0] = multiplier * ( uDen*gKalmanFilter.quaternion[Q3] +
                                                   -uNum*gKalmanFilter.quaternion[Q0]);
    gKalmanFilter.H[YAW][STATE_Q1] = multiplier * ( uDen*gKalmanFilter.quaternion[Q2] +
                                                   -uNum*gKalmanFilter.quaternion[Q1]);
    gKalmanFilter.H[YAW][STATE_Q2] = multiplier * ( uDen*gKalmanFilter.quaternion[Q1] + 
                                                    uNum*gKalmanFilter.quaternion[Q2]);
    gKalmanFilter.H[YAW][STATE_Q3] = multiplier * ( uDen*gKalmanFilter.quaternion[Q0] + 
                                                    uNum*gKalmanFilter.quaternion[Q3]);

    return 1;
}


// Specify whether to implement the yaw-switch by reducing the error or
//   increasing R.
#define  YAW_SWITCH_ORIG_METHOD

// GenerateObservationCovarianceMatrix_AHRS.m
void _GenerateObservationCovariance_AHRS_EA(void)
{
    //
    static real Rnom;

    // Only need to compute certain elements of R once
    static int initR = TRUE;
    if (initR) {
        initR = FALSE;

        // Clear the values in R (in AHRS mode, there are 3 rows in the Jacobian)
        // Initialize the Process Covariance (Q) matrix
        // DEBUG: Probably not needed
        memset(gKalmanFilter.R, 0, sizeof(gKalmanFilter.R));
        memset(gKalmanFilter.R_INS, 0, sizeof(gKalmanFilter.R_INS));

#ifdef INS_OFFLINE
        // This value is set based on the version string specified in the 
        //   simulation configuration file, ekfSim.cfg
        uint8_t sysRange = gSimulation.sysRange;
#else
        // This value is set based on the version string loaded into the unit
        //   via the system configuration load
        uint8_t sysRange = UcbGetSysRange(); // from system config
#endif

        // Set the matrix, R, based on whether the system is in high or low-gain
        //   (is the acceleration above or below the acceleration threshold)
        //
        //   R-values are based on the variance of the roll and pitch angles
        //   generated from the sensor noise passed through the measurement
        //   model.  The values are multiplied by dt^2 as well as vary with the
        //   angle.  The value can be made large enough to work with all angles
        //   but it may slow the response.
        //
        // These values are found by passing the accelerometer VRW values
        //   (determined from a very limited data set) through a Matlab
        //   script which generates the roll and pitch noise based on the
        //   sensor noise.  The value below is the 1-sigma value at 0
        //   degrees.  The quadratic correction below is meant to increase
        //   R as the angle increases (due to the geometry and
        //   mathematical function used to compute the angle).
        //
        //   Matlab script: R_Versus_Theta.m
        switch (sysRange) {
            case _200_DPS_RANGE:
                // -200 VRW value (average x/y/z): 7.2e-4 [(m/sec)/rt-sec]
                Rnom = (real)(1.0e-06);  // (1.0e-3 [rad])^2
                break;
            case _400_DPS_RANGE:
                // -400 VRW value (average x/y/z): 8.8e-4 [(m/sec)/rt-sec]
                Rnom = (real)(1.6e-06);  // (1.3e-3 [rad])^2
                break;
        }
    }

    // High/low-gain switching to increase the EKF gain when the system is
    //   static
    if( (gAlgorithm.state == HIGH_GAIN_AHRS) || gAlgorithm.linAccelSwitch )
    {
        // High-Gain
        gKalmanFilter.R_INS[STATE_ROLL][STATE_ROLL] = Rnom;

        //gKalmanFilter.R_INS[STATE_YAW][STATE_YAW]   = (real)1.0e-3;  // v14.6 values
        gKalmanFilter.R_INS[STATE_YAW][STATE_YAW]   = (real)2.0e-05; //(for sig = 1e-3)
    } else {
        // Low-gain -- Set to 1000x the HG value (this value determined via
        //             simulation).
        gKalmanFilter.R_INS[STATE_ROLL][STATE_ROLL] = (real)1000.0 * Rnom;

        //gKalmanFilter.R_INS[STATE_YAW][STATE_YAW]   = (real)1.0e-1; // v14.6 values
        gKalmanFilter.R_INS[STATE_YAW][STATE_YAW]   = (real)0.02; //(for sig = 1e-3)
    }

    // Set the pitch and yaw-values using the roll value (this will change, at
    //   least for the yaw-value) when the AHRS and INS are implemented.
    gKalmanFilter.R_INS[STATE_PITCH][STATE_PITCH] = gKalmanFilter.R_INS[STATE_ROLL][STATE_ROLL];

    // For 'large' roll/pitch angles, increase R-yaw to decrease the effect of
    //   update due to potential uncompensated z-axis magnetometer readings from
    //   affecting the yaw-update.
    if( ( gKalmanFilter.eulerAngles[ROLL]  > TEN_DEGREES_IN_RAD ) ||
        ( gKalmanFilter.eulerAngles[PITCH] > TEN_DEGREES_IN_RAD ) )
    {
        gKalmanFilter.R_INS[STATE_YAW][STATE_YAW]   = (real)0.2;
    }

    // Adjust the roll R-value based on the predicted pitch
    // The multiplier is due to the fact that R changes with phi and
    //   theta.  Either make it big enough to work with all angles
    //   or vary it with the angle.
    real mult = (real)1.0 + (real)0.65 * gKalmanFilter.eulerAngles[PITCH] * gKalmanFilter.eulerAngles[PITCH];
    gKalmanFilter.R_INS[STATE_ROLL][STATE_ROLL] = mult * mult * gKalmanFilter.R_INS[STATE_ROLL][STATE_ROLL];

#ifndef YAW_SWITCH_ORIG_METHOD
    // If the turn-switch is activated, increase R above the prescribed value
    if( gAlgorithm.bitStatus.swStatus.bit.turnSwitch == TRUE ) {
        gKalmanFilter.R_INS[STATE_ROLL][STATE_ROLL]   = 100.0 * gKalmanFilter.R_INS[STATE_ROLL][STATE_ROLL];
        gKalmanFilter.R_INS[STATE_PITCH][STATE_PITCH] = 100.0 * gKalmanFilter.R_INS[STATE_PITCH][STATE_PITCH];
        gKalmanFilter.R_INS[STATE_YAW][STATE_YAW]     = 100.0 * gKalmanFilter.R_INS[STATE_YAW][STATE_YAW];
    }
#endif
}


//
uint8_t rowNum, colNum, multIndex;

real S_1x1,       SInverse_1x1;
real S_2x2[2][2], SInverse_2x2[2][2];
real S_3x3[3][3], SInverse_3x3[3][3];
real S_4x4[4][4], SInverse_4x4[4][4];

real SInverse[ROWS_IN_H][ROWS_IN_H];
real PxHTranspose[ROWS_IN_P][ROWS_IN_H], HxPxHTranspose[ROWS_IN_H][ROWS_IN_H];
real PxHTranspose_Q[ROWS_IN_P][4], HxPxHTranspose_Q[4][4];

real KxH[NUMBER_OF_EKF_STATES][COLS_IN_H] = {{ 0.0 }};
real deltaP[ROWS_IN_P][COLS_IN_P];
real deltaP_tmp[ROWS_IN_P][COLS_IN_P];

real KxH_RP[NUMBER_OF_EKF_STATES][NUMBER_OF_EKF_STATES] = {{ 0.0 }};
real KxH_Y[NUMBER_OF_EKF_STATES][NUMBER_OF_EKF_STATES]  = {{ 0.0 }};

real stateUpdate[NUMBER_OF_EKF_STATES];


real PxHTranspose_RP[NUMBER_OF_EKF_STATES][2], HxPxHTranspose_RP[2][2];
real PxHTranspose_Y[NUMBER_OF_EKF_STATES][1],  HxPxHTranspose_Y[1][1];

real stateUpdate_RP[NUMBER_OF_EKF_STATES];
real stateUpdate_Y[NUMBER_OF_EKF_STATES];


real PxHTranspose_tmp[ROWS_IN_P][ROWS_IN_H], HxPxHTranspose_tmp[ROWS_IN_H][ROWS_IN_H];

#define  ROWS_IN_K  16

void Update_AHRS_EA(void)
{
    // Calculate the elements in the H and R matrices
    //                                             Matrix sizes for an Euler-angle based AHRS solution:
    _GenerateObservationJacobian_AHRS_EA();     // gKF.H: 3x16
    _GenerateObservationCovariance_AHRS_EA();   // gKF.R: 3x3

#if 1
    // This solution consists of an integrated roll/pitch/yaw solution
    // S = H*P*HTrans + R (However the matrix math can be simplified since
    //                     H is very sparse!  P is fully populated)
    // Update P from the P, H, and R matrices: P = HxPxHTranspose + R
    //   1) PxHTranspose is computed first
    for (rowNum = 0; rowNum < ROWS_IN_P; rowNum++) {
        for (colNum = 0; colNum < ROWS_IN_H; colNum++) {
            PxHTranspose[rowNum][colNum] = 0.0;
            for (multIndex = RLE_H[colNum][0]; multIndex <= RLE_H[colNum][1]; multIndex++) {
                PxHTranspose[rowNum][colNum] = PxHTranspose[rowNum][colNum] +
                    gKalmanFilter.P[rowNum][multIndex] * gKalmanFilter.H[colNum][multIndex];
            }
        }
    }

    //   2) Use gKalmanFilter.P as a temporary variable to hold HxPxHTranspose
    //      to reduce the number of "large" variables on the heap
    for (rowNum = 0; rowNum < 3; rowNum++) {
        for (colNum = 0; colNum < 3; colNum++) {
            HxPxHTranspose[rowNum][colNum] = 0.0;
            for (multIndex = RLE_H[rowNum][0]; multIndex <= RLE_H[rowNum][1]; multIndex++) {
                HxPxHTranspose[rowNum][colNum] = HxPxHTranspose[rowNum][colNum] +
                    gKalmanFilter.H[rowNum][multIndex] * PxHTranspose[multIndex][colNum];
            }
        }
    }

    // S = HxPxHTranspose + R (rows 7:10 and cols 7:10 of P PLUS diagonal of R)
    S_3x3[ROLL][ROLL]   = HxPxHTranspose[ROLL][ROLL]   + gKalmanFilter.R_INS[STATE_ROLL][STATE_ROLL];
    S_3x3[ROLL][PITCH]  = HxPxHTranspose[ROLL][PITCH];
    S_3x3[ROLL][YAW]    = HxPxHTranspose[ROLL][YAW];

    S_3x3[PITCH][ROLL]  = HxPxHTranspose[PITCH][ROLL];
    S_3x3[PITCH][PITCH] = HxPxHTranspose[PITCH][PITCH] + gKalmanFilter.R_INS[STATE_PITCH][STATE_PITCH];
    S_3x3[PITCH][YAW]   = HxPxHTranspose[PITCH][YAW];

    S_3x3[YAW][ROLL]    = HxPxHTranspose[YAW][ROLL];
    S_3x3[YAW][PITCH]   = HxPxHTranspose[YAW][PITCH];
    S_3x3[YAW][YAW]     = HxPxHTranspose[YAW][YAW]     + gKalmanFilter.R_INS[STATE_YAW][STATE_YAW];

    // Invert the S-Matrix (replace with sequential update)
    matrixInverse_3x3(&S_3x3[0][0], &SInverse_3x3[0][0]);

    // Compute the Kalman gain: K = P*HTrans*SInv
    AxB( &PxHTranspose[0][0],
         &SInverse_3x3[0][0],
         ROWS_IN_P, ROWS_IN_H, ROWS_IN_H,
         &gKalmanFilter.K[0][0] );

    // Compute attitude-quaternion updates: Dx = K*nu
    // NOTE: Can access nu in the elements that the attitude error is stored BUT the
    //       value of ROWS_IN_H must be correct or the multiplication will be wrong
    AxV( &gKalmanFilter.K[0][0],
         &gKalmanFilter.nu[STATE_ROLL],
         NUMBER_OF_EKF_STATES, ROWS_IN_H,
         &stateUpdate[0] );

    // Update states based on computed deltas
    // --- attitude quaternions (q = q + Dq) ---
    gKalmanFilter.quaternion[Q0] = gKalmanFilter.quaternion[Q0] + stateUpdate[STATE_Q0];
    gKalmanFilter.quaternion[Q1] = gKalmanFilter.quaternion[Q1] + stateUpdate[STATE_Q1];
    gKalmanFilter.quaternion[Q2] = gKalmanFilter.quaternion[Q2] + stateUpdate[STATE_Q2];
    gKalmanFilter.quaternion[Q3] = gKalmanFilter.quaternion[Q3] + stateUpdate[STATE_Q3];

    // Normalize q
    QuatNormalize(&gKalmanFilter.quaternion[0]);

    // --- Angular-rate bias (wBias = wBias = DwBias) ---
    //     If magnetometers are not used then set the rate bias to zero???
    gKalmanFilter.rateBias_B[X_AXIS] = gKalmanFilter.rateBias_B[X_AXIS] + stateUpdate[STATE_WBX];
    gKalmanFilter.rateBias_B[Y_AXIS] = gKalmanFilter.rateBias_B[Y_AXIS] + stateUpdate[STATE_WBY];
// FIXME: what if external magnetometers are used?
    if (gConfiguration.userBehavior.bit.useMags && gCalibration.productConfiguration.bit.hasMags) {
        gKalmanFilter.rateBias_B[Z_AXIS] = gKalmanFilter.rateBias_B[Z_AXIS] + stateUpdate[STATE_WBZ];
    } else {
        gKalmanFilter.rateBias_B[Z_AXIS] = 0.0;
    }

    // Update covariance: P = P + DP = P - K*H*P
    // KxH = gKF.K * gKF.H;
    //   2) Use gKalmanFilter.P as a temporary variable to hold FxPxFTranspose
    //      to reduce the number of "large" variables on the heap
    memset(KxH, 0, sizeof(KxH));
    for (rowNum = 0; rowNum < ROWS_IN_K; rowNum++) {
        for (colNum = RLE_KxH[rowNum][0]; colNum <= RLE_KxH[rowNum][1]; colNum++) {
            for (multIndex = 0; multIndex < ROWS_IN_H; multIndex++) {
                KxH[rowNum][colNum] = KxH[rowNum][colNum] +
                    gKalmanFilter.K[rowNum][multIndex] * gKalmanFilter.H[multIndex][colNum];
            }
        }
    }

// KxH is sparse too.  Only cols 6 to 9 are populated.
    // Only modify the rows and cols that correspond to q and wb (the remainder of DP will be zero)
    //   (this is to attemp to fix the slow response in roll and pitch due to 'fast' motions that
    //   Parvez brought to my attention.  If this does not fix it then maybe the values of R are too
    //   large when the system is at rest -- change with aMag???)
    // deltaP = KxH * gKF.P;
    memset(deltaP_tmp, 0, sizeof(deltaP_tmp));
    //   2) Use gKalmanFilter.P as a temporary variable to hold FxPxFTranspose
    //      to reduce the number of "large" variables on the heap
    for (rowNum = 0; rowNum < ROWS_IN_K; rowNum++) {
        for (colNum = 0; colNum < COLS_IN_P; colNum++) {
            for (multIndex = RLE_KxH[rowNum][0]; multIndex <= RLE_KxH[rowNum][1]; multIndex++) {
                deltaP_tmp[rowNum][colNum] = deltaP_tmp[rowNum][colNum] +
                    KxH[rowNum][multIndex] * gKalmanFilter.P[multIndex][colNum];
            }
        }
    }

    // Commenting out the following doesn't really save any time
//    // Zero out P values not affected by attitude information
//    memset(deltaP, 0, sizeof(deltaP));
//    for (rowNum = STATE_Q0; rowNum <= STATE_WBZ; rowNum++) {
//        for (colNum = STATE_Q0; colNum <= STATE_WBZ; colNum++) {
//            deltaP[rowNum][colNum] = deltaP_tmp[rowNum][colNum];
//        }
//    }

    // gKF.P = gKF.P - deltaP;
    AMinusB( &gKalmanFilter.P[0][0],
             &deltaP_tmp[0][0], //&deltaP[0][0],
             ROWS_IN_P, COLS_IN_P,
             &gKalmanFilter.P[0][0]);

    // Ensure P is symmetric
    ForceMatrixSymmetry( &gKalmanFilter.P[0][0],
                         ROWS_IN_P, COLS_IN_P );
#else
// place separated soln here
#endif
}


#define HG 0
#define LG 1


void _GPS_PosVel_To_NED(void)
{
    static int oneTimeOnly = TRUE;

    LLA_To_Base(&gAlgorithm.llaRad[0], &gAlgorithm.rGPS0_E[0], &gAlgorithm.rGPS_N[0], &gAlgorithm.R_NinE[0][0], &gAlgorithm.rGPS_E[0]);

    // Upon the first entry into INS, save off the base position and reset the
    //   Kalman filter variables.
    if (gAlgorithm.insFirstTime) {
        gAlgorithm.insFirstTime = FALSE;

        // Save off the base ECEF location
        gAlgorithm.rGPS0_E[X_AXIS] = gAlgorithm.rGPS_E[X_AXIS];
        gAlgorithm.rGPS0_E[Y_AXIS] = gAlgorithm.rGPS_E[Y_AXIS];
        gAlgorithm.rGPS0_E[Z_AXIS] = gAlgorithm.rGPS_E[Z_AXIS];

        // Reset the gps position (as position is relative to starting location)
        gAlgorithm.rGPS_N[X_AXIS] = 0.0;
        gAlgorithm.rGPS_N[Y_AXIS] = 0.0;
        gAlgorithm.rGPS_N[Z_AXIS] = 0.0;

        // ResetINS;  % <--need to reset H and R for each case (INS, MagOnly, ...)

        // Reset prediction values
        gKalmanFilter.Position_N[LAT_IDX] = 0.0;
        gKalmanFilter.Position_N[LON_IDX] = 0.0;
        gKalmanFilter.Position_N[ALT_IDX] = 0.0;

        gKalmanFilter.Velocity_N[X_AXIS] = (real)gGpsDataPtr->vNed[X_AXIS];
        gKalmanFilter.Velocity_N[Y_AXIS] = (real)gGpsDataPtr->vNed[Y_AXIS];
        gKalmanFilter.Velocity_N[Z_AXIS] = (real)gGpsDataPtr->vNed[Z_AXIS];

        if (oneTimeOnly) {
            oneTimeOnly = FALSE;
            gAlgorithm.applyDeclFlag = TRUE;

#ifdef DISPLAY_DIAGNOSTIC_MSG
            TimingVars_DiagnosticMsg("GPS_PosVel_To_NED: First time GPS");
#endif

            // add declination to prediction;

            // Adjust prediction when GPS goes healthy and WMM obtains the first
            //   solution.  rotate the quat by the declination at the current
            // position.
            real qDecl[4];
            qDecl[Q0] = cos((real)0.5 * gWorldMagModel.decl_rad);
            qDecl[Q1] = (real)0.0;
            qDecl[Q2] = (real)0.0;
            qDecl[Q3] = sin((real)0.5 * gWorldMagModel.decl_rad);

            real Q[4][4] = { { qDecl[Q0], -qDecl[Q1], -qDecl[Q2], -qDecl[Q3] },
                             { qDecl[Q1],  qDecl[Q0], -qDecl[Q3],  qDecl[Q2] },
                             { qDecl[Q2],  qDecl[Q3],  qDecl[Q0], -qDecl[Q1] },
                             { qDecl[Q3], -qDecl[Q2],  qDecl[Q1],  qDecl[Q0] } };

            real q[4];
            AxV(&Q[0][0], &gKalmanFilter.quaternion[0], 4, 4, &q[0]);

            //
            gKalmanFilter.quaternion[Q0] = q[Q0];
            gKalmanFilter.quaternion[Q1] = q[Q1];
            gKalmanFilter.quaternion[Q2] = q[Q2];
            gKalmanFilter.quaternion[Q3] = q[Q3];
        }
    }
}


void ComputeSystemInnovation_INS(void)
{
    // ----Compute the innovation vector, nu----
    // Position error
    gKalmanFilter.nu[STATE_RX] = gAlgorithm.rGPS_N[X_AXIS] - gKalmanFilter.Position_N[X_AXIS];
    gKalmanFilter.nu[STATE_RY] = gAlgorithm.rGPS_N[Y_AXIS] - gKalmanFilter.Position_N[Y_AXIS];
    gKalmanFilter.nu[STATE_RZ] = gAlgorithm.rGPS_N[Z_AXIS] - gKalmanFilter.Position_N[Z_AXIS];

    gKalmanFilter.nu[STATE_RX] = _LimitValue(gKalmanFilter.nu[STATE_RX], (real)270.0);
    gKalmanFilter.nu[STATE_RY] = _LimitValue(gKalmanFilter.nu[STATE_RY], (real)270.0);
    gKalmanFilter.nu[STATE_RZ] = _LimitValue(gKalmanFilter.nu[STATE_RZ], (real)270.0);

    // Velocity error
    gKalmanFilter.nu[STATE_VX] = (real)gGpsDataPtr->vNed[X_AXIS] - gKalmanFilter.Velocity_N[X_AXIS];
    gKalmanFilter.nu[STATE_VY] = (real)gGpsDataPtr->vNed[Y_AXIS] - gKalmanFilter.Velocity_N[Y_AXIS];
    gKalmanFilter.nu[STATE_VZ] = (real)gGpsDataPtr->vNed[Z_AXIS] - gKalmanFilter.Velocity_N[Z_AXIS];

    gKalmanFilter.nu[STATE_VX] = _LimitValue(gKalmanFilter.nu[STATE_VX], (real)27.0);
    gKalmanFilter.nu[STATE_VY] = _LimitValue(gKalmanFilter.nu[STATE_VY], (real)27.0);
    gKalmanFilter.nu[STATE_VZ] = _LimitValue(gKalmanFilter.nu[STATE_VZ], (real)27.0);

    // Attitude error
    gKalmanFilter.nu[STATE_Q0] = gKalmanFilter.measuredQuaternion[Q0] - gKalmanFilter.quaternion[Q0];
    gKalmanFilter.nu[STATE_Q1] = gKalmanFilter.measuredQuaternion[Q1] - gKalmanFilter.quaternion[Q1];
    gKalmanFilter.nu[STATE_Q2] = gKalmanFilter.measuredQuaternion[Q2] - gKalmanFilter.quaternion[Q2];
    gKalmanFilter.nu[STATE_Q3] = gKalmanFilter.measuredQuaternion[Q3] - gKalmanFilter.quaternion[Q3];

    // When the filtered yaw - rate is above certain thresholds then reduce the
    //   attitude - errors used to update roll and pitch.
    _TurnSwitch_Q();
}


void GenerateMeasurementJacobian_INS(void);
void GenerateMeasurementConvarianceMatrix_INS(void);

real stateUpdate_pos[NUMBER_OF_EKF_STATES], stateUpdate_vel[NUMBER_OF_EKF_STATES], stateUpdate_att[NUMBER_OF_EKF_STATES];
real K_pos[NUMBER_OF_EKF_STATES][3], K_vel[NUMBER_OF_EKF_STATES][3], K_att[NUMBER_OF_EKF_STATES][4];
real KxHxP[NUMBER_OF_EKF_STATES][NUMBER_OF_EKF_STATES];

// The position update only allows us to update the position and velocity states (along with Pr and Pv)
//   (want to verify this)
static void updateInsPos(void)
{
    real nu_pos[3];
    nu_pos[0] = gKalmanFilter.nu[STATE_RX];
    nu_pos[1] = gKalmanFilter.nu[STATE_RY];
    nu_pos[2] = gKalmanFilter.nu[STATE_RZ];

    // ++++++++++++++++++++++ POSITION ++++++++++++++++++++++
    // Step 1) Position% Step 1) Position
    // S1 = H1*gKF.P*H1' + R1;   // Top 3 rows of the first 3 cols of P + R
    // K1 = gKF.P*H1'*inv(S1);   // ( first 3 cols of P ) * S1Inverse
    // P1 = (eye(16) - K1*H1) * gKF.P;

    // S1 = H1*gKF.P*H1' + R1;
    S_3x3[0][0] = gKalmanFilter.P[STATE_RX][STATE_RX] + (real)R_VALS_GPS_POS_X;
    S_3x3[0][1] = gKalmanFilter.P[STATE_RX][STATE_RY];
    S_3x3[0][2] = gKalmanFilter.P[STATE_RX][STATE_RZ];

    S_3x3[1][0] = gKalmanFilter.P[STATE_RY][STATE_RX];
    S_3x3[1][1] = gKalmanFilter.P[STATE_RY][STATE_RY] + (real)R_VALS_GPS_POS_Y;
    S_3x3[1][2] = gKalmanFilter.P[STATE_RY][STATE_RZ];

    S_3x3[2][0] = gKalmanFilter.P[STATE_RZ][STATE_RX];
    S_3x3[2][1] = gKalmanFilter.P[STATE_RZ][STATE_RY];
    S_3x3[2][2] = gKalmanFilter.P[STATE_RZ][STATE_RZ] + (real)R_VALS_GPS_POS_Z;

    // S1_Inverse
    matrixInverse_3x3(&S_3x3[0][0], &SInverse_3x3[0][0]);

    // Compute K1 = ( gKF.P*H1' ) * S1Inverse = ( first 3 cols of P ) * S1Inverse
    for (rowNum = 0; rowNum < NUMBER_OF_EKF_STATES; rowNum++) {
        for (colNum = X_AXIS; colNum <= Z_AXIS; colNum++) {
            K_pos[rowNum][colNum] = 0.0;
            // H is sparse so only the columns of P associated with the position states are used
            //   in the calculation
            for (multIndex = STATE_RX; multIndex <= STATE_RZ; multIndex++) {
                K_pos[rowNum][colNum] = K_pos[rowNum][colNum] +
                                        gKalmanFilter.P[rowNum][multIndex] * SInverse_3x3[multIndex - STATE_RX][colNum];
            }
        }
    }

    // Compute the intermediate state update, stateUpdate_pos
    AxB(&K_pos[0][0], &nu_pos[0], 16, 3, 1, &stateUpdate_pos[0]);

    memset(KxHxP, 0, sizeof(KxHxP));
#if 1
    // Update the intermediate covariance estimate
    for (rowNum = 0; rowNum < NUMBER_OF_EKF_STATES; rowNum++) {
        for (colNum = 0; colNum < NUMBER_OF_EKF_STATES; colNum++) {
//            KxHxP[rowNum][colNum] = 0.0;
            // H is sparse so only the columns of P associated with the position states are used
            //   in the calculation
            for (multIndex = STATE_RX; multIndex <= STATE_RZ; multIndex++) {
                KxHxP[rowNum][colNum] = KxHxP[rowNum][colNum] +
                                        K_pos[rowNum][multIndex] * gKalmanFilter.P[multIndex][colNum];
            }
        }
    }
#else
    // Only modify the rows and cols that correspond to q and wb (the remainder of DP will be zero)
    //   (this is to attemp to fix the slow response in roll and pitch due to 'fast' motions that
    //   Parvez brought to my attention.  If this does not fix it then maybe the values of R are too
    //   large when the system is at rest -- change with aMag???)
    // deltaP = KxH * gKF.P;
    for (rowNum = STATE_RX; rowNum <= STATE_VZ; rowNum++) {
        for (colNum = STATE_RX; colNum <= STATE_VZ; colNum++) {
//            KxHxP[rowNum][colNum] = 0.0;
            for (multIndex = STATE_RX; multIndex <= STATE_RZ; multIndex++) {
                KxHxP[rowNum][colNum] = KxHxP[rowNum][colNum] +
                                        K_pos[rowNum][multIndex] * gKalmanFilter.P[multIndex][colNum];
            }
        }
    }
#endif

//deltaP[ROWS_IN_P][COLS_IN_P];
//deltaP_tmp

//    memset(deltaP_tmp, 0, sizeof(deltaP_tmp));   // <-- Probably don't need to set to zero
//    memset(deltaP, 0, sizeof(deltaP));

    AMinusB(&gKalmanFilter.P[0][0], &KxHxP[0][0], 16, 16, &gKalmanFilter.P[0][0]);
    // ++++++++++++++++++++++ END OF POSITION ++++++++++++++++++++++

    // Update states
    gKalmanFilter.Position_N[X_AXIS] = gKalmanFilter.Position_N[X_AXIS] + stateUpdate_pos[STATE_RX];
    gKalmanFilter.Position_N[Y_AXIS] = gKalmanFilter.Position_N[Y_AXIS] + stateUpdate_pos[STATE_RY];
    gKalmanFilter.Position_N[Z_AXIS] = gKalmanFilter.Position_N[Z_AXIS] + stateUpdate_pos[STATE_RZ];

    gKalmanFilter.Velocity_N[X_AXIS] = gKalmanFilter.Velocity_N[X_AXIS] + stateUpdate_pos[STATE_VX];
    gKalmanFilter.Velocity_N[Y_AXIS] = gKalmanFilter.Velocity_N[Y_AXIS] + stateUpdate_pos[STATE_VY];
    gKalmanFilter.Velocity_N[Z_AXIS] = gKalmanFilter.Velocity_N[Z_AXIS] + stateUpdate_pos[STATE_VZ];

//    gKalmanFilter.quaternion[Q0] = gKalmanFilter.quaternion[Q0] + stateUpdate_pos[STATE_Q0];
//    gKalmanFilter.quaternion[Q1] = gKalmanFilter.quaternion[Q1] + stateUpdate_pos[STATE_Q1];
//    gKalmanFilter.quaternion[Q2] = gKalmanFilter.quaternion[Q2] + stateUpdate_pos[STATE_Q2];
//    gKalmanFilter.quaternion[Q3] = gKalmanFilter.quaternion[Q3] + stateUpdate_pos[STATE_Q3];

//    gKalmanFilter.rateBias_B[X_AXIS] = gKalmanFilter.rateBias_B[X_AXIS] + stateUpdate_pos[STATE_WBX];
//    gKalmanFilter.rateBias_B[Y_AXIS] = gKalmanFilter.rateBias_B[Y_AXIS] + stateUpdate_pos[STATE_WBY];
//    gKalmanFilter.rateBias_B[Z_AXIS] = gKalmanFilter.rateBias_B[Z_AXIS] + stateUpdate_pos[STATE_WBZ];

//    gKalmanFilter.accelBias_B[X_AXIS] = gKalmanFilter.accelBias_B[X_AXIS] + stateUpdate_pos[STATE_ABX];
//    gKalmanFilter.accelBias_B[Y_AXIS] = gKalmanFilter.accelBias_B[Y_AXIS] + stateUpdate_pos[STATE_ABY];
//    gKalmanFilter.accelBias_B[Z_AXIS] = gKalmanFilter.accelBias_B[Z_AXIS] + stateUpdate_pos[STATE_ABZ];
}


// The velocity update only allows us to update the velocity, attitude, and acceleration-bias states (along with Pv, Pq, and Pab)
//   (want to verify this)
static void updateInsVel(void)
{
    real nu_vel[3];
    nu_vel[0] = gKalmanFilter.nu[STATE_VX];
    nu_vel[1] = gKalmanFilter.nu[STATE_VY];
    nu_vel[2] = gKalmanFilter.nu[STATE_VZ];

    // ++++++++++++++++++++++ VELOCITY ++++++++++++++++++++++
    // Step 2) Velocity
    //S2 = H2*P1*H2' + R2; (4th, 5th, and 6th rows of the 4th, 5th, and 6th cols of P1)
    //K2 = P1*H2'*inv(S2);
    //P2 = (eye(16) - K2*H2) * P1;

    // S2 = H2*P1*H2' + R2;
    S_3x3[0][0] = gKalmanFilter.P[STATE_VX][STATE_VX] + (real)R_VALS_GPS_VEL_X;
    S_3x3[0][1] = gKalmanFilter.P[STATE_VX][STATE_VY];
    S_3x3[0][2] = gKalmanFilter.P[STATE_VX][STATE_VZ];

    S_3x3[1][0] = gKalmanFilter.P[STATE_VY][STATE_VX];
    S_3x3[1][1] = gKalmanFilter.P[STATE_VY][STATE_VY] + (real)R_VALS_GPS_VEL_Y;
    S_3x3[1][2] = gKalmanFilter.P[STATE_VY][STATE_VZ];

    S_3x3[2][0] = gKalmanFilter.P[STATE_VZ][STATE_VX];
    S_3x3[2][1] = gKalmanFilter.P[STATE_VZ][STATE_VY];
    S_3x3[2][2] = gKalmanFilter.P[STATE_VZ][STATE_VZ] + (real)R_VALS_GPS_VEL_Z;

    // S2_Inverse
    matrixInverse_3x3(&S_3x3[0][0], &SInverse_3x3[0][0]);

    // Compute K2 = ( P1*H2' ) * S2Inverse = ( 4th, 5th, and 6th cols of P1 ) * S2Inverse
    for (rowNum = 0; rowNum < NUMBER_OF_EKF_STATES; rowNum++) {
        for (colNum = X_AXIS; colNum <= Z_AXIS; colNum++) {
            K_vel[rowNum][colNum] = 0.0;
            // H is sparse so only the columns of P associated with the velocity states are used
            //   in the calculation
            for (multIndex = STATE_VX; multIndex <= STATE_VZ; multIndex++) {
                K_vel[rowNum][colNum] = K_vel[rowNum][colNum] +
                                        gKalmanFilter.P[rowNum][multIndex] * SInverse_3x3[multIndex - STATE_VX][colNum];
            }
        }
    }

    // Compute the intermediate state update
    AxB(&K_vel[0][0], &nu_vel[0], 16, 3, 1, &stateUpdate_vel[0]);

    memset(KxHxP, 0, sizeof(KxHxP));
#if 1
    // Update the intermediate covariance estimate
    for (rowNum = 0; rowNum < NUMBER_OF_EKF_STATES; rowNum++) {
        for (colNum = 0; colNum < NUMBER_OF_EKF_STATES; colNum++) {
//            KxHxP[rowNum][colNum] = 0.0;
            // H is sparse so only the columns of P associated with the velocity states are used
            //   in the calculation
            for (multIndex = STATE_VX; multIndex <= STATE_VZ; multIndex++) {
                KxHxP[rowNum][colNum] = KxHxP[rowNum][colNum] +
                                        K_vel[rowNum][multIndex - STATE_VX] * gKalmanFilter.P[multIndex][colNum];
            }
        }
    }
#else
    // Only modify the rows and cols that correspond to q and wb (the remainder of DP will be zero)
    //   (this is to attemp to fix the slow response in roll and pitch due to 'fast' motions that
    //   Parvez brought to my attention.  If this does not fix it then maybe the values of R are too
    //   large when the system is at rest -- change with aMag???)
    // deltaP = KxH * gKF.P;
    memset(KxHxP, 0, sizeof(KxHxP));   // <-- Probably don't need to set to zero
    for (rowNum = STATE_VX; rowNum <= STATE_ABZ; rowNum++) {
        for (colNum = STATE_VX; colNum <= STATE_ABZ; colNum++) {
            //            KxHxP[rowNum][colNum] = 0.0;
            for (multIndex = STATE_VX; multIndex <= STATE_VZ; multIndex++) {
                KxHxP[rowNum][colNum] = KxHxP[rowNum][colNum] +
                                        K_vel[rowNum][multIndex - STATE_VX] * gKalmanFilter.P[multIndex][colNum];
            }
        }
    }
#endif

    // P2 = P2 - KxHxP2
    AMinusB(&gKalmanFilter.P[0][0], &KxHxP[0][0], 16, 16, &gKalmanFilter.P[0][0]);
    // ++++++++++++++++++++++ END OF VELOCITY ++++++++++++++++++++++

    // Update states
//    gKalmanFilter.Position_N[X_AXIS] = gKalmanFilter.Position_N[X_AXIS] + stateUpdate_vel[STATE_RX];
//    gKalmanFilter.Position_N[Y_AXIS] = gKalmanFilter.Position_N[Y_AXIS] + stateUpdate_vel[STATE_RY];
//    gKalmanFilter.Position_N[Z_AXIS] = gKalmanFilter.Position_N[Z_AXIS] + stateUpdate_vel[STATE_RZ];

    gKalmanFilter.Velocity_N[X_AXIS] = gKalmanFilter.Velocity_N[X_AXIS] + stateUpdate_vel[STATE_VX];
    gKalmanFilter.Velocity_N[Y_AXIS] = gKalmanFilter.Velocity_N[Y_AXIS] + stateUpdate_vel[STATE_VY];
    gKalmanFilter.Velocity_N[Z_AXIS] = gKalmanFilter.Velocity_N[Z_AXIS] + stateUpdate_vel[STATE_VZ];

    gKalmanFilter.quaternion[Q0] = gKalmanFilter.quaternion[Q0] + stateUpdate_vel[STATE_Q0];
    gKalmanFilter.quaternion[Q1] = gKalmanFilter.quaternion[Q1] + stateUpdate_vel[STATE_Q1];
    gKalmanFilter.quaternion[Q2] = gKalmanFilter.quaternion[Q2] + stateUpdate_vel[STATE_Q2];
    gKalmanFilter.quaternion[Q3] = gKalmanFilter.quaternion[Q3] + stateUpdate_vel[STATE_Q3];

//    gKalmanFilter.rateBias_B[X_AXIS] = gKalmanFilter.rateBias_B[X_AXIS] + stateUpdate_vel[STATE_WBX];
//    gKalmanFilter.rateBias_B[Y_AXIS] = gKalmanFilter.rateBias_B[Y_AXIS] + stateUpdate_vel[STATE_WBY];
//    gKalmanFilter.rateBias_B[Z_AXIS] = gKalmanFilter.rateBias_B[Z_AXIS] + stateUpdate_vel[STATE_WBZ];

    gKalmanFilter.accelBias_B[X_AXIS] = gKalmanFilter.accelBias_B[X_AXIS] + stateUpdate_vel[STATE_ABX];
    gKalmanFilter.accelBias_B[Y_AXIS] = gKalmanFilter.accelBias_B[Y_AXIS] + stateUpdate_vel[STATE_ABY];
    gKalmanFilter.accelBias_B[Z_AXIS] = gKalmanFilter.accelBias_B[Z_AXIS] + stateUpdate_vel[STATE_ABZ];

}


// The attitude update only allows us to update the attitude and rate-bias states (along with Pq and Pwb)
//   (want to verify this)
static void updateInsAtt(void)
{
    real nu_att[4];
    nu_att[0] = gKalmanFilter.nu[STATE_Q0];
    nu_att[1] = gKalmanFilter.nu[STATE_Q1];
    nu_att[2] = gKalmanFilter.nu[STATE_Q2];
    nu_att[3] = gKalmanFilter.nu[STATE_Q3];

    // ++++++++++++++++++++++ QUATERNION ++++++++++++++++++++++
    // Step 3) Quaternion
    //S3 = H3*P2*H3' + R3;   // (7th through 10th rows of the 7th through 10th cols of P)
    //K3 = P2*H3'*inv(S3);
    //gKF.P = (eye(16) - K3*H3) * P2;

    if (gAlgorithm.state == INS_SOLUTION) {
        if ((gAlgorithm.state == HIGH_GAIN_AHRS) ||
            (gAlgorithm.linAccelSwitch))
        {
// When the linear acceleration switch has been activated, and the system is operating in GPS-Aided mode, the
//   results look good when R-values of [ 0.0000002, 0.01*0.00005, 0.01*0.00005, 0.05*0.00050 ] are used alongside
//   values of 0.09 for the AHRS update.  When used in both the solution is not as good.  Reasons?  Possibly due
//   to how the states are updated in the aided updates.
#if 0
            gKalmanFilter.R_INS[STATE_Q0][STATE_Q0] = (real)0.003; //(real)0.0005;//9;//0.01;//0.003;
            gKalmanFilter.R_INS[STATE_Q1][STATE_Q1] = (real)0.003; //(real)0.0005;//9;//0.01;//0.0050;
            gKalmanFilter.R_INS[STATE_Q2][STATE_Q2] = (real)0.003; ///(real)0.0005;//9;//0.01;//0.0050;
            gKalmanFilter.R_INS[STATE_Q3][STATE_Q3] = (real)0.003; //(real)0.0005;//9;//0.01;//0.0100;
#else
#if 1
            gKalmanFilter.R_INS[STATE_Q0][STATE_Q0] = (real)0.0000002; //(real)0.0005;//9;//0.01;//0.003;
            gKalmanFilter.R_INS[STATE_Q1][STATE_Q1] = (real)(0.01*0.00005); //(real)0.0005;//9;//0.01;//0.0050;
            gKalmanFilter.R_INS[STATE_Q2][STATE_Q2] = (real)(0.01*0.00005); ///(real)0.0005;//9;//0.01;//0.0050;
            gKalmanFilter.R_INS[STATE_Q3][STATE_Q3] = (real)(0.05*0.00050); //(real)0.0005;//9;//0.01;//0.0100;
#else
            gKalmanFilter.R_INS[STATE_Q0][STATE_Q0] = (real)0.0000002; //(real)0.0005;//9;//0.01;//0.003;
            gKalmanFilter.R_INS[STATE_Q1][STATE_Q1] = (real)0.00005; //(real)0.0005;//9;//0.01;//0.0050;
            gKalmanFilter.R_INS[STATE_Q2][STATE_Q2] = (real)0.00005; ///(real)0.0005;//9;//0.01;//0.0050;
            gKalmanFilter.R_INS[STATE_Q3][STATE_Q3] = (real)0.0005; //(real)0.0005;//9;//0.01;//0.0100;
#endif
#endif
        } else {
            gKalmanFilter.R_INS[STATE_Q0][STATE_Q0] = (real)0.09;//0.003;
            gKalmanFilter.R_INS[STATE_Q1][STATE_Q1] = (real)0.09;//0.0050;
            gKalmanFilter.R_INS[STATE_Q2][STATE_Q2] = (real)0.09;//0.0050;
            gKalmanFilter.R_INS[STATE_Q3][STATE_Q3] = (real)0.09;//0.0100;
        }
    }

    // previously all the R values below were set to 0.09 (for the nov 18 testing)
    // S3 = H3*P2*H3' + R3;
    S_4x4[Q0][Q0] = gKalmanFilter.P[STATE_Q0][STATE_Q0] + gKalmanFilter.R_INS[STATE_Q0][STATE_Q0]; // + (real)R_VALS_QUAT;
    S_4x4[Q0][Q1] = gKalmanFilter.P[STATE_Q0][STATE_Q1];
    S_4x4[Q0][Q2] = gKalmanFilter.P[STATE_Q0][STATE_Q2];
    S_4x4[Q0][Q3] = gKalmanFilter.P[STATE_Q0][STATE_Q3];

    S_4x4[Q1][Q0] = gKalmanFilter.P[STATE_Q1][STATE_Q0];
    S_4x4[Q1][Q1] = gKalmanFilter.P[STATE_Q1][STATE_Q1] + gKalmanFilter.R_INS[STATE_Q1][STATE_Q1]; // + (real)R_VALS_QUAT;
    S_4x4[Q1][Q2] = gKalmanFilter.P[STATE_Q1][STATE_Q2];
    S_4x4[Q1][Q3] = gKalmanFilter.P[STATE_Q1][STATE_Q3];

    S_4x4[Q2][Q0] = gKalmanFilter.P[STATE_Q2][STATE_Q0];
    S_4x4[Q2][Q1] = gKalmanFilter.P[STATE_Q2][STATE_Q1];
    S_4x4[Q2][Q2] = gKalmanFilter.P[STATE_Q2][STATE_Q2] + gKalmanFilter.R_INS[STATE_Q2][STATE_Q2]; // + (real)R_VALS_QUAT;
    S_4x4[Q2][Q3] = gKalmanFilter.P[STATE_Q2][STATE_Q3];

    S_4x4[Q3][Q0] = gKalmanFilter.P[STATE_Q3][STATE_Q0];
    S_4x4[Q3][Q1] = gKalmanFilter.P[STATE_Q3][STATE_Q1];
    S_4x4[Q3][Q2] = gKalmanFilter.P[STATE_Q3][STATE_Q2];
    S_4x4[Q3][Q3] = gKalmanFilter.P[STATE_Q3][STATE_Q3] + gKalmanFilter.R_INS[STATE_Q3][STATE_Q3]; // + (real)R_VALS_QUAT;

    // S3_Inverse
    MatrixInverse_4x4(&S_4x4[0][0], &SInverse_4x4[0][0]);

    // Compute K3 = ( P2*H3' ) * S3Inverse = ( 7th through 10th cols of P2 ) * S3Inverse
    for (rowNum = 0; rowNum < NUMBER_OF_EKF_STATES; rowNum++) {
        for (colNum = Q0; colNum <= Q3; colNum++) {
            K_att[rowNum][colNum] = 0.0;
            // H is sparse so only the columns of P associated with the attitude states are used
            //   in the calculation
            for (multIndex = STATE_Q0; multIndex <= STATE_Q3; multIndex++) {
                K_att[rowNum][colNum] = K_att[rowNum][colNum] +
                    gKalmanFilter.P[rowNum][multIndex] * SInverse_4x4[multIndex - STATE_Q0][colNum];
            }
        }
    }

    // Compute the intermediate state update
    AxB(&K_att[0][0], &nu_att[0], 16, 4, 1, &stateUpdate_att[0]);

    memset(KxHxP, 0, sizeof(KxHxP));
#if 1
    // Update the covariance estimate
    for (rowNum = 0; rowNum < NUMBER_OF_EKF_STATES; rowNum++) {
        for (colNum = 0; colNum < NUMBER_OF_EKF_STATES; colNum++) {
//            KxHxP[rowNum][colNum] = 0.0;
            // H is sparse so only the columns of P associated with the attitude states are used
            //   in the calculation
            for (multIndex = STATE_Q0; multIndex <= STATE_Q3; multIndex++) {
                KxHxP[rowNum][colNum] = KxHxP[rowNum][colNum] +
                                        K_att[rowNum][multIndex - STATE_Q0] * gKalmanFilter.P[multIndex][colNum];
            }
        }
    }
#else
    // Only modify the rows and cols that correspond to q and wb (the remainder of DP will be zero)
    //   (this is to attemp to fix the slow response in roll and pitch due to 'fast' motions that
    //   Parvez brought to my attention.  If this does not fix it then maybe the values of R are too
    //   large when the system is at rest -- change with aMag???)
    // deltaP = KxH * gKF.P;
    memset(KxHxP, 0, sizeof(KxHxP));   // <-- Probably don't need to set to zero
    for (rowNum = STATE_Q0; rowNum <= STATE_WBZ; rowNum++) {
        for (colNum = STATE_Q0; colNum <= STATE_WBZ; colNum++) {
            //            KxHxP[rowNum][colNum] = 0.0;
            for (multIndex = STATE_Q0; multIndex <= STATE_Q3; multIndex++) {
                KxHxP[rowNum][colNum] = KxHxP[rowNum][colNum] +
                                        K_att[rowNum][multIndex - STATE_Q0] * gKalmanFilter.P[multIndex][colNum];
            }
        }
    }
#endif

    // P = P2 - KxHxP2 (Ensure P is symmetric and limit the values in the matrix)
    AMinusB(&gKalmanFilter.P[0][0], &KxHxP[0][0], 16, 16, &gKalmanFilter.P[0][0]);
    ForceMatrixSymmetry(&gKalmanFilter.P[0][0], ROWS_IN_P, COLS_IN_P);
    // ++++++++++++++++++++++ END OF QUATERNION ++++++++++++++++++++++

    // Update states
//    gKalmanFilter.Position_N[X_AXIS] = gKalmanFilter.Position_N[X_AXIS] + stateUpdate_att[STATE_RX];
//    gKalmanFilter.Position_N[Y_AXIS] = gKalmanFilter.Position_N[Y_AXIS] + stateUpdate_att[STATE_RY];
//    gKalmanFilter.Position_N[Z_AXIS] = gKalmanFilter.Position_N[Z_AXIS] + stateUpdate_att[STATE_RZ];

//    gKalmanFilter.Velocity_N[X_AXIS] = gKalmanFilter.Velocity_N[X_AXIS] + stateUpdate_att[STATE_VX];
//    gKalmanFilter.Velocity_N[Y_AXIS] = gKalmanFilter.Velocity_N[Y_AXIS] + stateUpdate_att[STATE_VY];
//    gKalmanFilter.Velocity_N[Z_AXIS] = gKalmanFilter.Velocity_N[Z_AXIS] + stateUpdate_att[STATE_VZ];

    gKalmanFilter.quaternion[Q0] = gKalmanFilter.quaternion[Q0] + stateUpdate_att[STATE_Q0];
    gKalmanFilter.quaternion[Q1] = gKalmanFilter.quaternion[Q1] + stateUpdate_att[STATE_Q1];
    gKalmanFilter.quaternion[Q2] = gKalmanFilter.quaternion[Q2] + stateUpdate_att[STATE_Q2];
    gKalmanFilter.quaternion[Q3] = gKalmanFilter.quaternion[Q3] + stateUpdate_att[STATE_Q3];

    // Normalize quaternion and force q0 to be positive
    QuatNormalize(gKalmanFilter.quaternion);

    gKalmanFilter.rateBias_B[X_AXIS] = gKalmanFilter.rateBias_B[X_AXIS] + stateUpdate_att[STATE_WBX];
    gKalmanFilter.rateBias_B[Y_AXIS] = gKalmanFilter.rateBias_B[Y_AXIS] + stateUpdate_att[STATE_WBY];
    gKalmanFilter.rateBias_B[Z_AXIS] = gKalmanFilter.rateBias_B[Z_AXIS] + stateUpdate_att[STATE_WBZ];

//    gKalmanFilter.accelBias_B[X_AXIS] = gKalmanFilter.accelBias_B[X_AXIS] + stateUpdate_att[STATE_ABX];
//    gKalmanFilter.accelBias_B[Y_AXIS] = gKalmanFilter.accelBias_B[Y_AXIS] + stateUpdate_att[STATE_ABY];
//    gKalmanFilter.accelBias_B[Z_AXIS] = gKalmanFilter.accelBias_B[Z_AXIS] + stateUpdate_att[STATE_ABZ];
}


void GenerateMeasurementJacobian_INS(void)
{
    // H = dh/dx: Only need to compute H once since quaternions are used to specify
    //   the attitude.
    static int oneTime = TRUE;

    if (oneTime) {
        oneTime = FALSE;

        // THE FOLLOWING GO INTO THE INIT CODE AS THEY DON'T CHANGE WITH TIME (BUT
        memset(gKalmanFilter.H_INS, 0, sizeof(gKalmanFilter.H_INS));

        //
        gKalmanFilter.H_INS[STATE_RX][STATE_RX] = (real)1.0;
        gKalmanFilter.H_INS[STATE_RY][STATE_RY] = (real)1.0;
        gKalmanFilter.H_INS[STATE_RZ][STATE_RZ] = (real)1.0;

        gKalmanFilter.H_INS[STATE_VX][STATE_VX] = (real)1.0;
        gKalmanFilter.H_INS[STATE_VY][STATE_VY] = (real)1.0;
        gKalmanFilter.H_INS[STATE_VZ][STATE_VZ] = (real)1.0;

        // THE ONLY ELEMENTS OF THE JACOBIAN THAT CHANGES OVER TIME ARE THE ONES BELOW
        gKalmanFilter.H_INS[STATE_Q0][STATE_Q0] = (real)1.0;
        gKalmanFilter.H_INS[STATE_Q1][STATE_Q1] = (real)1.0;
        gKalmanFilter.H_INS[STATE_Q2][STATE_Q2] = (real)1.0;
        gKalmanFilter.H_INS[STATE_Q3][STATE_Q3] = (real)1.0;
    }
}


// GenerateMeasurementConvarianceMatrix_INS.m
void GenerateMeasurementConvarianceMatrix_INS(void)
{
    // H = dh/dx: Only need to compute H once since quaternions are used to specify
    //   the attitude.
    static int oneTime = TRUE;

    if (oneTime) {
        oneTime = FALSE;

        // THE FOLLOWING GO INTO THE INIT CODE AS THEY DON'T CHANGE WITH TIME (BUT
        memset(gKalmanFilter.R_INS, 0, sizeof(gKalmanFilter.R_INS));

        // Populate matrix
        gKalmanFilter.R_INS[STATE_RX][STATE_RX] = (real)R_VALS_GPS_POS_X;
        gKalmanFilter.R_INS[STATE_RY][STATE_RY] = (real)R_VALS_GPS_POS_Y;
        gKalmanFilter.R_INS[STATE_RZ][STATE_RZ] = (real)R_VALS_GPS_POS_Z;

        gKalmanFilter.R_INS[STATE_VX][STATE_VX] = (real)R_VALS_GPS_VEL_X;
        gKalmanFilter.R_INS[STATE_VY][STATE_VY] = (real)R_VALS_GPS_VEL_Y;
        gKalmanFilter.R_INS[STATE_VZ][STATE_VZ] = (real)R_VALS_GPS_VEL_Z;

        gKalmanFilter.R_INS[STATE_Q0][STATE_Q0] = (real)0.09; // R_VALS_QUAT;
        gKalmanFilter.R_INS[STATE_Q1][STATE_Q1] = (real)0.09; // R_VALS_QUAT;
        gKalmanFilter.R_INS[STATE_Q2][STATE_Q2] = (real)0.09; // R_VALS_QUAT;
        gKalmanFilter.R_INS[STATE_Q3][STATE_Q3] = (real)0.09; // R_VALS_QUAT;
    }
}


//% Conversion from turn-rate threshold (loaded into gConfiguration) to
//%   rad/sec
//% thresh_rad = ( 10.0 * pi/180 );   % 0.1745
//% thresh_counts = floor( thresh_rad * ( 2^16 / (2*pi) ) );   % 1820
//% thresh_rad = thresh_counts * ( 2*pi / 2^16 )   % 1820 * (2*pi) / 2^16 = 0.1745

real TILT_YAW_SWITCH_GAIN = (real)0.05;

// TurnSwitch.m
static void _TurnSwitch_EA(void)
{
    static real minSwitch = 0.0, maxSwitch = 0.0;
    static real turnSwitchThresholdPast = 0.0;
    static real linInterpSF;

    real absYawRate;
    real turnSwitchScaleFactor;

    // gKF.filteredYawRate (calculated in the prediction stage)
    absYawRate = fabs(gAlgorithm.filteredYawRate);

    // In case the user changes the TST during operation
    if (gConfiguration.turnSwitchThreshold != turnSwitchThresholdPast) {
        // Example conversion: ( 1820*12868 / 2^27 ) * ( 180/pi )
        minSwitch = gConfiguration.turnSwitchThreshold * (real)(TWO_PI / TWO_POW_16);   // angle in radians
        maxSwitch = (real)2.0 * minSwitch;   // angle in radians

        turnSwitchThresholdPast = gConfiguration.turnSwitchThreshold;

        linInterpSF = ((real)1.0 - TILT_YAW_SWITCH_GAIN) / (maxSwitch - minSwitch);
    }

    // Linear interpolation if the yawRate is above the specified threshold
    if ((gAlgorithm.state > HIGH_GAIN_AHRS) && (absYawRate > minSwitch))
    {
        gAlgorithm.bitStatus.swStatus.bit.turnSwitch = TRUE;
        //        std::cout << "TurnSwitch (INS): Activated\n";

        // When the rate is below the maximum rate defined by
        //   turnSwitchThreshold, then generate a scale-factor that is between
        //   ( 1.0 - G ) and 0.0 (based on absYawRate).  If it is above
        //   'maxSwitch' then the SF is zero.
        if (absYawRate < maxSwitch) {
            turnSwitchScaleFactor = linInterpSF * (maxSwitch - absYawRate);
        } else {
            // yaw-rate is above maxSwitch ==> no gain
            turnSwitchScaleFactor = 0.0;
        }

        // Specify the multiplier so it is between G an 1.0
        real turnSwitchMultiplier = TILT_YAW_SWITCH_GAIN + turnSwitchScaleFactor;

        //
#ifdef YAW_SWITCH_ORIG_METHOD
        gKalmanFilter.nu[STATE_ROLL]  = turnSwitchMultiplier * gKalmanFilter.nu[STATE_ROLL];
        gKalmanFilter.nu[STATE_PITCH] = turnSwitchMultiplier * gKalmanFilter.nu[STATE_PITCH];
#endif
    } else {
        gAlgorithm.bitStatus.swStatus.bit.turnSwitch = FALSE;
    }
}


static real _UnwrapAttitudeError(real attitudeError)
{
    while (fabs(attitudeError) > PI)
    {
        if (attitudeError > PI) {
            attitudeError = attitudeError - (real)TWO_PI;
        } else if (attitudeError < -PI) {
            attitudeError = attitudeError + (real)TWO_PI;
        }
    }

    return attitudeError;
}


static real _LimitValue(real value, real limit)
{
    if (value > limit) {
        return limit;
    } else if (value < -limit) {
        return -limit;
    }

    return value;
}



//
//real SInverse[ROWS_IN_H][ROWS_IN_H];
//real PxHTranspose[ROWS_IN_P][ROWS_IN_H], HxPxHTranspose[ROWS_IN_H][ROWS_IN_H];

////    % Update_VG.m
//void Update_VG(void)
//{
//    //// DEBUG: Probably not needed
//    //memset(gKalmanFilter.stateUpdate, 0, sizeof(gKalmanFilter.stateUpdate));
//
//    ////                                      Matrix sizes for AHRS solution:
//    GenerateObservationJacobian_VG();     // gKF.H [ 3 x N ] or [ 4 x N ]
//    GenerateObservationCovariance_VG();   // gKF.R [ 3 x 3 ] or [ 4 x 4 ]
//
//
//% TurnSwitch;
//
//%                                         Matrix sizes for VG solution:
//GenerateObservationJacobian_VG;           % gKF.H [ 2 x N ]
//GenerateObservationCovarianceMatrix_VG;   % gKF.R [ 2 x 2 ]
//
//% S = H*P*HTrans + R
//S = gKF.H * gKF.P * transpose( gKF.H ) + gKF.R;
//SInverse = 1 \ S;
//% SInverse = pinv(S);
//% SInverse = inv_3x3(S);
//
//% K = P*HTrans*SInv
//gKF.K = gKF.P * transpose( gKF.H ) * SInverse;
//
//% Dx = K*nu
//% Compute attitude-quaternion updates
//stateUpdate(STATE_Q0) = gKF.K(STATE_Q0,ROLL)  * gKF.attitudeError(ROLL) + ...
//                            gKF.K(STATE_Q0,PITCH) * gKF.attitudeError(PITCH);
//stateUpdate(STATE_Q1) = gKF.K(STATE_Q1,ROLL)  * gKF.attitudeError(ROLL) + ...
//                            gKF.K(STATE_Q1,PITCH) * gKF.attitudeError(PITCH);
//stateUpdate(STATE_Q2) = gKF.K(STATE_Q2,ROLL)  * gKF.attitudeError(ROLL) + ...
//                            gKF.K(STATE_Q2,PITCH) * gKF.attitudeError(PITCH);
//stateUpdate(STATE_Q3) = gKF.K(STATE_Q3,ROLL)  * gKF.attitudeError(ROLL) + ...
//                            gKF.K(STATE_Q3,PITCH) * gKF.attitudeError(PITCH);
//
//% Compute angular-rate bias updates
//stateUpdate(STATE_WBX) = gKF.K(STATE_WBX,ROLL)  * gKF.attitudeError(ROLL) + ...
//                             gKF.K(STATE_WBX,PITCH) * gKF.attitudeError(PITCH);
//stateUpdate(STATE_WBY) = gKF.K(STATE_WBY,ROLL)  * gKF.attitudeError(ROLL) + ...
//                             gKF.K(STATE_WBY,PITCH) * gKF.attitudeError(PITCH);
//
//% Update states based on computed deltas
//% --- attitude quaternions (q = q + Dq) ---
//gKF.quaternion(Q0) = gKF.quaternion(Q0) + stateUpdate(STATE_Q0);
//gKF.quaternion(Q1) = gKF.quaternion(Q1) + stateUpdate(STATE_Q1);
//gKF.quaternion(Q2) = gKF.quaternion(Q2) + stateUpdate(STATE_Q2);
//gKF.quaternion(Q3) = gKF.quaternion(Q3) + stateUpdate(STATE_Q3);
//
//% Normalize q and force q0 > 0
//gKF.quaternion = QuatNormalize( gKF.quaternion );
//if( gKF.quaternion(Q0) < 0 ),
//    disp( sprintf( 'Update_VG: flip sign on q (k = %d)', k ) );
//    gKF.quaternion(Q0) = -gKF.quaternion(Q0);
//    gKF.quaternion(Q1) = -gKF.quaternion(Q1);
//    gKF.quaternion(Q2) = -gKF.quaternion(Q2);
//    gKF.quaternion(Q3) = -gKF.quaternion(Q3);
//}
//}
//
//% --- Angular-rate bias (wBias = wBias = DwBias) ---
//gKF.rateBias_B(X_AXIS) = gKF.rateBias_B(X_AXIS) + stateUpdate(STATE_WBX);
//gKF.rateBias_B(Y_AXIS) = gKF.rateBias_B(Y_AXIS) + stateUpdate(STATE_WBY);
//
//% Update covariance: P = P + DP = P - K*H*P
//KxH = gKF.K * gKF.H;
//deltaP = -( KxH * gKF.P );
//gKF.P = gKF.P + deltaP;
//
//% Ensure P is symmetric and limit the values in the matrix
//gKF.P = 0.5 * ( gKF.P + transpose( gKF.P ) );
//
//ind = find( gKF.P > LIMIT.P );
//if( ~isempty(ind) ),
//    gKF.P(ind) = LIMIT.P;
//end
//ind = find( gKF.P < -LIMIT.P );
//if( ~isempty(ind) ),
//    gKF.P(ind) = -LIMIT.P;
//end
//
//}




//// GenerateObservationJacobian_VG.m
//void GenerateObservationJacobian_VG(void)
//{
//    // Only need to compute H once when quaternions are used to specify the
//    //   attitude.
//    static int oneTime = TRUE;
//
//    if (oneTime) {
//        oneTime = FALSE;
//
//        // Clear the values in H(in AHRS mode, there are 3 rows in the Jacobian)
//        memset(gKalmanFilter.H, 0, sizeof(gKalmanFilter.H));
//
//        // h = q ==> H = I
//        gKalmanFilter.H[Q0][STATE_Q0] = 1.0;
//        gKalmanFilter.H[Q1][STATE_Q1] = 1.0;
//        gKalmanFilter.H[Q2][STATE_Q2] = 1.0;
//        gKalmanFilter.H[Q3][STATE_Q3] = 1.0;
//    }
//}


//// GenerateObservationCovariance_VG.m
//void GenerateObservationCovariance_VG(void)
//{
//    // Only need to compute R once
//    static int oneTime = TRUE;
//
//    if (oneTime) {
//        oneTime = FALSE;
//
//        // Clear the values in R (in AHRS mode, there are 3 rows in the Jacobian)
//        // Initialize the Process Covariance (Q) matrix
//        memset(gKalmanFilter.R, 0, sizeof(gKalmanFilter.R));
//
//        // h = q ==> H = I
//        gKalmanFilter.R[Q0][Q0] = (real)R_VALS_QUAT;
//        gKalmanFilter.R[Q1][Q1] = (real)R_VALS_QUAT;
//        gKalmanFilter.R[Q2][Q2] = (real)R_VALS_QUAT;
//        gKalmanFilter.R[Q3][Q3] = (real)R_VALS_QUAT;
//    }
//}


// Quaternion-based updates functions follow:
// Matlab and c-simulation match to this point when Update is not performed and
//   low-pass filter matches the one in the matlab model (innovation/predicte
//   quaternion/filtered yaw-rate/...)
// ComputeSystemInnovation_AHRS.m
void ComputeSystemInnovation_AHRS_Q(void)
{
    // Compute the innovation, nu, between measured and predicted attitude
    gKalmanFilter.nu[STATE_Q0] = gKalmanFilter.measuredQuaternion[Q0] -
                                 gKalmanFilter.quaternion[Q0];
    gKalmanFilter.nu[STATE_Q1] = gKalmanFilter.measuredQuaternion[Q1] -
                                 gKalmanFilter.quaternion[Q1];
    gKalmanFilter.nu[STATE_Q2] = gKalmanFilter.measuredQuaternion[Q2] -
                                 gKalmanFilter.quaternion[Q2];
    gKalmanFilter.nu[STATE_Q3] = gKalmanFilter.measuredQuaternion[Q3] -
                                 gKalmanFilter.quaternion[Q3];

    // If needed, limit the error (quaternion limit) here

    // When the filtered yaw-rate is above certain thresholds then reduce the
    //   attitude-errors used to update roll and pitch.
    _TurnSwitch_Q();
}


// Update_AHRS.m
void Update_AHRS_Q(void)
{
    //                                      Matrix sizes for AHRS solution:
    _GenerateObservationCovariance_AHRS_Q();   // gKF.R [ 3 x 3 ] or [ 4 x 4 ]
    if (gAlgorithm.state == INS_SOLUTION) {
        if ((gAlgorithm.state == HIGH_GAIN_AHRS) ||
            (gAlgorithm.linAccelSwitch))
        {
#if 0
            // The following values are much better than the 0.09 values (below) WHEN USED WITHOUT GPS UPDATES!!!
            //   With GPS-aiding, the solution does not work with these values
            //   1) [ 0.0000002, 0.00005, 0.00005, 0.0005 ] --> RMS: [0.678098855880845 0.601335372028136 6.969036228492140]
            //   2) [ 0.000002, 0.0005, 0.0005, 0.005 ] --> RMS: [0.993137574832301 0.954378676319410 10.661500464861005]
            //   3) [ 0.0000001, 0.000025, 0.000025, 0.00025 ] --> RMS: [0.636450762325675 0.555028735388798 6.847206076343041]
//            [4.728601977804202 7.945296690856819 8.318840663872650e+02]
            gKalmanFilter.R_INS[STATE_Q0][STATE_Q0] = (real)0.0000002; //(real)0.0005;//9;//0.01;//0.003;
            gKalmanFilter.R_INS[STATE_Q1][STATE_Q1] = (real)0.00005; //(real)0.0005;//9;//0.01;//0.0050;
            gKalmanFilter.R_INS[STATE_Q2][STATE_Q2] = (real)0.00005; ///(real)0.0005;//9;//0.01;//0.0050;
            gKalmanFilter.R_INS[STATE_Q3][STATE_Q3] = (real)0.0005; //(real)0.0005;//9;//0.01;//0.0100;
#else
            // [ 0.09, 0.09, 0.09, 0.09 ] --> RMS: [1.400638217083542 1.216654561109760 12.243804006189830]
            gKalmanFilter.R_INS[STATE_Q0][STATE_Q0] = (real)0.003;//0.09;//0.01;//0.003;
            gKalmanFilter.R_INS[STATE_Q1][STATE_Q1] = (real)0.003;//0.09;//0.01;//0.0050;
            gKalmanFilter.R_INS[STATE_Q2][STATE_Q2] = (real)0.003;//0.09;//0.01;//0.0050;
            gKalmanFilter.R_INS[STATE_Q3][STATE_Q3] = (real)0.003;//0.09;//0.01;//0.0100;
#endif
        } else {
            gKalmanFilter.R_INS[STATE_Q0][STATE_Q0] = (real)0.09;//0.003;
            gKalmanFilter.R_INS[STATE_Q1][STATE_Q1] = (real)0.09;//0.0050;
            gKalmanFilter.R_INS[STATE_Q2][STATE_Q2] = (real)0.09;//0.0050;
            gKalmanFilter.R_INS[STATE_Q3][STATE_Q3] = (real)0.09;//0.0100;
        }
    }

    // S = H*P*HTrans + R (However the matrix math can be simplified since
    //                     H is very sparse!  P is fully populated)
    // PxHTranspose (cols 7:10 of P)
    for (rowNum = 0; rowNum < ROWS_IN_P; rowNum++) {
        PxHTranspose_Q[rowNum][Q0] = gKalmanFilter.P[rowNum][STATE_Q0];
        PxHTranspose_Q[rowNum][Q1] = gKalmanFilter.P[rowNum][STATE_Q1];
        PxHTranspose_Q[rowNum][Q2] = gKalmanFilter.P[rowNum][STATE_Q2];
        PxHTranspose_Q[rowNum][Q3] = gKalmanFilter.P[rowNum][STATE_Q3];
    }
//    HxPxHTranspose_Q[4][4]

    //// S = HxPxHTranspose + R (rows 7:10 and cols 7:10 of P PLUS diagonal of R)
    S_4x4[Q0][Q0] = gKalmanFilter.P[STATE_Q0][STATE_Q0] + gKalmanFilter.R_INS[STATE_Q0][STATE_Q0];
    S_4x4[Q0][Q1] = gKalmanFilter.P[STATE_Q0][STATE_Q1];
    S_4x4[Q0][Q2] = gKalmanFilter.P[STATE_Q0][STATE_Q2];
    S_4x4[Q0][Q3] = gKalmanFilter.P[STATE_Q0][STATE_Q3];

    S_4x4[Q1][Q0] = gKalmanFilter.P[STATE_Q1][STATE_Q0];
    S_4x4[Q1][Q1] = gKalmanFilter.P[STATE_Q1][STATE_Q1] + gKalmanFilter.R_INS[STATE_Q1][STATE_Q1];
    S_4x4[Q1][Q2] = gKalmanFilter.P[STATE_Q1][STATE_Q2];
    S_4x4[Q1][Q3] = gKalmanFilter.P[STATE_Q1][STATE_Q3];

    S_4x4[Q2][Q0] = gKalmanFilter.P[STATE_Q2][STATE_Q0];
    S_4x4[Q2][Q1] = gKalmanFilter.P[STATE_Q2][STATE_Q1];
    S_4x4[Q2][Q2] = gKalmanFilter.P[STATE_Q2][STATE_Q2] + gKalmanFilter.R_INS[STATE_Q2][STATE_Q2];
    S_4x4[Q2][Q3] = gKalmanFilter.P[STATE_Q2][STATE_Q3];

    S_4x4[Q3][Q0] = gKalmanFilter.P[STATE_Q3][STATE_Q0];
    S_4x4[Q3][Q1] = gKalmanFilter.P[STATE_Q3][STATE_Q1];
    S_4x4[Q3][Q2] = gKalmanFilter.P[STATE_Q3][STATE_Q2];
    S_4x4[Q3][Q3] = gKalmanFilter.P[STATE_Q3][STATE_Q3] + gKalmanFilter.R_INS[STATE_Q3][STATE_Q3];

    // Invert the S-Matrix (replace with sequential update)
    MatrixInverse_4x4(&S_4x4[0][0], &SInverse_4x4[0][0]);

    // Compute the Kalman gain: K = P*HTrans*SInv
    AxB(&PxHTranspose_Q[0][0], &SInverse_4x4[0][0], ROWS_IN_P, 4, 4, &gKalmanFilter.K_Q[0][0]);

    // Compute attitude-quaternion updates: Dx = K*nu
    // NOTE: Can access nu in the elements that the attitude error is stored BUT the
    //       value of ROWS_IN_H must be correct or the multiplication will be wrong
    AxV( &gKalmanFilter.K_Q[0][0],
         &gKalmanFilter.nu[STATE_Q0],
         NUMBER_OF_EKF_STATES,
         4,
         &stateUpdate[0] );

    // Update states based on computed deltas
    // --- attitude quaternions (q = q + Dq) ---
    gKalmanFilter.quaternion[Q0] = gKalmanFilter.quaternion[Q0] + stateUpdate[STATE_Q0];
    gKalmanFilter.quaternion[Q1] = gKalmanFilter.quaternion[Q1] + stateUpdate[STATE_Q1];
    gKalmanFilter.quaternion[Q2] = gKalmanFilter.quaternion[Q2] + stateUpdate[STATE_Q2];
    gKalmanFilter.quaternion[Q3] = gKalmanFilter.quaternion[Q3] + stateUpdate[STATE_Q3];

    // Normalize q and force q0 > 0
    QuatNormalize(&gKalmanFilter.quaternion[0]);

    // --- Angular-rate bias (wBias = wBias = DwBias) ---
    //     If magnetometers are not used then set the rate bias to zero???
    gKalmanFilter.rateBias_B[X_AXIS] = gKalmanFilter.rateBias_B[X_AXIS] + stateUpdate[STATE_WBX];
    gKalmanFilter.rateBias_B[Y_AXIS] = gKalmanFilter.rateBias_B[Y_AXIS] + stateUpdate[STATE_WBY];
    if ( gConfiguration.userBehavior.bit.useMags && gCalibration.productConfiguration.bit.hasMags) {
        gKalmanFilter.rateBias_B[Z_AXIS] = gKalmanFilter.rateBias_B[Z_AXIS] + stateUpdate[STATE_WBZ];
    } else {
        gKalmanFilter.rateBias_B[Z_AXIS] = 0.0;
    }

    // Update covariance: P = P + DP = P - K*H*P
    // KxH = gKF.K * gKF.H;
    //   2) Use gKalmanFilter.P as a temporary variable to hold FxPxFTranspose
    //      to reduce the number of "large" variables on the heap
    for (rowNum = 0; rowNum < NUMBER_OF_EKF_STATES; rowNum++) {
        KxH[rowNum][STATE_Q0] = gKalmanFilter.K_Q[rowNum][Q0];
        KxH[rowNum][STATE_Q1] = gKalmanFilter.K_Q[rowNum][Q1];
        KxH[rowNum][STATE_Q2] = gKalmanFilter.K_Q[rowNum][Q2];
        KxH[rowNum][STATE_Q3] = gKalmanFilter.K_Q[rowNum][Q3];
    }

    // Set the values in DP to zero
    memset(deltaP, 0, sizeof(deltaP));

    // Only modify the rows and cols that correspond to q and wb (the remainder of DP will be zero)
    //   (this is to attemp to fix the slow response in roll and pitch due to 'fast' motions that
    //   Parvez brought to my attention.  If this does not fix it then maybe the values of R are too
    //   large when the system is at rest -- change with aMag???)
    // deltaP = KxH * gKF.P;
    for (rowNum = STATE_Q0; rowNum <= STATE_WBZ; rowNum++) {
        for (colNum = STATE_Q0; colNum <= STATE_WBZ; colNum++) {
            deltaP[rowNum][colNum] = 0.0;
            for (multIndex = RLE_KxH[rowNum][0]; multIndex <= RLE_KxH[rowNum][1]; multIndex++) {
                deltaP[rowNum][colNum] = deltaP[rowNum][colNum] +
                                         KxH[rowNum][multIndex] * gKalmanFilter.P[multIndex][colNum];
            }
        }
    }

    // gKF.P = gKF.P - deltaP;
    AMinusB(&gKalmanFilter.P[0][0], &deltaP[0][0], ROWS_IN_P, COLS_IN_P, &gKalmanFilter.P[0][0]);

    // Ensure P is symmetric
    ForceMatrixSymmetry(&gKalmanFilter.P[0][0], ROWS_IN_P, COLS_IN_P);
}


// GenerateObservationCovarianceMatrix_AHRS.m
void _GenerateObservationCovariance_AHRS_Q(void)
{
    // Only need to compute R once
    static int oneTime = TRUE;

    if (oneTime) {
        oneTime = FALSE;

        // Clear the values in R (in AHRS mode, there are 3 rows in the Jacobian)
        // Initialize the Process Covariance (Q) matrix
        // DEBUG: Probably not needed
        memset(gKalmanFilter.R, 0, sizeof(gKalmanFilter.R));
        memset(gKalmanFilter.R_INS, 0, sizeof(gKalmanFilter.R_INS));

//        // h = q ==> H = I
//        gKalmanFilter.R[Q0][Q0] = (real)R_VALS_QUAT;
//        gKalmanFilter.R[Q1][Q1] = (real)R_VALS_QUAT;
//        gKalmanFilter.R[Q2][Q2] = (real)R_VALS_QUAT;
//        gKalmanFilter.R[Q3][Q3] = (real)R_VALS_QUAT;
    }

    // The values in the first column are the R-Values that correspond to the EA values.  The second column
    //   in the LG case are the sigma-values (not sigma-squared).  The final column contains old values.
    if ( (gAlgorithm.state == HIGH_GAIN_AHRS) ||
         (gAlgorithm.linAccelSwitch) )
    {
        gKalmanFilter.R_INS[STATE_Q0][STATE_Q0] = (real)3.198e-8; //(real)0.01; //
        gKalmanFilter.R_INS[STATE_Q1][STATE_Q1] = (real)2.499e-5; //(real)0.01; //
        gKalmanFilter.R_INS[STATE_Q2][STATE_Q2] = (real)2.495e-5; //(real)0.01; //
        gKalmanFilter.R_INS[STATE_Q3][STATE_Q3] = (real)2.502e-4; //(real)0.01; //
    } else {
        gKalmanFilter.R_INS[STATE_Q0][STATE_Q0] = (real)3.061e-4; //(real)1.742e-2; //(real)0.09; //
        gKalmanFilter.R_INS[STATE_Q1][STATE_Q1] = (real)7.510e-4; //(real)2.737e-2; //(real)0.09; //
        gKalmanFilter.R_INS[STATE_Q2][STATE_Q2] = (real)7.491e-4; //(real)2.737e-2; //(real)0.09; //
        gKalmanFilter.R_INS[STATE_Q3][STATE_Q3] = (real)2.436e-2; //(real)1.559e-1; //(real)0.09; //
    }
}


// TurnSwitch.m
static void _TurnSwitch_Q(void)
{
    static real minSwitch = 0.0, maxSwitch = 0.0;
    static real turnSwitchThresholdPast = 0.0;
    static real linInterpSF;

    real absYawRate;
    real turnSwitchScaleFactor;

    // gKF.filteredYawRate (calculated in the prediction stage)
    absYawRate = fabs(gAlgorithm.filteredYawRate);

    // In case the user changes the TST during operation
    if( gConfiguration.turnSwitchThreshold != turnSwitchThresholdPast ) {
        // Example conversion: ( 1820*12868 / 2^27 ) * ( 180/pi )
        minSwitch = gConfiguration.turnSwitchThreshold * (real)(TWO_PI / TWO_POW_16);   // angle in radians
        maxSwitch = (real)2.0 * minSwitch;   // angle in radians

        turnSwitchThresholdPast = gConfiguration.turnSwitchThreshold;

        linInterpSF = ((real)1.0 - TILT_YAW_SWITCH_GAIN) / (maxSwitch - minSwitch);
    }

    // Linear interpolation if the yawRate is above the specified threshold
    if( ( gAlgorithm.state > HIGH_GAIN_AHRS ) && ( absYawRate > minSwitch ) )
    {
        gAlgorithm.bitStatus.swStatus.bit.turnSwitch = TRUE;
        //        std::cout << "TurnSwitch (INS): Activated\n";

        // When the rate is below the maximum rate defined by
        //   turnSwitchThreshold, then generate a scale-factor that is between
        //   ( 1.0 - G ) and 0.0 (based on absYawRate).  If it is above
        //   'maxSwitch' then the SF is zero.
        if( absYawRate < maxSwitch ) {
            turnSwitchScaleFactor = linInterpSF * ( maxSwitch - absYawRate );
        } else {
            // yaw-rate is above maxSwitch ==> no gain
            turnSwitchScaleFactor = 0.0;
        }

        // Specify the multiplier so it is between G an 1.0
        real turnSwitchMultiplier = TILT_YAW_SWITCH_GAIN + turnSwitchScaleFactor;

        // Don't know what the reason for the 0.99 limit
        if( turnSwitchMultiplier < 0.99 ) {

            real err[3];

            // gKalmanFilter.measuredEulerAngles is computed in ComputeSystemInnovation_AHRS
            //   at 10Hz (prior to this function call)
            err[ROLL]  = gKalmanFilter.measuredEulerAngles[ROLL] -
                         gKalmanFilter.eulerAngles[ROLL];
            err[PITCH] = gKalmanFilter.measuredEulerAngles[PITCH] -
                         gKalmanFilter.eulerAngles[PITCH];
            err[YAW]   = gKalmanFilter.measuredEulerAngles[YAW] -
                         gKalmanFilter.eulerAngles[YAW];

            err[ROLL]  = _UnwrapAttitudeError( err[ROLL]  );
            err[PITCH] = _UnwrapAttitudeError( err[PITCH] );
            err[YAW]   = _UnwrapAttitudeError( err[YAW]   );

            // Limit error
            err[ROLL]  = _LimitValue( err[ROLL],  (real)LIMIT_ANG_ERROR_ROLL  );
            err[PITCH] = _LimitValue( err[PITCH], (real)LIMIT_ANG_ERROR_PITCH );

            err[ROLL]  = turnSwitchMultiplier * err[ROLL];
            err[PITCH] = turnSwitchMultiplier * err[PITCH];

            real limitedEulerAngles[3];
            real limitedQuat[4];

            limitedEulerAngles[ROLL]  = gKalmanFilter.eulerAngles[ROLL]  + err[ROLL];
            limitedEulerAngles[PITCH] = gKalmanFilter.eulerAngles[PITCH] + err[PITCH];
            limitedEulerAngles[YAW]   = gKalmanFilter.eulerAngles[YAW]   + err[YAW];

            EulerAnglesToQuaternion( &limitedEulerAngles[0], &limitedQuat[0] );

            // Normalize quaternion and force q0 to be positive
            QuatNormalize( &limitedQuat[Q0] );

            //
            gKalmanFilter.nu[STATE_Q0] = limitedQuat[Q0] - gKalmanFilter.quaternion[Q0];
            gKalmanFilter.nu[STATE_Q1] = limitedQuat[Q1] - gKalmanFilter.quaternion[Q1];
            gKalmanFilter.nu[STATE_Q2] = limitedQuat[Q2] - gKalmanFilter.quaternion[Q2];
            gKalmanFilter.nu[STATE_Q3] = limitedQuat[Q3] - gKalmanFilter.quaternion[Q3];
        }
    } else {
        gAlgorithm.bitStatus.swStatus.bit.turnSwitch = FALSE;
    }
}


