/*
 * File:   EKF_PredictionStage.cpp
 * Author: joemotyka
 *
 * Created on May 8, 2016, 12:23 AM
 */

#include "GlobalConstants.h"

#include <string.h>   // memset
#include <math.h>      // pow
#include "VectorMath.h"
#include "TransformationMath.h"

#include "EKF_Algorithm.h"

#include "xbowsp_algorithm.h"
#include "xbowsp_generaldrivers.h"
#include "PredictFunctions.h"

#include "MatrixMath.h"
#include "QuaternionMath.h"  // QuatNormalize

#include "Indices.h"       // IND
#include "StateIndices.h"  // STATE_IND
#include "TimingVars.h"    // timer
#include "AlgorithmLimits.h"

#include "SensorNoiseParameters.h"

#include "WorldMagneticModel.h"

#include "xbowsp_algorithm.h"

#ifdef INS_OFFLINE
#include "c:\Projects\software\sim\INS380_Offline\INS380_Offline\SimulationParameters.h"
#endif

//#include "RunLengthEncoding.h"
// F is sparse and has elements in the following locations...
uint8_t RLE_F[ROWS_IN_F][2] = { {  0,  3 },     // Row  0: cols 0,3
                                {  1,  4 },     // Row  1: cols 1,4
                                {  2,  5 },     // Row  2: cols 2,5
                                {  3, 15 },     // Row  3: cols 3,6:9,13:15
                                {  4, 15 },     // Row  4: cols 4,6:9,13:15
                                {  5, 15 },     // Row  5: cols 5,6:9,13:15
                                {  6, 12 },     // Row  6: cols 6:12
                                {  6, 12 },     // Row  7: cols 6:12
                                {  6, 12 },     // Row  8: cols 6:12
                                {  6, 12 },     // Row  9: cols 6:12
                                { 10, 10 },     // Row 10: cols 10
                                { 11, 11 },     // Row 11: cols 11
                                { 12, 12 },     // Row 12: cols 12
                                { 13, 13 },     // Row 13: cols 13
                                { 14, 14 },     // Row 14: cols 14
                                { 15, 15 } };   // Row 15: cols 15

// Q is sparse and has elements in the following locations...
uint8_t RLE_Q[ROWS_IN_F][2] = { {  0, 0 },
                                {  1, 1 },
                                {  2, 2 },
                                {  3, 3 },
                                {  4, 4 },
                                {  5, 5 },
                                {  6, 9 },
                                {  6, 9 },
                                {  6, 9 },
                                {  6, 9 },
                                { 10, 10 },
                                { 11, 11 },
                                { 12, 12 },
                                { 13, 13 },
                                { 14, 14 },
                                { 15, 15 } };

#include "ucb_packet.h"   // for UcbGetSysType() and UNAIDED_AHRS_SYS

// Local functions
static void _PredictStateEstimate(void);
static void _PredictCovarianceEstimate(void);

static void _UpdateProcessJacobian(void);
static void _UpdateProcessCovariance(void);

void _FirstOrderLowPass(real *output, real input);  // todo tm20160603 - use filters from filter.h, or move this filter there

static void _PopulateFilterCoefficients(void);

// 16 States: [ STATE_RX,  STATE_RY,  STATE_RZ, ...
//              STATE_VX,  STATE_VY,  STATE_VZ, ...
//              STATE_Q0,  STATE_Q1,  STATE_Q2,  STATE_Q3, ...
//              STATE_WBX, STATE_WBY, STATE_WBZ, ...
//              STATE_ABX, STATE_ABY, STATE_ABZ ]

// Filter variables (Third-Order BWF w/ 10 Hz Cutoff)
#define FILTER_ORDER 3

#define CURRENT 0
#define PASTx1  1
#define PASTx2  2
#define PASTx3  3

static real b_AccelFilt[4];
static real a_AccelFilt[4];

#define  NO_LPF              0
#define  FIVE_HZ_LPF         1
#define  TEN_HZ_LPF          2
#define  TWENTY_HZ_LPF       3
#define  TWENTY_FIVE_HZ_LPF  4
#define  N_LPF               5

// Floating-point filter variables
static real accelFilt[4][3];
static real accelReading[4][3];

//EKF_PredictionStage.m
void EKF_PredictionStage(void)
{
    real magFieldVector[3];

    //
    _PredictStateEstimate();        // x(k+1) = x(k) + f(x(k), u(k))
    _PredictCovarianceEstimate();   // P = F*P*FTrans + Q

    // Extract the predicted Euler angles from the predicted quaternion
    QuaternionToEulerAngles( gKalmanFilter.eulerAngles,
                             gKalmanFilter.quaternion );

    // Filter the yaw-rate here for the TURN-SWITCH (despite being used in the
    //   update stage only -- since that is a ten-hertz routine).  The way this
    //   is coded, the filter function can only be used for filtering yaw-rate
    //   data as the previous input state is saved as a static in the function.
    gAlgorithm.filteredYawRate = gAlgorithm.filteredYawRatePast;
    _FirstOrderLowPass( &gAlgorithm.filteredYawRate,
                        gKalmanFilter.correctedRate_B[Z_AXIS] );
    gAlgorithm.filteredYawRatePast = gAlgorithm.filteredYawRate;

    // ------- Compute the measured euler angles at 100 or 200 Hz -------
    // Would this be better? http://stackoverflow.com/questions/537244/default-constructor-in-c
    static int initAccelFilt = true;
    if (initAccelFilt) {
        initAccelFilt = false;

        // Set the filter coefficients based on selected cutoff frequency and sampling rate
        _PopulateFilterCoefficients();

        // Initialize the filter variables (do not need to populate the 0th element
        //   as it is never used)
        for( int i = 1; i < 4; i++ ) {
            accelReading[i][X_AXIS] = (real)gAlgorithm.scaledSensors[XACCEL];
            accelReading[i][Y_AXIS] = (real)gAlgorithm.scaledSensors[YACCEL];
            accelReading[i][Z_AXIS] = (real)gAlgorithm.scaledSensors[ZACCEL];

            accelFilt[i][X_AXIS] = (real)gAlgorithm.scaledSensors[XACCEL];
            accelFilt[i][Y_AXIS] = (real)gAlgorithm.scaledSensors[YACCEL];
            accelFilt[i][Z_AXIS] = (real)gAlgorithm.scaledSensors[ZACCEL];
        }
    }

    // Filter accelerometer readings (Note: a[0] =  1.0)
    //   y = filtered output; x = raw input;
    //
    // a[0]*y(k) + a[1]*y(k-1) + a[2]*y(k-2) + a[3]*y(k-3) =
    //    b[0]*x(k) + b[1]*x(k-1) + b[2]*x(k-2) + b[3]*x(k-3) =
    //    b[0]*( x(k) + x(k-3) ) + b[1]*( x(k-1) + x(k-2) )
    accelFilt[CURRENT][X_AXIS] = b_AccelFilt[0] * (real)gAlgorithm.scaledSensors[XACCEL] +
                                 b_AccelFilt[1] * ( accelReading[PASTx1][X_AXIS] +
                                                    accelReading[PASTx2][X_AXIS] ) +
                                 b_AccelFilt[3] * accelReading[PASTx3][X_AXIS] -
                                 a_AccelFilt[1] * accelFilt[PASTx1][X_AXIS] -
                                 a_AccelFilt[2] * accelFilt[PASTx2][X_AXIS] -
                                 a_AccelFilt[3] * accelFilt[PASTx3][X_AXIS];
    accelFilt[CURRENT][Y_AXIS] = b_AccelFilt[0] * (real)gAlgorithm.scaledSensors[YACCEL] +
                                 b_AccelFilt[1] * ( accelReading[PASTx1][Y_AXIS] +
                                                    accelReading[PASTx2][Y_AXIS] ) +
                                 b_AccelFilt[3] * accelReading[PASTx3][Y_AXIS] -
                                 a_AccelFilt[1] * accelFilt[PASTx1][Y_AXIS] -
                                 a_AccelFilt[2] * accelFilt[PASTx2][Y_AXIS] -
                                 a_AccelFilt[3] * accelFilt[PASTx3][Y_AXIS];
    accelFilt[CURRENT][Z_AXIS] = b_AccelFilt[0] * (real)gAlgorithm.scaledSensors[ZACCEL] +
                                 b_AccelFilt[1] * ( accelReading[PASTx1][Z_AXIS] +
                                                    accelReading[PASTx2][Z_AXIS] ) +
                                 b_AccelFilt[3] * accelReading[PASTx3][Z_AXIS] -
                                 a_AccelFilt[1] * accelFilt[PASTx1][Z_AXIS] -
                                 a_AccelFilt[2] * accelFilt[PASTx2][Z_AXIS] -
                                 a_AccelFilt[3] * accelFilt[PASTx3][Z_AXIS];

    // Update 'past' readings
    accelReading[PASTx3][X_AXIS] = accelReading[PASTx2][X_AXIS];
    accelReading[PASTx2][X_AXIS] = accelReading[PASTx1][X_AXIS];
    accelReading[PASTx1][X_AXIS] = (real)gAlgorithm.scaledSensors[XACCEL];

    accelReading[PASTx3][Y_AXIS] = accelReading[PASTx2][Y_AXIS];
    accelReading[PASTx2][Y_AXIS] = accelReading[PASTx1][Y_AXIS];
    accelReading[PASTx1][Y_AXIS] = (real)gAlgorithm.scaledSensors[YACCEL];

    accelReading[PASTx3][Z_AXIS] = accelReading[PASTx2][Z_AXIS];
    accelReading[PASTx2][Z_AXIS] = accelReading[PASTx1][Z_AXIS];
    accelReading[PASTx1][Z_AXIS] = (real)gAlgorithm.scaledSensors[ZACCEL];

    accelFilt[PASTx3][X_AXIS] = accelFilt[PASTx2][X_AXIS];
    accelFilt[PASTx2][X_AXIS] = accelFilt[PASTx1][X_AXIS];
    accelFilt[PASTx1][X_AXIS] = accelFilt[CURRENT][X_AXIS];

    accelFilt[PASTx3][Y_AXIS] = accelFilt[PASTx2][Y_AXIS];
    accelFilt[PASTx2][Y_AXIS] = accelFilt[PASTx1][Y_AXIS];
    accelFilt[PASTx1][Y_AXIS] = accelFilt[CURRENT][Y_AXIS];

    accelFilt[PASTx3][Z_AXIS] = accelFilt[PASTx2][Z_AXIS];
    accelFilt[PASTx2][Z_AXIS] = accelFilt[PASTx1][Z_AXIS];
    accelFilt[PASTx1][Z_AXIS] = accelFilt[CURRENT][Z_AXIS];

#if 1
    gAlgorithm.aMag = sqrt( accelFilt[CURRENT][X_AXIS] * accelFilt[CURRENT][X_AXIS] +
                            accelFilt[CURRENT][Y_AXIS] * accelFilt[CURRENT][Y_AXIS] +
                            accelFilt[CURRENT][Z_AXIS] * accelFilt[CURRENT][Z_AXIS] );
#else
    // Compute the roll/pitch/yaw and use this to find the components of acceleration
    //   in the x/y/z directions.  Compare each to a limit.
#endif

    // Check for times when the acceleration is 'close' to 1 [g].  When this occurs,
    //   increment a counter.  When it exceeds a threshold (indicating that the system
    //   has been at rest for a given period) then decrease the R-values, effectively
    //   increasing the Kalman gain.
    if (fabs( 1.0 - gAlgorithm.aMag ) < gAlgorithm.Limit.accelSwitch ) {
        gAlgorithm.linAccelSwitchCntr++;
        if ( gAlgorithm.linAccelSwitchCntr >= gAlgorithm.Limit.linAccelSwitchDelay ) {
            gAlgorithm.linAccelSwitch = TRUE;
        } else {
            gAlgorithm.linAccelSwitch = FALSE;
        }
    } else {
        gAlgorithm.linAccelSwitchCntr = 0;
        gAlgorithm.linAccelSwitch     = FALSE;
    }
//    gAlgorithm.bitStatus.swStatus.bit.highGain = gAlgorithm.linAccelSwitch;

    // Extract the magnetometer readings (set to zero if the magnetometer is not
    //   present or unused).
    if( gCalibration.productConfiguration.bit.hasMags &&
        gConfiguration.userBehavior.bit.useMags )
    {
        magFieldVector[X_AXIS] = (real)gAlgorithm.scaledSensors[XMAG];
        magFieldVector[Y_AXIS] = (real)gAlgorithm.scaledSensors[YMAG];
        magFieldVector[Z_AXIS] = (real)gAlgorithm.scaledSensors[ZMAG];
    } else {
        magFieldVector[X_AXIS] = (real)0.0;
        magFieldVector[Y_AXIS] = (real)0.0;
        magFieldVector[Z_AXIS] = (real)0.0;
    }

    // Compute the measured Euler angles from gravity and magnetic field data
    //   ( phiMeas, thetaMeas, psiMeas ) = f( g_B, mMeas_B ).  Adjust for
    //   declination.
    FieldVectorsToEulerAngles( &accelFilt[CURRENT][0],
                               magFieldVector,
                               gAlgorithm.state == LOW_GAIN_AHRS,  // use pred to level when in LG
                               gKalmanFilter.measuredEulerAngles );

    // Should this apply to VG data???
    if( gAlgorithm.applyDeclFlag ) {
        gKalmanFilter.measuredEulerAngles[YAW] = gKalmanFilter.measuredEulerAngles[YAW] +
                                                 gWorldMagModel.decl_rad;
    }

    // Compute the attitude quaternion from sensor measurements (Euler angles).  This is used in
    //   innovation calculation.
    EulerAnglesToQuaternion(gKalmanFilter.measuredEulerAngles, gKalmanFilter.measuredQuaternion);
// ************ Compute the measured Euler Angles and associated quaternion ************
}


// PredictStateEstimate.m
static void _PredictStateEstimate(void)
{
    real aCorr_N[3];
    real deltaQuaternion[4];

    static real GRAVITY_VECTOR_N_FRAME[3] = { (real)0.0, (real)0.0, (real)(-GRAVITY) };

    // Predict the EKF states at 100 Hz based on readings from the:
    //  - accelerometer
    //  - angular-rate sensors

    // Generate the transformation matrix (R_BinN) based on the past value of
    //   the attitude quaternion (prior to prediction at the new time-step)
    QuaternionToR321(gKalmanFilter.quaternion, &gKalmanFilter.R_BinN[0][0]);

// (FIXME) JSM: if unaided then do not integrate the position or velocity
//if( UcbGetSysType() > UNAIDED_AHRS_SYS ) {
    // ================= NED Position (r_N) =================
    // r_N(k+1) = r_N(k) + dV_N = r_N(k) + v_N*DT
    gKalmanFilter.Position_N[X_AXIS] = gKalmanFilter.Position_N[X_AXIS] +
                                       gKalmanFilter.Velocity_N[X_AXIS] * gAlgorithm.dt;
    gKalmanFilter.Position_N[Y_AXIS] = gKalmanFilter.Position_N[Y_AXIS] +
                                       gKalmanFilter.Velocity_N[Y_AXIS] * gAlgorithm.dt;
    gKalmanFilter.Position_N[Z_AXIS] = gKalmanFilter.Position_N[Z_AXIS] +
                                       gKalmanFilter.Velocity_N[Z_AXIS] * gAlgorithm.dt;

    // ================= NED Velocity (v_N) =================
    // aCorr_B = aMeas_B - aBias_B
    // scaledSensors accels in g's, convert to m/s^2
    gKalmanFilter.correctedAccel_B[X_AXIS] = (real)(gAlgorithm.scaledSensors[XACCEL] * GRAVITY) -
                                             gKalmanFilter.accelBias_B[X_AXIS];
    gKalmanFilter.correctedAccel_B[Y_AXIS] = (real)(gAlgorithm.scaledSensors[YACCEL] * GRAVITY) -
                                             gKalmanFilter.accelBias_B[Y_AXIS];
    gKalmanFilter.correctedAccel_B[Z_AXIS] = (real)(gAlgorithm.scaledSensors[ZACCEL] * GRAVITY) -
                                             gKalmanFilter.accelBias_B[Z_AXIS];

    // Transform the corrected acceleration vector from the body to the NED-frame
    // a_N = R_BinN * a_B
    aCorr_N[X_AXIS] = gKalmanFilter.R_BinN[X_AXIS][X_AXIS] * gKalmanFilter.correctedAccel_B[X_AXIS] +
                      gKalmanFilter.R_BinN[X_AXIS][Y_AXIS] * gKalmanFilter.correctedAccel_B[Y_AXIS] +
                      gKalmanFilter.R_BinN[X_AXIS][Z_AXIS] * gKalmanFilter.correctedAccel_B[Z_AXIS];
    aCorr_N[Y_AXIS] = gKalmanFilter.R_BinN[Y_AXIS][X_AXIS] * gKalmanFilter.correctedAccel_B[X_AXIS] +
                      gKalmanFilter.R_BinN[Y_AXIS][Y_AXIS] * gKalmanFilter.correctedAccel_B[Y_AXIS] +
                      gKalmanFilter.R_BinN[Y_AXIS][Z_AXIS] * gKalmanFilter.correctedAccel_B[Z_AXIS];
    aCorr_N[Z_AXIS] = gKalmanFilter.R_BinN[Z_AXIS][X_AXIS] * gKalmanFilter.correctedAccel_B[X_AXIS] +
                      gKalmanFilter.R_BinN[Z_AXIS][Y_AXIS] * gKalmanFilter.correctedAccel_B[Y_AXIS] +
                      gKalmanFilter.R_BinN[Z_AXIS][Z_AXIS] * gKalmanFilter.correctedAccel_B[Z_AXIS];

    // Determine the acceleration of the system by removing the gravity vector
    // v_N(k+1) = v_N(k) + dV = v_N(k) + aMotion_N*DT = v_N(k) + ( a_N - g_N )*DT
    gKalmanFilter.Velocity_N[X_AXIS] = gKalmanFilter.Velocity_N[X_AXIS] +
                                       ( aCorr_N[X_AXIS] - GRAVITY_VECTOR_N_FRAME[X_AXIS] ) * gAlgorithm.dt;
    gKalmanFilter.Velocity_N[Y_AXIS] = gKalmanFilter.Velocity_N[Y_AXIS] +
                                       ( aCorr_N[Y_AXIS] - GRAVITY_VECTOR_N_FRAME[Y_AXIS] ) * gAlgorithm.dt;
    gKalmanFilter.Velocity_N[Z_AXIS] = gKalmanFilter.Velocity_N[Z_AXIS] +
                                       ( aCorr_N[Z_AXIS] - GRAVITY_VECTOR_N_FRAME[Z_AXIS] ) * gAlgorithm.dt;
//}

    // ================= Attitude quaternion =================
    // Find the 'true' angular rate (wTrue_B = wCorr_B = wMeas_B - wBias_B)
    gKalmanFilter.correctedRate_B[X_AXIS] = (real)gAlgorithm.scaledSensors[XRATE] -
                                            gKalmanFilter.rateBias_B[X_AXIS];
    gKalmanFilter.correctedRate_B[Y_AXIS] = (real)gAlgorithm.scaledSensors[YRATE] -
                                            gKalmanFilter.rateBias_B[Y_AXIS];
    gKalmanFilter.correctedRate_B[Z_AXIS] = (real)gAlgorithm.scaledSensors[ZRATE] -
                                            gKalmanFilter.rateBias_B[Z_AXIS];

    // Placed in gKalmanFilter as wTrueTimesDtOverTwo is used to compute the Jacobian (F)
    gKalmanFilter.wTrueTimesDtOverTwo[X_AXIS] = gKalmanFilter.correctedRate_B[X_AXIS] * gAlgorithm.dtOverTwo;
    gKalmanFilter.wTrueTimesDtOverTwo[Y_AXIS] = gKalmanFilter.correctedRate_B[Y_AXIS] * gAlgorithm.dtOverTwo;
    gKalmanFilter.wTrueTimesDtOverTwo[Z_AXIS] = gKalmanFilter.correctedRate_B[Z_AXIS] * gAlgorithm.dtOverTwo;

    // Find the attitude change based on angular rate data
    deltaQuaternion[Q0] = -gKalmanFilter.wTrueTimesDtOverTwo[X_AXIS] * gKalmanFilter.quaternion[Q1] +
                          -gKalmanFilter.wTrueTimesDtOverTwo[Y_AXIS] * gKalmanFilter.quaternion[Q2] +
                          -gKalmanFilter.wTrueTimesDtOverTwo[Z_AXIS] * gKalmanFilter.quaternion[Q3];
    deltaQuaternion[Q1] =  gKalmanFilter.wTrueTimesDtOverTwo[X_AXIS] * gKalmanFilter.quaternion[Q0] +
                           gKalmanFilter.wTrueTimesDtOverTwo[Z_AXIS] * gKalmanFilter.quaternion[Q2] +
                          -gKalmanFilter.wTrueTimesDtOverTwo[Y_AXIS] * gKalmanFilter.quaternion[Q3];
    deltaQuaternion[Q2] =  gKalmanFilter.wTrueTimesDtOverTwo[Y_AXIS] * gKalmanFilter.quaternion[Q0] +
                          -gKalmanFilter.wTrueTimesDtOverTwo[Z_AXIS] * gKalmanFilter.quaternion[Q1] +
                           gKalmanFilter.wTrueTimesDtOverTwo[X_AXIS] * gKalmanFilter.quaternion[Q3];
    deltaQuaternion[Q3] =  gKalmanFilter.wTrueTimesDtOverTwo[Z_AXIS] * gKalmanFilter.quaternion[Q0] +
                           gKalmanFilter.wTrueTimesDtOverTwo[Y_AXIS] * gKalmanFilter.quaternion[Q1] +
                          -gKalmanFilter.wTrueTimesDtOverTwo[X_AXIS] * gKalmanFilter.quaternion[Q2];

    // Save the past attitude quaternion before updating (for use in the
    //   covariance prediction)
    gKalmanFilter.quaternion_Past[Q0] = gKalmanFilter.quaternion[Q0];
    gKalmanFilter.quaternion_Past[Q1] = gKalmanFilter.quaternion[Q1];
    gKalmanFilter.quaternion_Past[Q2] = gKalmanFilter.quaternion[Q2];
    gKalmanFilter.quaternion_Past[Q3] = gKalmanFilter.quaternion[Q3];

    // Update the attitude
    // q_BinN(k+1) = q_BinN(k) + dq = q_BinN(k) + OMEGA*q_BinN(k)
    gKalmanFilter.quaternion[Q0] = gKalmanFilter.quaternion[Q0] + deltaQuaternion[Q0];
    gKalmanFilter.quaternion[Q1] = gKalmanFilter.quaternion[Q1] + deltaQuaternion[Q1];
    gKalmanFilter.quaternion[Q2] = gKalmanFilter.quaternion[Q2] + deltaQuaternion[Q2];
    gKalmanFilter.quaternion[Q3] = gKalmanFilter.quaternion[Q3] + deltaQuaternion[Q3];

    // Normalize quaternion and force q0 to be positive
    QuatNormalize(gKalmanFilter.quaternion);

    // ================= Angular-rate bias: wBias(k+1) = wBias(k) =================
    // N/A (predicted state is same as past state)

    // ================= Linear-acceleration bias: aBias(k+1) = aBias(k) =================
    // N/A (predicted state is same as past state)
}


// Define variables that reside on the heap
real PxFTranspose[ROWS_IN_P][ROWS_IN_F], FxPxFTranspose[ROWS_IN_F][ROWS_IN_F];

// PredictCovarianceEstimate.m
static void _PredictCovarianceEstimate(void)
{
    uint8_t rowNum, colNum, multIndex;

    // Compute the F and Q matrices used in the prediction stage (only certain
    //   elements in the process-covariance, Q, change with each time-step)
    _UpdateProcessJacobian();     // gKF.F  (16x16)
    _UpdateProcessCovariance();   // gKF.Q  (16x16)

    // Update P from the P, F, and Q matrices: P = FxPxFTranspose + Q
    //   1) PxFTranspose is computed first
    for (rowNum = 0; rowNum < ROWS_IN_P; rowNum++) {
        for (colNum = 0; colNum < ROWS_IN_F; colNum++) {
            PxFTranspose[rowNum][colNum] = 0.0;
            for (multIndex = RLE_F[colNum][0]; multIndex <= RLE_F[colNum][1]; multIndex++) {
                PxFTranspose[rowNum][colNum] = PxFTranspose[rowNum][colNum] +
                    gKalmanFilter.P[rowNum][multIndex] * gKalmanFilter.F[colNum][multIndex];
            }
        }
    }

    //   2) Use gKalmanFilter.P as a temporary variable to hold FxPxFTranspose
    //      to reduce the number of "large" variables on the heap
    for (rowNum = 0; rowNum < 16; rowNum++) {
        for (colNum = 0; colNum < 16; colNum++) {
            gKalmanFilter.P[rowNum][colNum] = 0.0;
            for (multIndex = RLE_F[rowNum][0]; multIndex <= RLE_F[rowNum][1]; multIndex++) {
                gKalmanFilter.P[rowNum][colNum] = gKalmanFilter.P[rowNum][colNum] +
                    gKalmanFilter.F[rowNum][multIndex] * PxFTranspose[multIndex][colNum];
            }
        }
    }

    //   3) Finally, add Q to FxPxFTranspose (P) to get the final value for
    //      gKalmanFilter.P (only the elements of Q that are nonzero)
    gKalmanFilter.P[STATE_RX][STATE_RX] = gKalmanFilter.P[STATE_RX][STATE_RX] + gKalmanFilter.Q[STATE_RX][STATE_RX];
    gKalmanFilter.P[STATE_RY][STATE_RY] = gKalmanFilter.P[STATE_RY][STATE_RY] + gKalmanFilter.Q[STATE_RY][STATE_RY];
    gKalmanFilter.P[STATE_RZ][STATE_RZ] = gKalmanFilter.P[STATE_RZ][STATE_RZ] + gKalmanFilter.Q[STATE_RZ][STATE_RZ];

    gKalmanFilter.P[STATE_VX][STATE_VX] = gKalmanFilter.P[STATE_VX][STATE_VX] + gKalmanFilter.Q[STATE_VX][STATE_VX];
    gKalmanFilter.P[STATE_VY][STATE_VY] = gKalmanFilter.P[STATE_VY][STATE_VY] + gKalmanFilter.Q[STATE_VY][STATE_VY];
    gKalmanFilter.P[STATE_VZ][STATE_VZ] = gKalmanFilter.P[STATE_VZ][STATE_VZ] + gKalmanFilter.Q[STATE_VZ][STATE_VZ];

    gKalmanFilter.P[STATE_Q0][STATE_Q0] = gKalmanFilter.P[STATE_Q0][STATE_Q0] + gKalmanFilter.Q[STATE_Q0][STATE_Q0];
    gKalmanFilter.P[STATE_Q0][STATE_Q1] = gKalmanFilter.P[STATE_Q0][STATE_Q1] + gKalmanFilter.Q[STATE_Q0][STATE_Q1];
    gKalmanFilter.P[STATE_Q0][STATE_Q2] = gKalmanFilter.P[STATE_Q0][STATE_Q2] + gKalmanFilter.Q[STATE_Q0][STATE_Q2];
    gKalmanFilter.P[STATE_Q0][STATE_Q3] = gKalmanFilter.P[STATE_Q0][STATE_Q3] + gKalmanFilter.Q[STATE_Q0][STATE_Q3];

    gKalmanFilter.P[STATE_Q1][STATE_Q0] = gKalmanFilter.P[STATE_Q1][STATE_Q0] + gKalmanFilter.Q[STATE_Q1][STATE_Q0];
    gKalmanFilter.P[STATE_Q1][STATE_Q1] = gKalmanFilter.P[STATE_Q1][STATE_Q1] + gKalmanFilter.Q[STATE_Q1][STATE_Q1];
    gKalmanFilter.P[STATE_Q1][STATE_Q2] = gKalmanFilter.P[STATE_Q1][STATE_Q2] + gKalmanFilter.Q[STATE_Q1][STATE_Q2];
    gKalmanFilter.P[STATE_Q1][STATE_Q3] = gKalmanFilter.P[STATE_Q1][STATE_Q3] + gKalmanFilter.Q[STATE_Q1][STATE_Q3];

    gKalmanFilter.P[STATE_Q2][STATE_Q0] = gKalmanFilter.P[STATE_Q2][STATE_Q0] + gKalmanFilter.Q[STATE_Q2][STATE_Q0];
    gKalmanFilter.P[STATE_Q2][STATE_Q1] = gKalmanFilter.P[STATE_Q2][STATE_Q1] + gKalmanFilter.Q[STATE_Q2][STATE_Q1];
    gKalmanFilter.P[STATE_Q2][STATE_Q2] = gKalmanFilter.P[STATE_Q2][STATE_Q2] + gKalmanFilter.Q[STATE_Q2][STATE_Q2];
    gKalmanFilter.P[STATE_Q2][STATE_Q3] = gKalmanFilter.P[STATE_Q2][STATE_Q3] + gKalmanFilter.Q[STATE_Q2][STATE_Q3];

    gKalmanFilter.P[STATE_Q3][STATE_Q0] = gKalmanFilter.P[STATE_Q3][STATE_Q0] + gKalmanFilter.Q[STATE_Q3][STATE_Q0];
    gKalmanFilter.P[STATE_Q3][STATE_Q1] = gKalmanFilter.P[STATE_Q3][STATE_Q1] + gKalmanFilter.Q[STATE_Q3][STATE_Q1];
    gKalmanFilter.P[STATE_Q3][STATE_Q2] = gKalmanFilter.P[STATE_Q3][STATE_Q2] + gKalmanFilter.Q[STATE_Q3][STATE_Q2];
    gKalmanFilter.P[STATE_Q3][STATE_Q3] = gKalmanFilter.P[STATE_Q3][STATE_Q3] + gKalmanFilter.Q[STATE_Q3][STATE_Q3];

    gKalmanFilter.P[STATE_WBX][STATE_WBX] = gKalmanFilter.P[STATE_WBX][STATE_WBX] + gKalmanFilter.Q[STATE_WBX][STATE_WBX];
    gKalmanFilter.P[STATE_WBY][STATE_WBY] = gKalmanFilter.P[STATE_WBY][STATE_WBY] + gKalmanFilter.Q[STATE_WBY][STATE_WBY];
    gKalmanFilter.P[STATE_WBZ][STATE_WBZ] = gKalmanFilter.P[STATE_WBZ][STATE_WBZ] + gKalmanFilter.Q[STATE_WBZ][STATE_WBZ];

    gKalmanFilter.P[STATE_ABX][STATE_ABX] = gKalmanFilter.P[STATE_ABX][STATE_ABX] + gKalmanFilter.Q[STATE_ABX][STATE_ABX];
    gKalmanFilter.P[STATE_ABY][STATE_ABY] = gKalmanFilter.P[STATE_ABY][STATE_ABY] + gKalmanFilter.Q[STATE_ABY][STATE_ABY];
    gKalmanFilter.P[STATE_ABZ][STATE_ABZ] = gKalmanFilter.P[STATE_ABZ][STATE_ABZ] + gKalmanFilter.Q[STATE_ABZ][STATE_ABZ];

    // P is a fully populated matrix (nominally) so all the elements of the matrix have to be
    //   considered when working with it.
    ForceMatrixSymmetry(&gKalmanFilter.P[0][0], ROWS_IN_P, COLS_IN_P);
}


// GenerateProcessJacobian.m: Set the elements of F that DO NOT change with each time-step
void GenerateProcessJacobian(void)
{
    // Initialize the Process Jacobian matrix (F)
    memset(gKalmanFilter.F, 0, sizeof(gKalmanFilter.F));

    // Form the process Jacobian

    // ---------- Rows corresponding to POSITION ----------
    gKalmanFilter.F[STATE_RX][STATE_VX] = gAlgorithm.dt;
    gKalmanFilter.F[STATE_RY][STATE_VY] = gAlgorithm.dt;
    gKalmanFilter.F[STATE_RZ][STATE_VZ] = gAlgorithm.dt;

    // ---------- Rows corresponding to VELOCITY ----------

    // ---------- Rows corresponding to RATE-BIAS ----------

    // ---------- Rows corresponding to ACCELERATION-BIAS ----------

    // ---------- Add to I16 to get final formulation of F ----------
    // Populate the diagonals of F with 1.0
    gKalmanFilter.F[STATE_RX][STATE_RX] = 1.0;
    gKalmanFilter.F[STATE_RY][STATE_RY] = 1.0;
    gKalmanFilter.F[STATE_RZ][STATE_RZ] = 1.0;

    gKalmanFilter.F[STATE_VX][STATE_VX] = 1.0;
    gKalmanFilter.F[STATE_VY][STATE_VY] = 1.0;
    gKalmanFilter.F[STATE_VZ][STATE_VZ] = 1.0;

    gKalmanFilter.F[STATE_Q0][STATE_Q0] = 1.0;
    gKalmanFilter.F[STATE_Q1][STATE_Q1] = 1.0;
    gKalmanFilter.F[STATE_Q2][STATE_Q2] = 1.0;
    gKalmanFilter.F[STATE_Q3][STATE_Q3] = 1.0;

    // The elements in F that correspond to the rate and acceleration bias
    //   are 1.0.  Note: previous values of 0.998 were used in the Matlab
    //   model.  This was due to an error in the implementation of the
    //   matrix inverse.  Fixed the inverse and the correct values (of 1.0)
    //   work for both the AHRS and INS.
    gKalmanFilter.F[STATE_WBX][STATE_WBX] = (real)1.0;
    gKalmanFilter.F[STATE_WBY][STATE_WBY] = (real)1.0;
    gKalmanFilter.F[STATE_WBZ][STATE_WBZ] = (real)1.0;

    gKalmanFilter.F[STATE_ABX][STATE_ABX] = (real)1.0;
    gKalmanFilter.F[STATE_ABY][STATE_ABY] = (real)1.0;
    gKalmanFilter.F[STATE_ABZ][STATE_ABZ] = (real)1.0;
}


// _UpdateProcessJacobian.m: Update the elements of F that change with each time-step
static void _UpdateProcessJacobian(void)
{
    real q0aXdT, q1aXdT, q2aXdT, q3aXdT;
    real q0aYdT, q1aYdT, q2aYdT, q3aYdT;
    real q0aZdT, q1aZdT, q2aZdT, q3aZdT;
    real q0DtOver2, q1DtOver2, q2DtOver2, q3DtOver2;

    // ---------- Rows corresponding to POSITION ----------
    // No updates

    // ---------- Rows corresponding to VELOCITY ----------
    // Columns corresponding to the attitude-quaternion states
    q0aXdT = gKalmanFilter.quaternion_Past[Q0] * gKalmanFilter.correctedAccel_B[X_AXIS] * gAlgorithm.dt;
    q1aXdT = gKalmanFilter.quaternion_Past[Q1] * gKalmanFilter.correctedAccel_B[X_AXIS] * gAlgorithm.dt;
    q2aXdT = gKalmanFilter.quaternion_Past[Q2] * gKalmanFilter.correctedAccel_B[X_AXIS] * gAlgorithm.dt;
    q3aXdT = gKalmanFilter.quaternion_Past[Q3] * gKalmanFilter.correctedAccel_B[X_AXIS] * gAlgorithm.dt;

    q0aYdT = gKalmanFilter.quaternion_Past[Q0] * gKalmanFilter.correctedAccel_B[Y_AXIS] * gAlgorithm.dt;
    q1aYdT = gKalmanFilter.quaternion_Past[Q1] * gKalmanFilter.correctedAccel_B[Y_AXIS] * gAlgorithm.dt;
    q2aYdT = gKalmanFilter.quaternion_Past[Q2] * gKalmanFilter.correctedAccel_B[Y_AXIS] * gAlgorithm.dt;
    q3aYdT = gKalmanFilter.quaternion_Past[Q3] * gKalmanFilter.correctedAccel_B[Y_AXIS] * gAlgorithm.dt;

    q0aZdT = gKalmanFilter.quaternion_Past[Q0] * gKalmanFilter.correctedAccel_B[Z_AXIS] * gAlgorithm.dt;
    q1aZdT = gKalmanFilter.quaternion_Past[Q1] * gKalmanFilter.correctedAccel_B[Z_AXIS] * gAlgorithm.dt;
    q2aZdT = gKalmanFilter.quaternion_Past[Q2] * gKalmanFilter.correctedAccel_B[Z_AXIS] * gAlgorithm.dt;
    q3aZdT = gKalmanFilter.quaternion_Past[Q3] * gKalmanFilter.correctedAccel_B[Z_AXIS] * gAlgorithm.dt;

    //
    gKalmanFilter.F[STATE_VX][STATE_Q0] = (real)2.0 * (  q0aXdT - q3aYdT + q2aZdT );
    gKalmanFilter.F[STATE_VX][STATE_Q1] = (real)2.0 * (  q1aXdT + q2aYdT + q3aZdT );
    gKalmanFilter.F[STATE_VX][STATE_Q2] = (real)2.0 * ( -q2aXdT + q1aYdT + q0aZdT );
    gKalmanFilter.F[STATE_VX][STATE_Q3] = (real)2.0 * ( -q3aXdT - q0aYdT + q1aZdT );

    gKalmanFilter.F[STATE_VY][STATE_Q0] = (real)2.0 * (  q3aXdT + q0aYdT - q1aZdT );
    gKalmanFilter.F[STATE_VY][STATE_Q1] = (real)2.0 * (  q2aXdT - q1aYdT - q0aZdT );
    gKalmanFilter.F[STATE_VY][STATE_Q2] = (real)2.0 * (  q1aXdT + q2aYdT + q3aZdT );
    gKalmanFilter.F[STATE_VY][STATE_Q3] = (real)2.0 * (  q0aXdT - q3aYdT + q2aZdT );

    gKalmanFilter.F[STATE_VZ][STATE_Q0] = (real)2.0 * ( -q2aXdT + q1aYdT + q0aZdT );
    gKalmanFilter.F[STATE_VZ][STATE_Q1] = (real)2.0 * (  q3aXdT + q0aYdT - q1aZdT );
    gKalmanFilter.F[STATE_VZ][STATE_Q2] = (real)2.0 * ( -q0aXdT + q3aYdT - q2aZdT );
    gKalmanFilter.F[STATE_VZ][STATE_Q3] = (real)2.0 * (  q1aXdT + q2aYdT + q3aZdT );

    // Columns corresponding to the acceleration-bias state (-R_BinN*DT)
    gKalmanFilter.F[STATE_VX][STATE_ABX] = -gKalmanFilter.R_BinN[0][0] * gAlgorithm.dt;
    gKalmanFilter.F[STATE_VX][STATE_ABY] = -gKalmanFilter.R_BinN[0][1] * gAlgorithm.dt;
    gKalmanFilter.F[STATE_VX][STATE_ABZ] = -gKalmanFilter.R_BinN[0][2] * gAlgorithm.dt;

    gKalmanFilter.F[STATE_VY][STATE_ABX] = -gKalmanFilter.R_BinN[1][0] * gAlgorithm.dt;
    gKalmanFilter.F[STATE_VY][STATE_ABY] = -gKalmanFilter.R_BinN[1][1] * gAlgorithm.dt;
    gKalmanFilter.F[STATE_VY][STATE_ABZ] = -gKalmanFilter.R_BinN[1][2] * gAlgorithm.dt;

    gKalmanFilter.F[STATE_VZ][STATE_ABX] = -gKalmanFilter.R_BinN[2][0] * gAlgorithm.dt;
    gKalmanFilter.F[STATE_VZ][STATE_ABY] = -gKalmanFilter.R_BinN[2][1] * gAlgorithm.dt;
    gKalmanFilter.F[STATE_VZ][STATE_ABZ] = -gKalmanFilter.R_BinN[2][2] * gAlgorithm.dt;

    // ---------- Rows corresponding to attitude-QUATERNION ----------
    // Columns corresponding to the attitude-quaternion state (0.5*Omega*DT)
    //gKalmanFilter.F[STATE_Q0][STATE_Q0] =     0;
    gKalmanFilter.F[STATE_Q0][STATE_Q1] = -gKalmanFilter.wTrueTimesDtOverTwo[X_AXIS];
    gKalmanFilter.F[STATE_Q0][STATE_Q2] = -gKalmanFilter.wTrueTimesDtOverTwo[Y_AXIS];
    gKalmanFilter.F[STATE_Q0][STATE_Q3] = -gKalmanFilter.wTrueTimesDtOverTwo[Z_AXIS];

    gKalmanFilter.F[STATE_Q1][STATE_Q0] =  gKalmanFilter.wTrueTimesDtOverTwo[X_AXIS];
    //gKalmanFilter.F[STATE_Q1][STATE_Q1] =     0;
    gKalmanFilter.F[STATE_Q1][STATE_Q2] =  gKalmanFilter.wTrueTimesDtOverTwo[Z_AXIS];
    gKalmanFilter.F[STATE_Q1][STATE_Q3] = -gKalmanFilter.wTrueTimesDtOverTwo[Y_AXIS];

    gKalmanFilter.F[STATE_Q2][STATE_Q0] =  gKalmanFilter.wTrueTimesDtOverTwo[Y_AXIS];
    gKalmanFilter.F[STATE_Q2][STATE_Q1] = -gKalmanFilter.wTrueTimesDtOverTwo[Z_AXIS];
    //gKalmanFilter.F[STATE_Q2][STATE_Q2] =     0;
    gKalmanFilter.F[STATE_Q2][STATE_Q3] =  gKalmanFilter.wTrueTimesDtOverTwo[X_AXIS];

    gKalmanFilter.F[STATE_Q3][STATE_Q0] =  gKalmanFilter.wTrueTimesDtOverTwo[Z_AXIS];
    gKalmanFilter.F[STATE_Q3][STATE_Q1] =  gKalmanFilter.wTrueTimesDtOverTwo[Y_AXIS];
    gKalmanFilter.F[STATE_Q3][STATE_Q2] = -gKalmanFilter.wTrueTimesDtOverTwo[X_AXIS];
    //gKalmanFilter.F[STATE_Q3,STATE.Q3] =     0;

    // Columns corresponding to the rate-bias state (-0.5*Xi*DT)
    q0DtOver2 = gKalmanFilter.quaternion[Q0] * gAlgorithm.dtOverTwo;
    q1DtOver2 = gKalmanFilter.quaternion[Q1] * gAlgorithm.dtOverTwo;
    q2DtOver2 = gKalmanFilter.quaternion[Q2] * gAlgorithm.dtOverTwo;
    q3DtOver2 = gKalmanFilter.quaternion[Q3] * gAlgorithm.dtOverTwo;

    // ( DT/2 )*QStar
    gKalmanFilter.F[STATE_Q0][STATE_WBX] =  q1DtOver2;
    gKalmanFilter.F[STATE_Q0][STATE_WBY] =  q2DtOver2;
    gKalmanFilter.F[STATE_Q0][STATE_WBZ] =  q3DtOver2;

    gKalmanFilter.F[STATE_Q1][STATE_WBX] = -q0DtOver2;
    gKalmanFilter.F[STATE_Q1][STATE_WBY] =  q3DtOver2;
    gKalmanFilter.F[STATE_Q1][STATE_WBZ] = -q2DtOver2;

    gKalmanFilter.F[STATE_Q2][STATE_WBX] = -q3DtOver2;
    gKalmanFilter.F[STATE_Q2][STATE_WBY] = -q0DtOver2;
    gKalmanFilter.F[STATE_Q2][STATE_WBZ] =  q1DtOver2;

    gKalmanFilter.F[STATE_Q3][STATE_WBX] =  q2DtOver2;
    gKalmanFilter.F[STATE_Q3][STATE_WBY] = -q1DtOver2;
    gKalmanFilter.F[STATE_Q3][STATE_WBZ] = -q0DtOver2;

    // ---------- Rows corresponding to RATE-BIAS ----------
    // All zeros

    // ---------- Rows corresponding to ACCELERATION-BIAS ----------
    // All zeros
}


// UpdateProcessCovarMatrix.m
static void _UpdateProcessCovariance(void)
{
    // Variables used to initially populate the Q-matrix
    real arw, biSq[3];
    real sigDriftDot;

    // Variables used to populate the Q-matrix each time-step
    static real multiplier;
    real v[4];

    // Only need to generate Q-Bias values once
    static int initQ = TRUE;
    if (initQ) {
        initQ = FALSE;

#ifdef INS_OFFLINE
        // This value is set based on the version string specified in the 
        //   simulation configuration file, ekfSim.cfg
		uint8_t sysRange = gSimulation.sysRange;
        uint8_t rsType   = gSimulation.rsType;
#else
        // This value is set based on the version string loaded into the unit
        //   via the system configuration load
        uint8_t sysRange = UcbGetSysRange(); // from system config
#ifdef GYRO_BMI160
        uint8_t rsType = BMI_RS;
#else
        uint8_t rsType = MAXIM_RS;
#endif
#endif

        // Set the matrix, Q, based on whether the system is a -200 or -400
        //   Q-values are based on the rate-sensor's ARW (noise) and BI values
        //   passed through the process model
        switch (sysRange) {
            case _200_DPS_RANGE: // same as default
                // Bias-stability value for the rate-sensors
                if (rsType == BMI_RS) {
                    // BMI: 1.508e-3 [deg/sec] = 2.63e-5 [rad/sec]
                    biSq[0] = (real)(6.93e-10);  // (2.63e-5)^2
                    biSq[1] = biSq[0];
                    biSq[2] = biSq[0];
                } else {
                    // Maxim x/y: 1.85e-3 [deg/sec] = 3.24E-05 [rad/sec]
                    // Maxim z:   7.25e-4 [deg/sec] = 1.27E-05 [rad/sec]
                    biSq[0] = (real)(1.05e-9);  // (3.25e-5)^2
                    biSq[1] = biSq[0];
                    biSq[2] = (real)(1.60e-10);  // (1.27e-5)^2
                }
                break;

            // -400 values
            case _400_DPS_RANGE:
                // bi in [rad]
                if (rsType == BMI_RS) {
                    // BMI: 1.27e-3 [deg/sec] = 2.21e-5 [rad/sec]
                    biSq[0] = (real)(4.91e-10);  // (2.21e-5)^2
                    biSq[1] = biSq[0];
                    biSq[2] = biSq[0];
                } else {
                    // Maxim x/y: 2.16e-3 [deg/sec] = 3.24E-05 [rad/sec]
                    // Maxim z:   1.07e-3 [deg/sec] = 1.86E-05 [rad/sec]
                    biSq[0] = (real)(1.42e-09);  // (3.24e-5)^2
                    biSq[1] = biSq[0];
                    biSq[2] = (real)(3.48e-10);  // (1.86e-5)^2
                }
                break;
        }

        // ARW is not affected by setting, 200/400 (or it seems).  Value is in
        //   units of [rad/rt-sec]
        if (rsType == BMI_RS) {
            // BMI: 4.39e-3 [deg/rt-sec] = 7.66e-5 [rad/rt-sec]
            arw = (real)7.66e-5;
        } else {
            // Maxim: 5.63e-3 [deg/rt-sec] = 9.82E-05 [rad/rt-sec]
            arw = (real)9.82e-5;
        }

        // Rate-bias terms (computed once as it does not change with attitude). 
        //   sigDriftDot = (2*pi/ln(2)) * BI^2 / ARW
        //   2*pi/ln(2) = 9.064720283654388
        sigDriftDot = (real)9.064720283654388 / arw;

        // Rate-bias terms (Q is ultimately the squared value, which is done in the second line of the assignment)
        gKalmanFilter.Q[STATE_WBX][STATE_WBX] = sigDriftDot * biSq[0] * gAlgorithm.dt;
        gKalmanFilter.Q[STATE_WBX][STATE_WBX] = gKalmanFilter.Q[STATE_WBX][STATE_WBX] * gKalmanFilter.Q[STATE_WBX][STATE_WBX];

        gKalmanFilter.Q[STATE_WBY][STATE_WBY] = gKalmanFilter.Q[STATE_WBX][STATE_WBX];

        gKalmanFilter.Q[STATE_WBZ][STATE_WBZ] = sigDriftDot * biSq[2] * gAlgorithm.dt;
        gKalmanFilter.Q[STATE_WBZ][STATE_WBZ] = gKalmanFilter.Q[STATE_WBZ][STATE_WBZ] * gKalmanFilter.Q[STATE_WBZ][STATE_WBZ];

        // FIXME -- tighten up Q to prevent bias from changing quickly
        //gKalmanFilter.Q[STATE_WBX][STATE_WBX] = 0.01 * gKalmanFilter.Q[STATE_WBX][STATE_WBX];
        //gKalmanFilter.Q[STATE_WBY][STATE_WBY] = 0.01 * gKalmanFilter.Q[STATE_WBY][STATE_WBY];
        //gKalmanFilter.Q[STATE_WBZ][STATE_WBZ] = 0.01 * gKalmanFilter.Q[STATE_WBZ][STATE_WBZ];
        
        // May want to reduce Q to 'filter' the rate-bias estimate (need to test
        //   in a simulation before deciding upon the value)
        //gKalmanFilter.Q[STATE_WBX][STATE_WBX] = 0.25 * gKalmanFilter.Q[STATE_WBX][STATE_WBX];
        //gKalmanFilter.Q[STATE_WBY][STATE_WBY] = 0.25 * gKalmanFilter.Q[STATE_WBY][STATE_WBY];
        //gKalmanFilter.Q[STATE_WBZ][STATE_WBZ] = 0.25 * gKalmanFilter.Q[STATE_WBZ][STATE_WBZ];

        // Precalculate the multiplier applied to the Q terms associated with
        //   attitude.
        multiplier = (real)(-0.5) * gAlgorithm.sqrtDt * arw;
    }

    // Update the elements of the process covariance matrix, Q, that change
    //   with each time-step (the elements that correspond to the quaternion-
    //   block of the Q-matrix).  The rest of the elements in the matrix are set
    //   during the transition into and between EKF states (high-gain, low-gain,
    //   etc) or above (upon first entry into this function).

    // 
    v[0] = -gKalmanFilter.quaternion[Q1] - gKalmanFilter.quaternion[Q2] - gKalmanFilter.quaternion[Q3];
    v[1] =  gKalmanFilter.quaternion[Q0] - gKalmanFilter.quaternion[Q3] + gKalmanFilter.quaternion[Q2];
    v[2] =  gKalmanFilter.quaternion[Q3] - gKalmanFilter.quaternion[Q0] - gKalmanFilter.quaternion[Q1];
    v[3] = -gKalmanFilter.quaternion[Q2] + gKalmanFilter.quaternion[Q1] + gKalmanFilter.quaternion[Q0];

    // v = -0.5 * ( rateSensorParams.arw * DEG_TO_RAD ) * sqrt(algo.dt_sec) * Xi * ones(3,1);
    v[0] = multiplier * v[0];
    v[1] = multiplier * v[1];
    v[2] = multiplier * v[2];
    v[3] = multiplier * v[3];

    // Note: the block of values is symmetric: Q = v * v'
    gKalmanFilter.Q[STATE_Q0][STATE_Q0] = v[0] * v[0];
    gKalmanFilter.Q[STATE_Q0][STATE_Q1] = v[0] * v[1];
    gKalmanFilter.Q[STATE_Q0][STATE_Q2] = v[0] * v[2];
    gKalmanFilter.Q[STATE_Q0][STATE_Q3] = v[0] * v[3];

    gKalmanFilter.Q[STATE_Q1][STATE_Q0] = gKalmanFilter.Q[STATE_Q0][STATE_Q1];
    gKalmanFilter.Q[STATE_Q1][STATE_Q1] = v[1] * v[1];
    gKalmanFilter.Q[STATE_Q1][STATE_Q2] = v[1] * v[2];
    gKalmanFilter.Q[STATE_Q1][STATE_Q3] = v[1] * v[3];

    gKalmanFilter.Q[STATE_Q2][STATE_Q0] = gKalmanFilter.Q[STATE_Q0][STATE_Q2];
    gKalmanFilter.Q[STATE_Q2][STATE_Q1] = gKalmanFilter.Q[STATE_Q1][STATE_Q2];
    gKalmanFilter.Q[STATE_Q2][STATE_Q2] = v[2] * v[2];
    gKalmanFilter.Q[STATE_Q2][STATE_Q3] = v[2] * v[3];

    gKalmanFilter.Q[STATE_Q3][STATE_Q0] = gKalmanFilter.Q[STATE_Q0][STATE_Q3];
    gKalmanFilter.Q[STATE_Q3][STATE_Q1] = gKalmanFilter.Q[STATE_Q1][STATE_Q3];
    gKalmanFilter.Q[STATE_Q3][STATE_Q2] = gKalmanFilter.Q[STATE_Q2][STATE_Q3];
    gKalmanFilter.Q[STATE_Q3][STATE_Q3] = v[3] * v[3];
}


//
// GenerateProcessCovarMatrix.m
void GenerateProcessCovariance(void)
{
    real sigRate; //, sigWalk;
    static real sigWalk;

    // Initialize the Process Covariance (Q) matrix with values that do not change
    memset(gKalmanFilter.Q, 0, sizeof(gKalmanFilter.Q));

    // Load in the CONSTANT ELEMENTS
    sigRate = (real)SENSOR_NOISE_RATE_STD_DEV;
    sigWalk = (real)0.004 * sigRate;

    real dtSigRateSq = (real)(gAlgorithm.dt * sigRate);
    dtSigRateSq = dtSigRateSq * dtSigRateSq;
    //oneHalfDtSigRateSq = (real)0.25 * dtSigRateSq;

    gKalmanFilter.Q[STATE_WBX][STATE_WBX] = (real)(gAlgorithm.dt * sigWalk * sigWalk );   // 7e-13
    gKalmanFilter.Q[STATE_WBY][STATE_WBY] = gKalmanFilter.Q[STATE_WBX][STATE_WBX];
    gKalmanFilter.Q[STATE_WBZ][STATE_WBZ] = gKalmanFilter.Q[STATE_WBX][STATE_WBX];

    //% THE FOLLOWING COVARIANCE VALUES AREN'T CORRECT, JUST SELECTED SO THE
    //% PROGRAM COULD RUN

    // Acceleration based values
    real dtSigAccelSq = (real)(gAlgorithm.dt * SENSOR_NOISE_ACCEL_STD_DEV);
    dtSigAccelSq = dtSigAccelSq * dtSigAccelSq;

    // Position
    gKalmanFilter.Q[STATE_RX][STATE_RX] = gAlgorithm.dtSquared * dtSigAccelSq;
    gKalmanFilter.Q[STATE_RY][STATE_RY] = gAlgorithm.dtSquared * dtSigAccelSq;
    gKalmanFilter.Q[STATE_RZ][STATE_RZ] = gAlgorithm.dtSquared * dtSigAccelSq;

    //% Velocity
    gKalmanFilter.Q[STATE_VX][STATE_VX] = dtSigAccelSq;
    gKalmanFilter.Q[STATE_VY][STATE_VY] = dtSigAccelSq;
    gKalmanFilter.Q[STATE_VZ][STATE_VZ] = dtSigAccelSq;

    // Acceleration - bias
    gKalmanFilter.Q[STATE_ABX][STATE_ABX] = dtSigAccelSq; //%1e-8 %sigmaAccelBiasSq;
    gKalmanFilter.Q[STATE_ABY][STATE_ABY] = dtSigAccelSq; //%sigmaAccelBiasSq;
    gKalmanFilter.Q[STATE_ABZ][STATE_ABZ] = dtSigAccelSq; //%sigmaAccelBiasSq;
}


/** ****************************************************************************
 * @name: firstOrderLowPass_float  implements a low pass yaw axis filter
 * @brief floating point version
 * TRACE:
 * @param [out] - output pointer to the filtered value
 * @param [in] - input pointer to a new raw value
 * @retval N/A
 * @details  This is a replacement for 'lowPass2Pole' found in the 440 code.
 *           Note, the implementation in the 440 SW is not a second-order filter
 *           but is an implementation of a first-order filter.
 ******************************************************************************/
void _FirstOrderLowPass( real *output,   // <-- INITIALLY THIS IS OUTPUT PAST (FILTERED VALUED IS RETURNED HERE)
                         real input )   // <-- CURRENT VALUE OF SIGNAL TO BE FILTERED
{
    static real inputPast;

    // To match the Matlab model (0.25 Hz LPF)
    if( gAlgorithm.callingFreq == ODR_100_HZ ) {
        *output = (real)(0.984414127416097) * (*output) + 
                  (real)(0.007792936291952) * (input + inputPast);
    } else {
        *output = (real)(0.992176700177507) * (*output) + 
                  (real)(0.003911649911247) * (input + inputPast);
    }

    inputPast = input;
}


// Set the accelerometer filter coefficients, which are used to filter the 
//   accelerometer readings prior to determining the setting of the linear-
//   acceleration switch and computing the roll and pitch from accelerometer
//   readings.
static void _PopulateFilterCoefficients(void)
{
#ifdef INS_OFFLINE
    uint8_t accelLowPassFiltType = gSimulation.accelLowPassFilterType;
#else
    uint8_t accelLowPassFiltType = TEN_HZ_LPF;
#endif

    switch( accelLowPassFiltType ) {
        case NO_LPF:
            b_AccelFilt[0] = (real)(1.0);
            b_AccelFilt[1] = (real)(0.0);
            b_AccelFilt[2] = (real)(0.0);
            b_AccelFilt[3] = (real)(0.0);

            a_AccelFilt[0] = (real)(0.0);
            a_AccelFilt[1] = (real)(0.0);
            a_AccelFilt[2] = (real)(0.0);
            a_AccelFilt[3] = (real)(0.0);
            break;
        case FIVE_HZ_LPF:
            if( gAlgorithm.callingFreq == ODR_100_HZ ) {
                b_AccelFilt[0] = (real)( 0.002898194633721);
                b_AccelFilt[1] = (real)( 0.008694583901164);
                b_AccelFilt[2] = (real)( 0.008694583901164);
                b_AccelFilt[3] = (real)( 0.002898194633721);

                a_AccelFilt[0] = (real)( 1.000000000000000);
                a_AccelFilt[1] = (real)(-2.374094743709352);
                a_AccelFilt[2] = (real)( 1.929355669091215);
                a_AccelFilt[3] = (real)(-0.532075368312092);
            } else {
                b_AccelFilt[0] = (real)( 0.000416546139076);
                b_AccelFilt[1] = (real)( 0.001249638417227);
                b_AccelFilt[2] = (real)( 0.001249638417227);
                b_AccelFilt[3] = (real)( 0.000416546139076);

                a_AccelFilt[0] = (real)( 1.000000000000000);
                a_AccelFilt[1] = (real)(-2.686157396548143);
                a_AccelFilt[2] = (real)( 2.419655110966473);
                a_AccelFilt[3] = (real)(-0.730165345305723);
            }
            break;
        case TWENTY_HZ_LPF:
            if (gAlgorithm.callingFreq == ODR_100_HZ) {
                b_AccelFilt[0] = (real)( 0.098531160923927);
                b_AccelFilt[1] = (real)( 0.295593482771781);
                b_AccelFilt[2] = (real)( 0.295593482771781);
                b_AccelFilt[3] = (real)( 0.098531160923927);

                a_AccelFilt[0] = (real)( 1.000000000000000);
                a_AccelFilt[1] = (real)(-0.577240524806303);
                a_AccelFilt[2] = (real)( 0.421787048689562);
                a_AccelFilt[3] = (real)(-0.056297236491843);
            } else {
                b_AccelFilt[0] = (real)( 0.018098933007514);
                b_AccelFilt[1] = (real)( 0.054296799022543);
                b_AccelFilt[2] = (real)( 0.054296799022543);
                b_AccelFilt[3] = (real)( 0.018098933007514);
                
                a_AccelFilt[0] = (real)( 1.000000000000000);
                a_AccelFilt[1] = (real)(-1.760041880343169);
                a_AccelFilt[2] = (real)( 1.182893262037831);
                a_AccelFilt[3] = (real)(-0.278059917634546);
            }
            break;
        case TWENTY_FIVE_HZ_LPF:
            if (gAlgorithm.callingFreq == ODR_100_HZ) {
                b_AccelFilt[0] = (real)( 0.166666666666667);
                b_AccelFilt[1] = (real)( 0.500000000000000);
                b_AccelFilt[2] = (real)( 0.500000000000000);
                b_AccelFilt[3] = (real)( 0.166666666666667);

                a_AccelFilt[0] = (real)( 1.000000000000000);
                a_AccelFilt[1] = (real)(-0.000000000000000);
                a_AccelFilt[2] = (real)( 0.333333333333333);
                a_AccelFilt[3] = (real)(-0.000000000000000);
            } else {
                b_AccelFilt[0] = (real)( 0.031689343849711);
                b_AccelFilt[1] = (real)( 0.095068031549133);
                b_AccelFilt[2] = (real)( 0.095068031549133);
                b_AccelFilt[3] = (real)( 0.031689343849711);

                a_AccelFilt[0] = (real)( 1.000000000000000);
                a_AccelFilt[1] = (real)(-1.459029062228061);
                a_AccelFilt[2] = (real)( 0.910369000290069);
                a_AccelFilt[3] = (real)(-0.197825187264319);
            }
            break;
        case TEN_HZ_LPF:
        default:
            if( gAlgorithm.callingFreq == ODR_100_HZ ) {
                b_AccelFilt[0] = (real)( 0.0180989330075144);
                b_AccelFilt[1] = (real)( 0.0542967990225433);
                b_AccelFilt[2] = (real)( 0.0542967990225433);
                b_AccelFilt[3] = (real)( 0.0180989330075144);

                a_AccelFilt[0] = (real)( 1.0000000000000000);
                a_AccelFilt[1] = (real)(-1.7600418803431690);
                a_AccelFilt[2] = (real)( 1.1828932620378310);
                a_AccelFilt[3] = (real)(-0.2780599176345460);
            } else {
                b_AccelFilt[0] = (real)( 0.002898194633721);
                b_AccelFilt[1] = (real)( 0.008694583901164);
                b_AccelFilt[2] = (real)( 0.008694583901164);
                b_AccelFilt[3] = (real)( 0.002898194633721);

                a_AccelFilt[0] = (real)( 1.000000000000000);
                a_AccelFilt[1] = (real)(-2.374094743709352);
                a_AccelFilt[2] = (real)( 1.929355669091215);
                a_AccelFilt[3] = (real)(-0.532075368312092);
            }
            break;
    }
}

