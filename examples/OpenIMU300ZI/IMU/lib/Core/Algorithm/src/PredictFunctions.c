/*
 * File:   EKF_PredictionStage.cpp
 * Author: joemotyka
 *
 * Created on May 8, 2016, 12:23 AM
 */


#include <string.h>   // memset
#include <math.h>      // pow

#include "GlobalConstants.h"
#include "platformAPI.h"
#include "Indices.h"       // IND
#include "StateIndices.h"  // STATE_IND

#include "MatrixMath.h"
#include "VectorMath.h"
#include "QuaternionMath.h"  // QuatNormalize
#include "TransformationMath.h"

#include "algorithm.h"
#include "algorithmAPI.h"
#include "SensorNoiseParameters.h"
#include "MotionStatus.h"
#include "EKF_Algorithm.h"
#include "PredictFunctions.h"
#include "AlgorithmLimits.h"
#include "WorldMagneticModel.h"

#ifndef INS_OFFLINE
#ifdef DISPLAY_DIAGNOSTIC_MSG
#include "debug.h"
#endif
#endif

extern ImuStatsStruct gImuStats;

/* F is sparse and has elements in the following locations...
 * There may be some more efficient ways of implementing this as this method
 * still performs multiplication with zero values.  (Ask Andrey)
 */
uint8_t RLE_F[ROWS_IN_F][2] = { {  STATE_RX, STATE_VX  },     // Row  0: cols 0,3
                                {  STATE_RY, STATE_VY  },     // Row  1: cols 1,4
                                {  STATE_RZ, STATE_VZ  },     // Row  2: cols 2,5
                                {  STATE_VX, STATE_ABZ },     // Row  3: cols 3,6:9,13:15
                                {  STATE_VY, STATE_ABZ },     // Row  4: cols 4,6:9,13:15
                                {  STATE_VZ, STATE_ABZ },     // Row  5: cols 5,6:9,13:15
                                {  STATE_Q0, STATE_WBZ },     // Row  6: cols 6:12
                                {  STATE_Q0, STATE_WBZ },     // Row  7: cols 6:12
                                {  STATE_Q0, STATE_WBZ },     // Row  8: cols 6:12
                                {  STATE_Q0, STATE_WBZ },     // Row  9: cols 6:12
                                { STATE_WBX, STATE_WBX },     // Row 10: cols 10
                                { STATE_WBY, STATE_WBY },     // Row 11: cols 11
                                { STATE_WBZ, STATE_WBZ },     // Row 12: cols 12
                                { STATE_ABX, STATE_ABX },     // Row 13: cols 13
                                { STATE_ABY, STATE_ABY },     // Row 14: cols 14
                                { STATE_ABZ, STATE_ABZ } };   // Row 15: cols 15

// Q is sparse and has elements in the following locations...
uint8_t RLE_Q[ROWS_IN_F][2] = { {  STATE_RX, STATE_RX  },
                                {  STATE_RY, STATE_RY  },
                                {  STATE_RZ, STATE_RZ  },
                                {  STATE_VX, STATE_VX  },
                                {  STATE_VY, STATE_VY  },
                                {  STATE_VZ, STATE_VZ  },
                                {  STATE_Q0, STATE_Q3  },
                                {  STATE_Q0, STATE_Q3  },
                                {  STATE_Q0, STATE_Q3  },
                                {  STATE_Q0, STATE_Q3  },
                                { STATE_WBX, STATE_WBX },
                                { STATE_WBY, STATE_WBY },
                                { STATE_WBZ, STATE_WBZ },
                                { STATE_ABX, STATE_ABX },
                                { STATE_ABY, STATE_ABY },
                                { STATE_ABZ, STATE_ABZ } };

// Local functions
static void _PredictStateEstimate(void);
static void _PredictCovarianceEstimate(void);

static void _UpdateProcessJacobian(void);
static void _UpdateProcessCovariance(void);

// todo tm20160603 - use filters from filter.h, or move this filter there  (Ask Andrey)
void _FirstOrderLowPass(real *output, real input);

/* 16 States: [ STATE_RX,  STATE_RY,  STATE_RZ, ...
 *              STATE_VX,  STATE_VY,  STATE_VZ, ...
 *              STATE_Q0,  STATE_Q1,  STATE_Q2,  STATE_Q3, ...
 *              STATE_WBX, STATE_WBY, STATE_WBZ, ...
 *              STATE_ABX, STATE_ABY, STATE_ABZ ]
 */
//=============================================================================
//EKF_PredictionStage.m
void EKF_PredictionStage(real *filteredAccel)
{
    real magFieldVector[3];

    // Propagate the state (22 usec) and covariance (1.82 msec) estimates
    _PredictStateEstimate();        // x(k+1) = x(k) + f(x(k), u(k))
    _PredictCovarianceEstimate();   // P = F*P*FTrans + Q

    // Extract the predicted Euler angles from the predicted quaternion
    QuaternionToEulerAngles( gKalmanFilter.eulerAngles,
                             gKalmanFilter.quaternion );

    /* Filter the yaw-rate at 200 Hz for the TURN-SWITCH (used in the
     * update stage only -- since that is a ten-hertz routine).  The way this
     * is coded, the filter function can only be used for filtering yaw-rate
     * data as the previous input state is saved as a static in the function.
     */
    _FirstOrderLowPass( &gAlgorithm.filteredYawRate,
                        gKalmanFilter.correctedRate_B[Z_AXIS] );

    /* Extract the magnetometer readings (set to zero if the magnetometer is not
     * present or unused).
     */
    if(magUsedInAlgorithm())
    {
        magFieldVector[X_AXIS] = (real)gEKFInput.magField_B[X_AXIS];
        magFieldVector[Y_AXIS] = (real)gEKFInput.magField_B[Y_AXIS];
        magFieldVector[Z_AXIS] = (real)gEKFInput.magField_B[Z_AXIS];
    }
    else
    {
        magFieldVector[X_AXIS] = magFieldVector[Y_AXIS] = magFieldVector[Z_AXIS] = (real)0.0;
    }

    /* Compute the measured Euler angles from gravity and magnetic field data
     * ( phiMeas, thetaMeas, psiMeas ) = f( g_B, mMeas_B ).  Adjust for declination.
     */
    // Compute the unit gravity vector (-accel) in the body frame 
    real unitGravityVector[3] = {0.0f};
    UnitGravity(filteredAccel, unitGravityVector);
    // Compute roll and pitch from the unit gravity vector.
    UnitGravityToEulerAngles(unitGravityVector, gKalmanFilter.measuredEulerAngles);
    /* Compute measured yaw.
     * If mag is not in use, yaw is the predicted yaw in the Kalman filter
     * If mag is in use, predicted roll and pitch are used to project mag and compute yaw in LG (chage to >=LG?),
     * measured pitch and roll (indeed the unit gravity vector from measured accel) are used to project
     * mag and compute yaw in other cases.
    */
    if ( magUsedInAlgorithm() )
    {
        // Transform the magnetic field vector from the body-frame to the plane normal to the gravity vector
        if ( gAlgorithm.state == LOW_GAIN_AHRS )
        {
            // Using predicted pitch and roll to project the mag measurement
            gKalmanFilter.measuredEulerAngles[YAW] =
                RollPitchAndMagToYaw( gKalmanFilter.eulerAngles[ROLL],
                                      gKalmanFilter.eulerAngles[PITCH],
                                      magFieldVector );
        }
        else
        {
            // Using accel measurement to project the mag measurement
            gKalmanFilter.measuredEulerAngles[YAW] =
                UnitGravityAndMagToYaw( unitGravityVector,
                                        magFieldVector );
        }
    }
    else
    {
        // For VG, set the measured heading to the predicted heading (this
        //   forces the error to zero)
        gKalmanFilter.measuredEulerAngles[YAW] = gKalmanFilter.eulerAngles[YAW];
    }


    // Adjust for declination if the GPS signal is good
    if( gAlgorithm.applyDeclFlag ) 
    {
        gKalmanFilter.measuredEulerAngles[YAW] = gKalmanFilter.measuredEulerAngles[YAW] +
                                                 gWorldMagModel.decl_rad;
    }
}


/* Predict the EKF states at 100 Hz based on readings from the:
 *  - accelerometer
 *  - angular-rate sensors
 */
static void _PredictStateEstimate(void)
{
    real aCorr_N[3];
    real deltaQuaternion[4];

    if( gAlgorithm.state > LOW_GAIN_AHRS ) 
    {
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
        // gEKFInput.accel_B in g's, convert to m/s^2 for integration
        gKalmanFilter.correctedAccel_B[X_AXIS] = gEKFInput.accel_B[X_AXIS] -
                                                 gKalmanFilter.accelBias_B[X_AXIS];
        gKalmanFilter.correctedAccel_B[Y_AXIS] = gEKFInput.accel_B[Y_AXIS] -
                                                 gKalmanFilter.accelBias_B[Y_AXIS];
        gKalmanFilter.correctedAccel_B[Z_AXIS] = gEKFInput.accel_B[Z_AXIS] -
                                                 gKalmanFilter.accelBias_B[Z_AXIS];

        /* Transform the corrected acceleration vector from the body to the NED-frame and remove gravity
         * a_N = R_BinN * a_B
         */
        aCorr_N[X_AXIS] =
                gKalmanFilter.R_BinN[X_AXIS][X_AXIS] * gKalmanFilter.correctedAccel_B[X_AXIS] +
                gKalmanFilter.R_BinN[X_AXIS][Y_AXIS] * gKalmanFilter.correctedAccel_B[Y_AXIS] +
                gKalmanFilter.R_BinN[X_AXIS][Z_AXIS] * gKalmanFilter.correctedAccel_B[Z_AXIS];
        aCorr_N[Y_AXIS] =
                gKalmanFilter.R_BinN[Y_AXIS][X_AXIS] * gKalmanFilter.correctedAccel_B[X_AXIS] +
                gKalmanFilter.R_BinN[Y_AXIS][Y_AXIS] * gKalmanFilter.correctedAccel_B[Y_AXIS] +
                gKalmanFilter.R_BinN[Y_AXIS][Z_AXIS] * gKalmanFilter.correctedAccel_B[Z_AXIS];
        aCorr_N[Z_AXIS] =
                gKalmanFilter.R_BinN[Z_AXIS][X_AXIS] * gKalmanFilter.correctedAccel_B[X_AXIS] +
                gKalmanFilter.R_BinN[Z_AXIS][Y_AXIS] * gKalmanFilter.correctedAccel_B[Y_AXIS] +
                gKalmanFilter.R_BinN[Z_AXIS][Z_AXIS] * gKalmanFilter.correctedAccel_B[Z_AXIS] +
                (real)GRAVITY;

        /* Determine the acceleration of the system by removing the gravity vector
         * v_N(k+1) = v_N(k) + dV = v_N(k) + aMotion_N*DT = v_N(k) + ( a_N - g_N )*DT
         */
        gKalmanFilter.Velocity_N[X_AXIS] = gKalmanFilter.Velocity_N[X_AXIS] + aCorr_N[X_AXIS] * gAlgorithm.dt;
        gKalmanFilter.Velocity_N[Y_AXIS] = gKalmanFilter.Velocity_N[Y_AXIS] + aCorr_N[Y_AXIS] * gAlgorithm.dt;
        gKalmanFilter.Velocity_N[Z_AXIS] = gKalmanFilter.Velocity_N[Z_AXIS] + aCorr_N[Z_AXIS] * gAlgorithm.dt;
        
        // Calculate linear acceleration in the body frame.
        gKalmanFilter.linearAccel_B[X_AXIS] += (gKalmanFilter.correctedAccel_B[X_AXIS] +
                                              gKalmanFilter.R_BinN[Z_AXIS][X_AXIS] * (real)GRAVITY)*gAlgorithm.dt;
        gKalmanFilter.linearAccel_B[Y_AXIS] = gKalmanFilter.correctedAccel_B[Y_AXIS] +
                                              gKalmanFilter.R_BinN[Z_AXIS][Y_AXIS] * (real)GRAVITY;
        gKalmanFilter.linearAccel_B[Z_AXIS] = gKalmanFilter.correctedAccel_B[Z_AXIS] +
                                              gKalmanFilter.R_BinN[Z_AXIS][Z_AXIS] * (real)GRAVITY;
    }
    else
    {
        // GPS not valid yet, do not propagate the position or velocity
        gKalmanFilter.Position_N[X_AXIS] = (real)0.0;
        gKalmanFilter.Position_N[Y_AXIS] = (real)0.0;
        gKalmanFilter.Position_N[Z_AXIS] = (real)0.0;

        gKalmanFilter.Velocity_N[X_AXIS] = (real)0.0;
        gKalmanFilter.Velocity_N[Y_AXIS] = (real)0.0;
        gKalmanFilter.Velocity_N[Z_AXIS] = (real)0.0;

        // what should this be???
        gKalmanFilter.correctedAccel_B[XACCEL] = gEKFInput.accel_B[X_AXIS];
        gKalmanFilter.correctedAccel_B[YACCEL] = gEKFInput.accel_B[Y_AXIS];
        gKalmanFilter.correctedAccel_B[ZACCEL] = gEKFInput.accel_B[Z_AXIS];
    }

    // ================= Attitude quaternion =================
    // Find the 'true' angular rate (wTrue_B = wCorr_B = wMeas_B - wBias_B)
    gKalmanFilter.correctedRate_B[X_AXIS] = gEKFInput.angRate_B[X_AXIS] -
                                            gKalmanFilter.rateBias_B[X_AXIS];
    gKalmanFilter.correctedRate_B[Y_AXIS] = gEKFInput.angRate_B[Y_AXIS] -
                                            gKalmanFilter.rateBias_B[Y_AXIS];
    gKalmanFilter.correctedRate_B[Z_AXIS] = gEKFInput.angRate_B[Z_AXIS] -
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

    /* Compute the F and Q matrices used in the prediction stage (only certain
     * elements in the process-covariance, Q, change with each time-step)
     */
    _UpdateProcessJacobian();     // gKF.F  (16x16)
    _UpdateProcessCovariance();   // gKF.Q  (16x16)

    // Update P from the P, F, and Q matrices: P = FxPxFTranspose + Q
    // 1) PxFTranspose is computed first
    memset(PxFTranspose, 0, sizeof(PxFTranspose));
    for (rowNum = 0; rowNum < ROWS_IN_P; rowNum++) 
    {
        for (colNum = 0; colNum < ROWS_IN_F; colNum++) 
        {
            for (multIndex = RLE_F[colNum][0]; multIndex <= RLE_F[colNum][1]; multIndex++) 
            {
                PxFTranspose[rowNum][colNum] = PxFTranspose[rowNum][colNum] +
                    gKalmanFilter.P[rowNum][multIndex] * gKalmanFilter.F[colNum][multIndex];
            }
        }
    }

#if 0
    //   2) Use gKalmanFilter.P as a temporary variable to hold FxPxFTranspose
    //      to reduce the number of "large" variables on the heap
    for (rowNum = 0; rowNum < 16; rowNum++) 
    {
        for (colNum = 0; colNum < 16; colNum++) 
        {
            gKalmanFilter.P[rowNum][colNum] = 0.0;
            for (multIndex = RLE_F[rowNum][0]; multIndex <= RLE_F[rowNum][1]; multIndex++) 
            {
                gKalmanFilter.P[rowNum][colNum] = gKalmanFilter.P[rowNum][colNum] +
                    gKalmanFilter.F[rowNum][multIndex] * PxFTranspose[multIndex][colNum];
            }
        }
    }

    // P is a fully populated matrix (nominally) so all the elements of the matrix have to be
    //   considered when working with it.
    LimitValuesAndForceMatrixSymmetry_noAvg(&gKalmanFilter.P[0][0], (real)LIMIT_P, ROWS_IN_P, COLS_IN_P);
#else
    /* 2) Use gKalmanFilter.P as a temporary variable to hold FxPxFTranspose
     * to reduce the number of "large" variables on the heap.
     * This matrix is symmetric so only need to multiply one half and reflect the values
     * across the diagonal
     */
    memset(gKalmanFilter.P, 0, sizeof(gKalmanFilter.P));
    for (rowNum = 0; rowNum < 16; rowNum++) 
    {
        for (colNum = rowNum; colNum < 16; colNum++) 
        {
            //gKalmanFilter.P[rowNum][colNum] = 0.0;
            for (multIndex = RLE_F[rowNum][0]; multIndex <= RLE_F[rowNum][1]; multIndex++) 
            {
                gKalmanFilter.P[rowNum][colNum] = gKalmanFilter.P[rowNum][colNum] +
                    gKalmanFilter.F[rowNum][multIndex] * PxFTranspose[multIndex][colNum];
            }
            gKalmanFilter.P[colNum][rowNum] = gKalmanFilter.P[rowNum][colNum];
             //Limit values in P
            if(gKalmanFilter.P[rowNum][colNum] > (real)LIMIT_P) 
            {
                gKalmanFilter.P[rowNum][colNum] = (real)LIMIT_P;
            }
            else if(gKalmanFilter.P[rowNum][colNum] < -(real)LIMIT_P)
            {
                gKalmanFilter.P[rowNum][colNum] = -(real)LIMIT_P;
            }
        }
    }
#endif
    
    /* 3) Finally, add Q to FxPxFTranspose (P) to get the final value for
     * gKalmanFilter.P (only the quaternion elements of Q have nonzero off-
     * diagonal terms)
     */
    gKalmanFilter.P[STATE_RX][STATE_RX] = gKalmanFilter.P[STATE_RX][STATE_RX] + gKalmanFilter.Q[STATE_RX];
    gKalmanFilter.P[STATE_RY][STATE_RY] = gKalmanFilter.P[STATE_RY][STATE_RY] + gKalmanFilter.Q[STATE_RY];
    gKalmanFilter.P[STATE_RZ][STATE_RZ] = gKalmanFilter.P[STATE_RZ][STATE_RZ] + gKalmanFilter.Q[STATE_RZ];

    gKalmanFilter.P[STATE_VX][STATE_VX] = gKalmanFilter.P[STATE_VX][STATE_VX] + gKalmanFilter.Q[STATE_VX];
    gKalmanFilter.P[STATE_VY][STATE_VY] = gKalmanFilter.P[STATE_VY][STATE_VY] + gKalmanFilter.Q[STATE_VY];
    gKalmanFilter.P[STATE_VZ][STATE_VZ] = gKalmanFilter.P[STATE_VZ][STATE_VZ] + gKalmanFilter.Q[STATE_VZ];

    gKalmanFilter.P[STATE_Q0][STATE_Q0] = gKalmanFilter.P[STATE_Q0][STATE_Q0] + gKalmanFilter.Q[STATE_Q0];
    gKalmanFilter.P[STATE_Q0][STATE_Q1] = gKalmanFilter.P[STATE_Q0][STATE_Q1] + gKalmanFilter.Qq[0];
    gKalmanFilter.P[STATE_Q0][STATE_Q2] = gKalmanFilter.P[STATE_Q0][STATE_Q2] + gKalmanFilter.Qq[1];
    gKalmanFilter.P[STATE_Q0][STATE_Q3] = gKalmanFilter.P[STATE_Q0][STATE_Q3] + gKalmanFilter.Qq[2];

    //gKalmanFilter.P[STATE_Q1][STATE_Q0] = gKalmanFilter.P[STATE_Q1][STATE_Q0] + gKalmanFilter.Q[STATE_Q1][STATE_Q0];
    gKalmanFilter.P[STATE_Q1][STATE_Q0] = gKalmanFilter.P[STATE_Q0][STATE_Q1];
    gKalmanFilter.P[STATE_Q1][STATE_Q1] = gKalmanFilter.P[STATE_Q1][STATE_Q1] + gKalmanFilter.Q[STATE_Q1];
    gKalmanFilter.P[STATE_Q1][STATE_Q2] = gKalmanFilter.P[STATE_Q1][STATE_Q2] + gKalmanFilter.Qq[3];
    gKalmanFilter.P[STATE_Q1][STATE_Q3] = gKalmanFilter.P[STATE_Q1][STATE_Q3] + gKalmanFilter.Qq[4];

    //gKalmanFilter.P[STATE_Q2][STATE_Q0] = gKalmanFilter.P[STATE_Q2][STATE_Q0] + gKalmanFilter.Q[STATE_Q2][STATE_Q0];
    //gKalmanFilter.P[STATE_Q2][STATE_Q1] = gKalmanFilter.P[STATE_Q2][STATE_Q1] + gKalmanFilter.Q[STATE_Q2][STATE_Q1];
    gKalmanFilter.P[STATE_Q2][STATE_Q0] = gKalmanFilter.P[STATE_Q0][STATE_Q2];
    gKalmanFilter.P[STATE_Q2][STATE_Q1] = gKalmanFilter.P[STATE_Q1][STATE_Q2];
    gKalmanFilter.P[STATE_Q2][STATE_Q2] = gKalmanFilter.P[STATE_Q2][STATE_Q2] + gKalmanFilter.Q[STATE_Q2];
    gKalmanFilter.P[STATE_Q2][STATE_Q3] = gKalmanFilter.P[STATE_Q2][STATE_Q3] + gKalmanFilter.Qq[5];

    //gKalmanFilter.P[STATE_Q3][STATE_Q0] = gKalmanFilter.P[STATE_Q3][STATE_Q0] + gKalmanFilter.Q[STATE_Q3][STATE_Q0];
    //gKalmanFilter.P[STATE_Q3][STATE_Q1] = gKalmanFilter.P[STATE_Q3][STATE_Q1] + gKalmanFilter.Q[STATE_Q3][STATE_Q1];
    //gKalmanFilter.P[STATE_Q3][STATE_Q2] = gKalmanFilter.P[STATE_Q3][STATE_Q2] + gKalmanFilter.Q[STATE_Q3][STATE_Q2];
    gKalmanFilter.P[STATE_Q3][STATE_Q0] = gKalmanFilter.P[STATE_Q0][STATE_Q3];
    gKalmanFilter.P[STATE_Q3][STATE_Q1] = gKalmanFilter.P[STATE_Q1][STATE_Q3];
    gKalmanFilter.P[STATE_Q3][STATE_Q2] = gKalmanFilter.P[STATE_Q2][STATE_Q3];
    gKalmanFilter.P[STATE_Q3][STATE_Q3] = gKalmanFilter.P[STATE_Q3][STATE_Q3] + gKalmanFilter.Q[STATE_Q3];

    gKalmanFilter.P[STATE_WBX][STATE_WBX] = gKalmanFilter.P[STATE_WBX][STATE_WBX] + gKalmanFilter.Q[STATE_WBX];
    gKalmanFilter.P[STATE_WBY][STATE_WBY] = gKalmanFilter.P[STATE_WBY][STATE_WBY] + gKalmanFilter.Q[STATE_WBY];
    gKalmanFilter.P[STATE_WBZ][STATE_WBZ] = gKalmanFilter.P[STATE_WBZ][STATE_WBZ] + gKalmanFilter.Q[STATE_WBZ];

    gKalmanFilter.P[STATE_ABX][STATE_ABX] = gKalmanFilter.P[STATE_ABX][STATE_ABX] + gKalmanFilter.Q[STATE_ABX];
    gKalmanFilter.P[STATE_ABY][STATE_ABY] = gKalmanFilter.P[STATE_ABY][STATE_ABY] + gKalmanFilter.Q[STATE_ABY];
    gKalmanFilter.P[STATE_ABZ][STATE_ABZ] = gKalmanFilter.P[STATE_ABZ][STATE_ABZ] + gKalmanFilter.Q[STATE_ABZ];
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
    // N/A (other than diagonal, set below, all other terms changes with attitude)

    // ---------- Rows corresponding to ATTITUDE ----------
    // N/A (other than diagonal, set below, all other terms changes with attitude)

    // ---------- Rows corresponding to RATE-BIAS ----------
    // N/A (no terms changes with attitude)

    // ---------- Rows corresponding to ACCELERATION-BIAS ----------
    // N/A (no terms changes with attitude)

    // ---------- Add to I16 to get final formulation of F ----------
    // Populate the diagonals of F with 1.0
    gKalmanFilter.F[STATE_RX][STATE_RX] = (real)1.0;
    gKalmanFilter.F[STATE_RY][STATE_RY] = (real)1.0;
    gKalmanFilter.F[STATE_RZ][STATE_RZ] = (real)1.0;

    gKalmanFilter.F[STATE_VX][STATE_VX] = (real)1.0;
    gKalmanFilter.F[STATE_VY][STATE_VY] = (real)1.0;
    gKalmanFilter.F[STATE_VZ][STATE_VZ] = (real)1.0;

    gKalmanFilter.F[STATE_Q0][STATE_Q0] = (real)1.0;
    gKalmanFilter.F[STATE_Q1][STATE_Q1] = (real)1.0;
    gKalmanFilter.F[STATE_Q2][STATE_Q2] = (real)1.0;
    gKalmanFilter.F[STATE_Q3][STATE_Q3] = (real)1.0;

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

#if 1
    // mod, DXG
    gKalmanFilter.F[STATE_VX][STATE_Q0] = (real)2.0 * (  q0aXdT - q3aYdT + q2aZdT -
                                                        gKalmanFilter.R_BinN[0][0]*q0aXdT - 
                                                        gKalmanFilter.R_BinN[0][1]*q0aYdT - 
                                                        gKalmanFilter.R_BinN[0][2]*q0aZdT);
    gKalmanFilter.F[STATE_VX][STATE_Q1] = (real)2.0 * (  q1aXdT + q2aYdT + q3aZdT -
                                                        gKalmanFilter.R_BinN[0][0]*q1aXdT - 
                                                        gKalmanFilter.R_BinN[0][1]*q1aYdT - 
                                                        gKalmanFilter.R_BinN[0][2]*q1aZdT);
    gKalmanFilter.F[STATE_VX][STATE_Q2] = (real)2.0 * ( -q2aXdT + q1aYdT + q0aZdT -
                                                        gKalmanFilter.R_BinN[0][0]*q2aXdT - 
                                                        gKalmanFilter.R_BinN[0][1]*q2aYdT - 
                                                        gKalmanFilter.R_BinN[0][2]*q2aZdT);
    gKalmanFilter.F[STATE_VX][STATE_Q3] = (real)2.0 * ( -q3aXdT - q0aYdT + q1aZdT -
                                                        gKalmanFilter.R_BinN[0][0]*q3aXdT - 
                                                        gKalmanFilter.R_BinN[0][1]*q3aYdT - 
                                                        gKalmanFilter.R_BinN[0][2]*q3aZdT);

    gKalmanFilter.F[STATE_VY][STATE_Q0] = (real)2.0 * (  q3aXdT + q0aYdT - q1aZdT -
                                                        gKalmanFilter.R_BinN[1][0]*q0aXdT - 
                                                        gKalmanFilter.R_BinN[1][1]*q0aYdT - 
                                                        gKalmanFilter.R_BinN[1][2]*q0aZdT);
    gKalmanFilter.F[STATE_VY][STATE_Q1] = (real)2.0 * (  q2aXdT - q1aYdT - q0aZdT -
                                                        gKalmanFilter.R_BinN[1][0]*q1aXdT - 
                                                        gKalmanFilter.R_BinN[1][1]*q1aYdT - 
                                                        gKalmanFilter.R_BinN[1][2]*q1aZdT);
    gKalmanFilter.F[STATE_VY][STATE_Q2] = (real)2.0 * (  q1aXdT + q2aYdT + q3aZdT -
                                                        gKalmanFilter.R_BinN[1][0]*q2aXdT - 
                                                        gKalmanFilter.R_BinN[1][1]*q2aYdT - 
                                                        gKalmanFilter.R_BinN[1][2]*q2aZdT);
    gKalmanFilter.F[STATE_VY][STATE_Q3] = (real)2.0 * (  q0aXdT - q3aYdT + q2aZdT -
                                                        gKalmanFilter.R_BinN[1][0]*q3aXdT - 
                                                        gKalmanFilter.R_BinN[1][1]*q3aYdT - 
                                                        gKalmanFilter.R_BinN[1][2]*q3aZdT);

    gKalmanFilter.F[STATE_VZ][STATE_Q0] = (real)2.0 * ( -q2aXdT + q1aYdT + q0aZdT -
                                                        gKalmanFilter.R_BinN[2][0]*q0aXdT - 
                                                        gKalmanFilter.R_BinN[2][1]*q0aYdT - 
                                                        gKalmanFilter.R_BinN[2][2]*q0aZdT);
    gKalmanFilter.F[STATE_VZ][STATE_Q1] = (real)2.0 * (  q3aXdT + q0aYdT - q1aZdT -
                                                        gKalmanFilter.R_BinN[2][0]*q1aXdT - 
                                                        gKalmanFilter.R_BinN[2][1]*q1aYdT - 
                                                        gKalmanFilter.R_BinN[2][2]*q1aZdT);
    gKalmanFilter.F[STATE_VZ][STATE_Q2] = (real)2.0 * ( -q0aXdT + q3aYdT - q2aZdT -
                                                        gKalmanFilter.R_BinN[2][0]*q2aXdT - 
                                                        gKalmanFilter.R_BinN[2][1]*q2aYdT - 
                                                        gKalmanFilter.R_BinN[2][2]*q2aZdT);
    gKalmanFilter.F[STATE_VZ][STATE_Q3] = (real)2.0 * (  q1aXdT + q2aYdT + q3aZdT -
                                                        gKalmanFilter.R_BinN[2][0]*q3aXdT - 
                                                        gKalmanFilter.R_BinN[2][1]*q3aYdT - 
                                                        gKalmanFilter.R_BinN[2][2]*q3aZdT);
#else
    gKalmanFilter.F[STATE_VX][STATE_Q0] = (real)2.0 * (q0aXdT - q3aYdT + q2aZdT);
    gKalmanFilter.F[STATE_VX][STATE_Q1] = (real)2.0 * (q1aXdT + q2aYdT + q3aZdT);
    gKalmanFilter.F[STATE_VX][STATE_Q2] = (real)2.0 * (-q2aXdT + q1aYdT + q0aZdT);
    gKalmanFilter.F[STATE_VX][STATE_Q3] = (real)2.0 * (-q3aXdT - q0aYdT + q1aZdT);

    gKalmanFilter.F[STATE_VY][STATE_Q0] = (real)2.0 * (q3aXdT + q0aYdT - q1aZdT);
    gKalmanFilter.F[STATE_VY][STATE_Q1] = (real)2.0 * (q2aXdT - q1aYdT - q0aZdT);
    gKalmanFilter.F[STATE_VY][STATE_Q2] = (real)2.0 * (q1aXdT + q2aYdT + q3aZdT);
    gKalmanFilter.F[STATE_VY][STATE_Q3] = (real)2.0 * (q0aXdT - q3aYdT + q2aZdT);

    gKalmanFilter.F[STATE_VZ][STATE_Q0] = (real)2.0 * (-q2aXdT + q1aYdT + q0aZdT);
    gKalmanFilter.F[STATE_VZ][STATE_Q1] = (real)2.0 * (q3aXdT + q0aYdT - q1aZdT);
    gKalmanFilter.F[STATE_VZ][STATE_Q2] = (real)2.0 * (-q0aXdT + q3aYdT - q2aZdT);
    gKalmanFilter.F[STATE_VZ][STATE_Q3] = (real)2.0 * (q1aXdT + q2aYdT + q3aZdT);
#endif

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

    gKalmanFilter.F[STATE_Q1][STATE_Q0] = gKalmanFilter.wTrueTimesDtOverTwo[X_AXIS];
    //gKalmanFilter.F[STATE_Q1][STATE_Q1] =     0;
    gKalmanFilter.F[STATE_Q1][STATE_Q2] = gKalmanFilter.wTrueTimesDtOverTwo[Z_AXIS];
    gKalmanFilter.F[STATE_Q1][STATE_Q3] = -gKalmanFilter.wTrueTimesDtOverTwo[Y_AXIS];

    gKalmanFilter.F[STATE_Q2][STATE_Q0] = gKalmanFilter.wTrueTimesDtOverTwo[Y_AXIS];
    gKalmanFilter.F[STATE_Q2][STATE_Q1] = -gKalmanFilter.wTrueTimesDtOverTwo[Z_AXIS];
    //gKalmanFilter.F[STATE_Q2][STATE_Q2] =     0;
    gKalmanFilter.F[STATE_Q2][STATE_Q3] = gKalmanFilter.wTrueTimesDtOverTwo[X_AXIS];

    gKalmanFilter.F[STATE_Q3][STATE_Q0] = gKalmanFilter.wTrueTimesDtOverTwo[Z_AXIS];
    gKalmanFilter.F[STATE_Q3][STATE_Q1] = gKalmanFilter.wTrueTimesDtOverTwo[Y_AXIS];
    gKalmanFilter.F[STATE_Q3][STATE_Q2] = -gKalmanFilter.wTrueTimesDtOverTwo[X_AXIS];
    //gKalmanFilter.F[STATE_Q3,STATE.Q3] =     0;

    // Columns corresponding to the rate-bias state (-0.5*Xi*DT)
    q0DtOver2 = gKalmanFilter.quaternion_Past[Q0] * gAlgorithm.dtOverTwo;
    q1DtOver2 = gKalmanFilter.quaternion_Past[Q1] * gAlgorithm.dtOverTwo;
    q2DtOver2 = gKalmanFilter.quaternion_Past[Q2] * gAlgorithm.dtOverTwo;
    q3DtOver2 = gKalmanFilter.quaternion_Past[Q3] * gAlgorithm.dtOverTwo;

    // 
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

// 
static void _UpdateProcessCovariance(void)
{
    // Variables used to initially populate the Q-matrix
    real biSq[3] = {(real)1.0e-10, (real)1.0e-10, (real)1.0e-10};
    real sigDriftDot;

    // Variables used to populate the Q-matrix each time-step
    static real multiplier_Q, multiplier_Q_Sq;

    static BOOL initQ_HG = TRUE;
    static BOOL initQ_LG = TRUE;

    // Only need to generate Q-Bias values once
    if (initQ_HG == TRUE) 
    {
        initQ_HG = FALSE;

        // squated gyro bias instability
        biSq[X_AXIS] = gAlgorithm.imuSpec.biW * gAlgorithm.imuSpec.biW;
        biSq[Y_AXIS] = biSq[X_AXIS];
        biSq[Z_AXIS] = biSq[X_AXIS];


        /* Rate-bias terms (computed once as it does not change with attitude). 
         *   sigDriftDot = (2*pi/ln(2)) * BI^2 / ARW
         *   2*pi/ln(2) = 9.064720283654388
         */
        sigDriftDot = (real)9.064720283654388 / gAlgorithm.imuSpec.arw;

        // Rate-bias terms (Q is ultimately the squared value, which is done in the second line of the assignment)
        gKalmanFilter.Q[STATE_WBX] = sigDriftDot * biSq[X_AXIS] * gAlgorithm.dt;
        gKalmanFilter.Q[STATE_WBX] = gKalmanFilter.Q[STATE_WBX] * gKalmanFilter.Q[STATE_WBX];

        gKalmanFilter.Q[STATE_WBY] = gKalmanFilter.Q[STATE_WBX];

        gKalmanFilter.Q[STATE_WBZ] = sigDriftDot * biSq[Z_AXIS] * gAlgorithm.dt;
        gKalmanFilter.Q[STATE_WBZ] = gKalmanFilter.Q[STATE_WBZ] * gKalmanFilter.Q[STATE_WBZ];

        gKalmanFilter.Q[STATE_ABX] = (real)1.0e-12;
        gKalmanFilter.Q[STATE_ABY] = (real)1.0e-12;
        gKalmanFilter.Q[STATE_ABZ] = (real)1.0e-12;

        /* Precalculate the multiplier applied to the Q terms associated with
         * attitude.  sigRate = ARW / sqrt( dt )
         * mult = 0.5 * dt * sigRate
         *      = 0.5 * sqrt(dt) * sqrt(dt) * ( ARW / sqrt(dt) )
         *      = 0.5 * sqrt(dt) * ARW
         */
        multiplier_Q = (real)(0.5) * gAlgorithm.sqrtDt * gAlgorithm.imuSpec.arw;
        multiplier_Q_Sq = multiplier_Q * multiplier_Q;
    }

    /* Attempt to solve the rate-bias problem: Decrease the process covariance,
     * Q, upon transition to low-gain mode to limit the change in the rate-bias
     * estimate.
     */
    if( initQ_LG == TRUE ) 
    {
        if( gAlgorithm.state == LOW_GAIN_AHRS ) 
        {
            initQ_LG = FALSE;

            /* After running drive-test data through the AHRS simulation,
             * reducing Q by 1000x reduced the changeability of the rate-bias
             * estimate (the estimate was less affected by errors).  This
             * seems to have improved the solution as much of the errors
             * was due to rapid changes in the rate-bias estimate.  This
             * seemed to result in a better than nominal solution (for the
             * drive-test).  Note: this is only called upon the first-entry
             * into low-gain mode.
             */
            /*gKalmanFilter.Q[STATE_WBX] = (real)1.0e-3 * gKalmanFilter.Q[STATE_WBX];
            gKalmanFilter.Q[STATE_WBY] = (real)1.0e-3 * gKalmanFilter.Q[STATE_WBY];
            gKalmanFilter.Q[STATE_WBZ] = (real)1.0e-3 * gKalmanFilter.Q[STATE_WBZ];*/
        }
    }

    /* Update the elements of the process covariance matrix, Q, that change
     * with each time-step (the elements that correspond to the quaternion-
     * block of the Q-matrix).  The rest of the elements in the matrix are set
     * during the transition into and between EKF states (high-gain, low-gain,
     * etc) or above (upon first entry into this function).
     * The process cov matrix of quaternion is
     *          [1-q0*q0    -q0*q1      -q0*q2      -q0*q3;
     *          -q0*q1      1-q1*q1     -q1*q2      -q1*q3;
     *          -q0*q2      -q1*q2      1-q2*q2     -q2*q3;
     *          -q0*q3      -q1*q3      -q2*q3      1-q3*q3] * (0.5*dt*sigma_gyro)^2
     * The eigenvalue of the matrix is [1 1 1 1-q0^2-q1^2-q2^2-q3^2], which means it
     * is not positive defintie when quaternion norm is above or equal to 1. Quaternion
     * norm can be above 1 due to numerical accuray. A scale factor 0.99 is added here to
     * make sure the positive definiteness of the covariance matrix. The eigenvalues now
     * are [1 1 1 1-0.99*(q0^2+q1^2+q2^2+q3^2)]. Even if there is numerical accuracy issue,
     * the cov matrix is still positive definite.
     */
    real q0q0 = gKalmanFilter.quaternion_Past[Q0] * gKalmanFilter.quaternion_Past[Q0] * 0.99f;
    real q0q1 = gKalmanFilter.quaternion_Past[Q0] * gKalmanFilter.quaternion_Past[Q1] * 0.99f;
    real q0q2 = gKalmanFilter.quaternion_Past[Q0] * gKalmanFilter.quaternion_Past[Q2] * 0.99f;
    real q0q3 = gKalmanFilter.quaternion_Past[Q0] * gKalmanFilter.quaternion_Past[Q3] * 0.99f;

    real q1q1 = gKalmanFilter.quaternion_Past[Q1] * gKalmanFilter.quaternion_Past[Q1] * 0.99f;
    real q1q2 = gKalmanFilter.quaternion_Past[Q1] * gKalmanFilter.quaternion_Past[Q2] * 0.99f;
    real q1q3 = gKalmanFilter.quaternion_Past[Q1] * gKalmanFilter.quaternion_Past[Q3] * 0.99f;

    real q2q2 = gKalmanFilter.quaternion_Past[Q2] * gKalmanFilter.quaternion_Past[Q2] * 0.99f;
    real q2q3 = gKalmanFilter.quaternion_Past[Q2] * gKalmanFilter.quaternion_Past[Q3] * 0.99f;

    real q3q3 = gKalmanFilter.quaternion_Past[Q3] * gKalmanFilter.quaternion_Past[Q3] * 0.99f;

    // Note: this block of the covariance matrix is symmetric
    real tmpQMultiplier = multiplier_Q_Sq;
    /* Only considering gyro noise can underestimate the cov of the quaternion.
     * A scale factor 100 is added here. This is mainly for faster convergence
     * of the heading angle in the INS solution.
     */
    if (gAlgorithm.state == INS_SOLUTION)
    {
        tmpQMultiplier = 1.0f * multiplier_Q_Sq;
    }
    gKalmanFilter.Q[STATE_Q0] = ((real)1.0 - q0q0) * tmpQMultiplier;
    gKalmanFilter.Qq[0] = (-q0q1) * tmpQMultiplier;
    gKalmanFilter.Qq[1] = (-q0q2) * tmpQMultiplier;
    gKalmanFilter.Qq[2] = (-q0q3) * tmpQMultiplier;

    gKalmanFilter.Q[STATE_Q1] = ((real)1.0 - q1q1) * tmpQMultiplier;
    gKalmanFilter.Qq[3] = (-q1q2) * tmpQMultiplier;
    gKalmanFilter.Qq[4] = (-q1q3) * tmpQMultiplier;

    gKalmanFilter.Q[STATE_Q2] = ((real)1.0 - q2q2) * tmpQMultiplier;
    gKalmanFilter.Qq[5] = (-q2q3) * tmpQMultiplier;

    gKalmanFilter.Q[STATE_Q3] = ((real)1.0 - q3q3) * tmpQMultiplier;
}


//
// GenerateProcessCovarMatrix.m
void GenerateProcessCovariance(void)
{
    // Initialize the Process Covariance (Q) matrix with values that do not change
    memset(gKalmanFilter.Q, 0, sizeof(gKalmanFilter.Q));

    /* THE FOLLOWING COVARIANCE VALUES AREN'T CORRECT, JUST SELECTED SO THE
     * PROGRAM COULD RUN
     */

    // Acceleration based values
    real dtSigAccelSq = (real)(gAlgorithm.dt * gAlgorithm.imuSpec.sigmaA);
    dtSigAccelSq = dtSigAccelSq * dtSigAccelSq;

    // Position
    gKalmanFilter.Q[STATE_RX] = gAlgorithm.dtSquared * dtSigAccelSq;
    gKalmanFilter.Q[STATE_RY] = gAlgorithm.dtSquared * dtSigAccelSq;
    gKalmanFilter.Q[STATE_RZ] = gAlgorithm.dtSquared * dtSigAccelSq;

    /* Velocity, todo. 100 is to take under-estimated accel bias, gyro bias and
     * attitude error since none of them is Gaussian. Non-Gaussian error produces
     * velocity drift. High-freq vibration can also be handled by this.
     */
    gKalmanFilter.Q[STATE_VX] = 100 * dtSigAccelSq;//(real)1e-10;
    gKalmanFilter.Q[STATE_VY] = 100 * dtSigAccelSq;
    gKalmanFilter.Q[STATE_VZ] = 100 * dtSigAccelSq;

    // Acceleration - bias
    gKalmanFilter.Q[STATE_ABX] = (real)5e-11; //(real)1e-10; // dtSigAccelSq; //%1e-8 %sigmaAccelBiasSq;
    gKalmanFilter.Q[STATE_ABY] = (real)5e-11; //(real)1e-10; //dtSigAccelSq; //%sigmaAccelBiasSq;
    gKalmanFilter.Q[STATE_ABZ] = (real)5e-11; //(real)1e-10; //dtSigAccelSq; //%sigmaAccelBiasSq;
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

    // 0.25 Hz LPF
    if( gAlgorithm.callingFreq == FREQ_100_HZ ) 
    {
        *output = (real)(0.984414127416097) * (*output) + 
                  (real)(0.007792936291952) * (input + inputPast);
    } 
    else 
    {
        *output = (real)(0.992176700177507) * (*output) + 
                  (real)(0.003911649911247) * (input + inputPast);
    }

    inputPast = input;
}
