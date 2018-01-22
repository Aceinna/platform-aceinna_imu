/*
 * File:   EKF_Algorithm.h
 * Author: joemotyka
 *
 * Created on May 8, 2016, 12:23 AM
 */

#ifndef _EKF_ALGORITHM_H_
#define _EKF_ALGORITHM_H_

#include <stdint.h>

#include "GlobalConstants.h"
#include "StateIndices.h"
#include "UpdateMatrixSizing.h"  // used to specify the size of the update vectors

#include "Indices.h"

// Changed to 1e-2 on Sep 13, 2016
#define INIT_P 0.01


// Global Kalman Filter structure
typedef struct {
    // States
    real Velocity_N[3];
    real Position_N[3];
    real rateBias_B[3];
    real accelBias_B[3];

    real stateUpdate[NUMBER_OF_EKF_STATES];

    // Prediction variables: P = FxPxFTranspose + Q
    real Q[NUMBER_OF_EKF_STATES][NUMBER_OF_EKF_STATES];
    real F[NUMBER_OF_EKF_STATES][NUMBER_OF_EKF_STATES];
    real P[NUMBER_OF_EKF_STATES][NUMBER_OF_EKF_STATES];

    real correctedRate_B[3];
    real correctedAccel_B[3];
    real aCorr_N[3];
    real aMotion_N[3];

    real R_BinN[3][3];   // 321-Rotation matrix

    real quaternion[4],  measuredQuaternion[4], quaternion_Past[4];
    real eulerAngles[3], measuredEulerAngles[3];
    real attitudeError[3];

    // Update variables: S = HxPxHTranspose + R
    // DEBUG H is 4x16 or 10x16 (AHRS vs INS)
    real nu[10];
    real R_INS[10][10];
#ifdef EULER_ANGLE_SOLN
    real H[3][NUMBER_OF_EKF_STATES];   // DEBUG: <--- this is incorrect!  H is 4x16!  correct this and proceed
    real S[3][3], SInverse[3][3];
    real R[3][3];
    real K[NUMBER_OF_EKF_STATES][3];
    real K_Q[NUMBER_OF_EKF_STATES][4];

    real K_RP[NUMBER_OF_EKF_STATES][2];
    real K_Y[NUMBER_OF_EKF_STATES][1];

    real H_INS[9][NUMBER_OF_EKF_STATES];
//    real R_INS[9][9];
#else
    real H[4][NUMBER_OF_EKF_STATES];   // DEBUG: <--- this is incorrect!  H is 4x16!  correct this and proceed
    real S[4][4], SInverse[4][4];
    real R[4][4];
    real K[NUMBER_OF_EKF_STATES][4];

    real H_INS[10][NUMBER_OF_EKF_STATES];
    real R_INS[10][10];

    real nu[10];
#endif

    double llaDeg[3];
//    double Position_E[3];

    real wTrueTimesDtOverTwo[3];

// Not in v18.1.10 but keep
    real turnSwitchMultiplier;
} KalmanFilterStruct;

extern KalmanFilterStruct gKalmanFilter;

void EKF_Algorithm(void);

#endif /* _EKF_ALGORITHM_H_ */

