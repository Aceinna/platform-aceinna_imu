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

#include "gpsAPI.h"   // For gpsDataStruct_t in EKF setter

#include "Indices.h"

// Changed to 1e-2 on Sep 13, 2016
#define INIT_P 0.01


// Global Kalman Filter structure
typedef struct {
    // States
    real Velocity_N[NUM_AXIS];
    real Position_N[NUM_AXIS];
    real quaternion[4], quaternion_Past[4];
    real rateBias_B[NUM_AXIS];
    real accelBias_B[NUM_AXIS];

    real stateUpdate[NUMBER_OF_EKF_STATES];

    // Prediction variables: P = FxPxFTranspose + Q
    real F[NUMBER_OF_EKF_STATES][NUMBER_OF_EKF_STATES];
    real P[NUMBER_OF_EKF_STATES][NUMBER_OF_EKF_STATES];
    real Q[NUMBER_OF_EKF_STATES];
    real Qq[6]; /* The process cov matrix of quaternion should be 4x4.
                 * Its 4 diagonal terms are stored in Q. 
                 * Its off-diagonol terms are stored in Qq. Because the matrix
                 * is symmetric, only 6 off-diagonal terms need stored.
                 */

    real correctedRate_B[NUM_AXIS];
    real correctedAccel_B[NUM_AXIS];
    real aCorr_N[NUM_AXIS];

    real R_BinN[NUM_AXIS][NUM_AXIS];   // convert body to NED
    real eulerAngles[NUM_AXIS], measuredEulerAngles[NUM_AXIS];

    // Update variables: S = HxPxHTranspose + R
    // DEBUG H is 3x16 or 9x16 (AHRS vs INS)
    real nu[9];

    real H[3][NUMBER_OF_EKF_STATES];
    real R[9];
    real K[NUMBER_OF_EKF_STATES][3];

    double llaDeg[NUM_AXIS];
//    double Position_E[NUM_AXIS];

    real wTrueTimesDtOverTwo[NUM_AXIS];

    real turnSwitchMultiplier;
} KalmanFilterStruct;

extern KalmanFilterStruct gKalmanFilter;

/* Global Algorithm structure  */
typedef struct {
    // Sensor readings in the body-frame (B)
    real            accel_B[NUM_AXIS];      // [g]
    real            angRate_B[NUM_AXIS];    // [rad/s]
    real            magField_B[NUM_AXIS];   // [G]

    // GPS information
    BOOL gpsValid;
    BOOL gpsUpdate;
    
    double llaRad[3];
    double vNed[NUM_AXIS];

    double trueCourse;
    double rawGroundSpeed;
    
    uint32_t itow;

    float GPSHorizAcc, GPSVertAcc;
    float HDOP;
} EKF_InputDataStruct;

extern EKF_InputDataStruct gEKFInputData;


/* Global Algorithm structure  */
typedef struct {
    // Algorithm states (15 states)
    double            position_N[NUM_AXIS];
    double            velocity_N[NUM_AXIS];
    double            quaternion_BinN[4];
    double            angRateBias_B[NUM_AXIS];
    double            accelBias_B[NUM_AXIS];
    
    double            llaDeg[NUM_AXIS];

    // Derived variables
    double            eulerAngs_BinN[NUM_AXIS];
    double            corrAngRates_B[NUM_AXIS];
    double            corrAccel_B[NUM_AXIS];

    // Operational states
    uint8_t           opMode;
    uint8_t           turnSwitchFlag;
    uint8_t           linAccelSwitch;
    uint8_t           gpsMeasurementUpdate;
} EKF_OutputDataStruct;

extern EKF_OutputDataStruct gEKFOutputData;

typedef struct
{
    BOOL bValid;        // tell if stats are valid
	BOOL accelErrLimit; // accelErr is set to max/min limit
	real lpfAccel[3];   // low-pass filtered accel
    real accelNorm;     // magnitude of current accel
    real accelMean[3];  // average of past n accel
    real accelVar[3];   // variance of past n accel
    real accelErr[3];   // estimated accel error
} AccelStatsStruct;

void EKF_Algorithm(void);
void enableFreeIntegration(BOOL enable);

// Getters for data extraction from the EKF output data structure
void EKF_GetAttitude_EA(real *EulerAngles);
void EKF_GetAttitude_EA_RAD(real *EulerAngles);
void EKF_GetAttitude_Q(real *Quaternions);
void EKF_GetCorrectedAngRates(real *CorrAngRates_B);
void EKF_GetCorrectedAccels(real *CorrAccels_B);
void EKF_GetEstimatedAngRateBias(real *AngRateBias_B);
void EKF_GetEstimatedAccelBias(real *AccelBias_B);
void EKF_GetEstimatedPosition(real *Position_N);
void EKF_GetEstimatedVelocity(real *Velocity_N);
void EKF_GetEstimatedLLA(double *LLA);

void EKF_GetOperationalMode(uint8_t *EKF_OperMode);
void EKF_GetOperationalSwitches(uint8_t *EKF_LinAccelSwitch, uint8_t *EKF_TurnSwitch);

// Setter functions
void EKF_SetInputStruct(double *accels, double *rates, double *mags, gpsDataStruct_t *gps);
void EKF_SetOutputStruct(void);

#endif /* _EKF_ALGORITHM_H_ */

