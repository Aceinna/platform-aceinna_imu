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

    // Prediction variables: P = FxPxFTranspose + Q
    real F[NUMBER_OF_EKF_STATES][NUMBER_OF_EKF_STATES];
    real P[NUMBER_OF_EKF_STATES][NUMBER_OF_EKF_STATES];
    real Q[NUMBER_OF_EKF_STATES];
    real Qq[6]; /* The process cov matrix of quaternion should be 4x4.
                 * Its 4 diagonal terms are stored in Q. 
                 * Its off-diagonol terms are stored in Qq. Because the matrix
                 * is symmetric, only 6 off-diagonal terms need stored.
                 */

    real correctedRate_B[NUM_AXIS];     // [rad/s]
    real correctedAccel_B[NUM_AXIS];    // [m/s/s]
    real linearAccel_B[NUM_AXIS];       // [m/s/s], linear acceleration in body frame, used to detect drive position

    /* Algorithm results. Velocity states are directly used as results for output.
     * The following two are calculated from state
     */
    real eulerAngles[NUM_AXIS];
    double llaDeg[NUM_AXIS];

    // measurements
    real R_BinN[3][3];                  // convert body to NED
    real Rn2e[3][3];                    // Coordinate tranformation matrix from NED to ECEF
    real measuredEulerAngles[3];        // Euler angles measurements
    real rGPS_N[3];                     // current IMU position w.r.t rGPS0_E in NED.
    double rGPS0_E[3];                  // Initial IMU ECEF position when first entering INS state.
    double rGPS_E[3];                   // current IMU ECEF position

    // Update variables: S = HxPxHTranspose + R
    real nu[9];
    real H[3][NUMBER_OF_EKF_STATES];
    real R[9];
    real K[NUMBER_OF_EKF_STATES][3];
    real stateUpdate[NUMBER_OF_EKF_STATES];

    // The following two are used in more than one functions, so they are pre-computed.
    real wTrueTimesDtOverTwo[NUM_AXIS];
    real turnSwitchMultiplier;
} KalmanFilterStruct;

extern KalmanFilterStruct gKalmanFilter;

/* Global Algorithm structure  */
typedef struct {
    // Sensor readings in the body-frame (B)
    real accel_B[NUM_AXIS];         // [m/s/s]
    real angRate_B[NUM_AXIS];       // [rad/s]
    real magField_B[NUM_AXIS];      // [G]

    // GPS information
    uint32_t itow;
    double llaAnt[3];               // Antenna Lat, Lon, ellipsoid Altitude, [rad, rad, meter]
    double vNedAnt[NUM_AXIS];       // Antenna NED velocity, [m/s, m/s, m/s]
    double lla[3];                  // IMU Lat, Lon, ellipsoid Altitude, [rad, rad, meter]
    double vNed[3];                 // IMU NED velocity, [m/s, m/s, m/s]
    float geoidAboveEllipsoid;      // [m]
    real trueCourse;                // Antenna heading, [deg]
    real rawGroundSpeed;            // IMU ground speed, calculated from vNed, [m/s]
    float GPSHorizAcc;              // [m]
    float GPSVertAcc;               // [m]
    float HDOP;
    uint8_t gpsFixType;             // Indicate if this GNSS measurement is valid
    uint8_t numSatellites;          /* Num of satellites in this GNSS measurement.
                                     * This is valid only when there is gps udpate.
                                     */
    BOOL gpsUpdate;                 // Indicate if GNSS measurement is updated.
} EKF_InputDataStruct;

extern EKF_InputDataStruct gEKFInput;


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
} EKF_OutputDataStruct;

extern EKF_OutputDataStruct gEKFOutput;


// Initialize Kalman filter parameters of the INS app
uint8_t InitINSFilter(void);

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

