/*****************************************************************************
* @name xbowsp_algorithm.h
*
* @brief  Copyright (c) 2013, 2014 All Rights Reserved.
*
* THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
* KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
* PARTICULAR PURPOSE.
*
* @rev: 16976
* @date: 2010-11-16 14:59:36 -0800 (Tue, 16 Nov 2010)
* @author: dan
* @brief:
*   -Algorithm data structure used in sensor calibration and communication
* protocols with outside world.
******************************************************************************/
#ifndef XBOWSP_ALGORITHM_H
#define XBOWSP_ALGORITHM_H

#include "GlobalConstants.h"

#include "BITStatus.h" // BITStatusStruct
#include "Indices.h"

// Define the algorithm states
#define STABILIZE_SYSTEM    0
#define INITIALIZE_ATTITUDE 1
#define HIGH_GAIN_AHRS      2
#define LOW_GAIN_AHRS       3
#define INS_SOLUTION        4

typedef enum  {
    NO_ALGO          = 0,
    AHRS_ALGO        = 1,
    INS_ALGO         = 2
} AlgorithmType;

// Specify the minimum state times (in seconds)
#define STABILIZE_SYSTEM_DURATION    0.36    // [sec]
#define INITIALIZE_ATTITUDE_DURATION 0.64    // ( 1.0 - 0.36 ) [sec]
#define HIGH_GAIN_AHRS_DURATION      60.0    // 60.0 * SAMPLING_RATE
#define LOW_GAIN_AHRS_DURATION       30.0    // 30.0 * SAMPLING_RATE

typedef struct {
    uint32_t Stabilize_System;      // SAMPLING_RATE * 0.36
    uint32_t Initialize_Attitude;   // SAMPLING_RATE * ( 1.0 - 0.36 )
    uint32_t High_Gain_AHRS;        // 60.0 * SAMPLING_RATE
    uint32_t Low_Gain_AHRS;         // 30.0 * SAMPLING_RATE
} DurationStruct;

typedef struct {
    int32_t Max_GPS_Drop_Time;   // [msec]
    int32_t Max_Rest_Time_Before_Drop_To_AHRS;   // [msec]
    int32_t Declination_Expiration_Time;   // [msec]

    uint16_t Free_Integration_Cntr;   // [count]
//#define LIMIT_MAX_REST_TIME_BEFORE_DROP_TO_AHRS     60000 // 60000 [ msec ] = 60 [ sec ]
//#define LIMIT_DECL_EXPIRATION_TIME                  60000  // 60,000 [ counts ] = 10 [ min ]

    real accelSwitch;
    uint32_t linAccelSwitchDelay;
    
    //
    real rateAlarm, accelAlarm, magAlarm;
} LimitStruct;


/* Global Algorithm structure  */
typedef struct {
    uint32_t          rawSensors[N_RAW_SENS];
    int32_t           tempCompBias[6];           /// temperature compnesation inertial sensor bias

    double            scaledSensors[N_RAW_SENS]; /// g's, rad/s, G, deg C, (body frame)

    // Used to generate the system ICs
    real              accumulatedAccelVector[3];
    real              accumulatedMagVector[3];
    real              averagedAccelVector[3];
    real              averagedMagVector[3];

    // Yaw-Rate (Turn-Switch) filter
    real              filteredYawRate, filteredYawRatePast;
    real              yawRateMeasPast;

    real              correctedRate[3];          /// algorithm corrected rate
    real              tangentRates[3];
    real              tangentAccels[3];
    real              attitude[3];              /// from qb2t
    real              leveledMags[3];           /// x and y mags (tangent frame w/o user alignment)
    volatile int      calState;                 /// current (mag-align) calibration state
    volatile uint16_t counter;                  /// inc. with every continuous mode output packet
    volatile uint32_t timer;  	                 ///< timer since power up (ms)
    volatile BITStatusStruct bitStatus;   //bit and status structure

    real              downRate;

    real              compassHeading;

    real              RateOutSF, AccelOutSF, MagOutSF, TempOutSF;

    int32_t           scaledSensors_q27[N_RAW_SENS];  // g's, rad/s, G, deg C, (body frame)

    // control the stage of operation for the algorithms
    uint8_t           state;   // takes values from HARDWARE_STABILIZE to INIT_ATTITUDE to HG_AHRS
    uint32_t          stateTimer; // may not need

    int insFirstTime;
    int applyDeclFlag;

    int32_t timeOfLastSufficientGPSVelocity;
    int32_t timeOfLastGoodGPSReading;

    double rGPS0_E[3];
    real  rGPS_N[3];

    int   magAlignUnderway;

    float             tempMisalign[18];

    double llaRad[3], rGPS_E[3];
    real R_NinE[3][3];

    uint32_t itow, dITOW;

    uint8_t  callingFreq;
    real dt, dtOverTwo, dtSquared, sqrtDt;

    // The following variables are used to increase the Kalman filter gain when the
    //   acceleration is very close to one (i.e. the system is at rest)
    real aMag;
    real aFilt_N[3];
    real aMagThreshold;
    uint32_t linAccelSwitchCntr;
    uint8_t linAccelSwitch;
    real R_Mult;

    AlgorithmType     algorithmType;                  /// AHRS or INS

    DurationStruct    Duration;
    LimitStruct       Limit;
} AlgorithmStruct;

extern AlgorithmStruct gAlgorithm; // xbowsp_init.c

void InitializeAlgorithmStruct(AlgorithmStruct *algorithmStruct);
void InitializeSensorScaling(void);

// Declare the getter and setter functions (None, AHRS, INS)
AlgorithmType  getAlgoType(void);
void           setAlgoType(AlgorithmType inType);

#endif
