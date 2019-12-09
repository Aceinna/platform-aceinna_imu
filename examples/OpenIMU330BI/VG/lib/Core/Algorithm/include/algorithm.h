/*****************************************************************************
* @name algorihm.h
*
*
* THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
* KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
* PARTICULAR PURPOSE.
*
*   -Algorithm data structure used in sensor calibration and communication
* protocols with outside world.
******************************************************************************/
/*******************************************************************************
Copyright 2018 ACEINNA, INC

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

	http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*******************************************************************************/

#ifndef ALGORITHM_H
#define ALGORITHM_H

#include "GlobalConstants.h"

#include "Indices.h"

// Define the algorithm states
#define STABILIZE_SYSTEM    0
#define INITIALIZE_ATTITUDE 1
#define HIGH_GAIN_AHRS      2
#define LOW_GAIN_AHRS       3
#define INS_SOLUTION        4


// Specify the minimum state times (in seconds)
#define STABILIZE_SYSTEM_DURATION    0.36    // [sec]
#define INITIALIZE_ATTITUDE_DURATION 0.64    // ( 1.0 - 0.36 ) [sec]
#define HIGH_GAIN_AHRS_DURATION      30.0    // 60.0 * SAMPLING_RATE
#define LOW_GAIN_AHRS_DURATION       30.0    // 30.0 * SAMPLING_RATE

// Define heading initialization reliability
#define HEADING_UNINITIALIZED   0
#define HEADING_MAG             1
#define HEADING_GNSS_LOW        2
#define HEADING_GNSS_HIGH       3

typedef struct {
	uint32_t Stabilize_System;      // SAMPLING_RATE * 0.36
	uint32_t Initialize_Attitude;   // SAMPLING_RATE * ( 1.0 - 0.36 )
	uint32_t High_Gain_AHRS;        // 60.0 * SAMPLING_RATE
	uint32_t Low_Gain_AHRS;         // 30.0 * SAMPLING_RATE
} DurationStruct;

typedef struct {
	real positionError;
	real velocityError;
	real attitudeError;
} InnovationStruct;

typedef struct {
    int32_t maxGpsDropTime;     // [msec]
    int32_t maxReliableDRTime;  /* [msec] When GPS outage duration exceeds this limit,
                                 * the position and velocity will be reinitialized when GPS
                                 * is available again. Otherwise, the fusion algorithm will
                                 * gradually correct the position and velocity to the GPS.
                                 */
	int32_t Max_Rest_Time_Before_Drop_To_AHRS;   // [msec]
	int32_t Declination_Expiration_Time;   // [msec]

	uint16_t Free_Integration_Cntr;   // [count]
//#define LIMIT_MAX_REST_TIME_BEFORE_DROP_TO_AHRS     60000 // 60000 [ msec ] = 60 [ sec ]
//#define LIMIT_DECL_EXPIRATION_TIME                  60000  // 60,000 [ counts ] = 10 [ min ]

	real accelSwitch;
	uint32_t linAccelSwitchDelay;

	InnovationStruct       Innov;
} LimitStruct;

/// specifying how the user sets up the device algorithm
struct algoBehavior_BITS {        /// bits   description
	uint16_t freeIntegrate : 1; /// 0
	uint16_t useMag : 1; /// 1
	uint16_t useGPS : 1; /// 2 - Not used yet
	uint16_t stationaryLockYaw : 1; /// 3 - Not used yet
	uint16_t restartOnOverRange : 1; /// 4
	uint16_t dynamicMotion : 1; /// 5 - Not used
	uint16_t rsvd : 10; /// 6:15
};

union AlgoBehavior
{
	uint16_t                 all;
	struct algoBehavior_BITS bit;
};

// Algorithm states
struct ALGO_STATUS_BITS
{
	uint16_t algorithmInit : 1; // 0  algorithm initialization
	uint16_t highGain : 1; // 1  high gain mode
	uint16_t attitudeOnlyAlgorithm : 1; // 2  attitude only algorithm
	uint16_t turnSwitch : 1; // 3  turn switch
	uint16_t noAirdataAiding : 1; // 4  airdata aiding
	uint16_t noMagnetometerheading : 1; // 5  magnetometer heading
	uint16_t noGPSTrackReference : 1; // 6  GPS track
	uint16_t gpsUpdate : 1; // 7  GPS measurement update
	uint16_t rsvd : 8; // 8:15
};

typedef union ALGO_STATUS
{
	uint16_t                all;
	struct ALGO_STATUS_BITS bit;
} AlgoStatus;

extern AlgoStatus gAlgoStatus;

typedef struct
{
    real arw;                       // [rad/sqrt(s)], gyro angle random walk
    real sigmaW;                    // [rad/s], gyro noise std
    real biW;                       // [rad/s], gyro bias instability
    real maxBiasW;                  // [rad/s], max possible gyro bias
    real vrw;                       // [m/s/sqrt(s)], accel velocity random walk
    real sigmaA;                    // [m/s/s], accel noise std
    real biA;                       // [m/s/s], accel bias instability
    real maxBiasA;                  // [m/s/s], max possible accel bias
} IMU_SPEC;

typedef struct 
{
    real staticVarGyro;             // [rad/s]^2
    real staticVarAccel;            // [m/s/s]^2
    real maxGyroBias;               // [rad/s]
    real staticGnssVel;             // [m/s]
    real staticNoiseMultiplier[3];  /* Use IMU noise level and gyro output to detect static period.
                                     * The nominal noise level and max gyro bias of an IMU is defined in
                                     * SensorNoiseParameters.h. These parameters are determined by IMU
                                     * output when static and are hightly related to ARW and VRW.
                                     * When IMU is installed on a vehicle, its noise level when
                                     * vehicle is static could be higher than the nominal noise
                                     * level due to vibration. This setting is used to scale
                                     * the nominal noise level and gyro bias for static detection.
                                     * [scale_gyro_var, scale_accel_var, scale_gyro_bias]
                                     */
} STATIC_DETECT_SETTING;


/* Global Algorithm structure  */
typedef struct {
    uint32_t    itow;
    uint32_t    dITOW;

    // control the stage of operation for the algorithms
    uint32_t    stateTimer;
    uint8_t     state;			// takes values from HARDWARE_STABILIZE to INIT_ATTITUDE to HG_AHRS

    uint8_t insFirstTime;
    uint8_t headingIni;
    uint8_t applyDeclFlag;

    int32_t timeOfLastSufficientGPSVelocity;
    int32_t timeOfLastGoodGPSReading;

    real filteredYawRate;				// Yaw-Rate (Turn-Switch) filter

    /* The following variables are used to increase the Kalman filter gain when the
     * acceleration is very close to one (i.e. the system is at rest)
     */
    uint32_t linAccelSwitchCntr;
    uint8_t linAccelSwitch;

    uint8_t linAccelLPFType;
    uint8_t useRawAccToDetectLinAccel;

    uint8_t callingFreq;
    real    dt;
    real    dtOverTwo;
    real    dtSquared;
    real    sqrtDt;

    volatile uint32_t timer;  			// timer since power up (ms)
    volatile uint16_t counter;			// inc. with every continuous mode output packet

    union   AlgoBehavior Behavior;
    float    turnSwitchThreshold;		// 0, 0.4, 10 driving, 1 flying [deg/sec]   0x000d

    real leverArmB[3];					// Antenna position w.r.t IMU in vehicle body frame
    real pointOfInterestB[3];			// Point of interest position w.r.t IMU in vehicle body frame

    BOOL velocityAlwaysAlongBodyX;      // enable zero velocity update

    IMU_SPEC imuSpec;                   // IMU specifications
    STATIC_DETECT_SETTING staticDetectParam;    // params used for static detection         

    DurationStruct    Duration;
    LimitStruct       Limit;
} AlgorithmStruct;

extern AlgorithmStruct gAlgorithm;

#endif
