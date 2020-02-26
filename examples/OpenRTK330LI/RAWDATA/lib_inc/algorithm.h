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
#include "gpsAPI.h"

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
// #define HEADING_MAG             1
#define HEADING_GNSS_LOW        2
#define HEADING_GNSS_HIGH       3

#define  NUMBER_OF_EKF_STATES  16

#define STATE_RX        0
#define STATE_RY        1
#define STATE_RZ        2
#define STATE_VX        3
#define STATE_VY        4
#define STATE_VZ        5
#define STATE_Q0        6
#define STATE_Q1        7
#define STATE_Q2        8
#define STATE_Q3        9
#define STATE_WBX       10
#define STATE_WBY       11
#define STATE_WBZ       12
#define STATE_ABX       13
#define STATE_ABY       14
#define STATE_ABZ       15

#define STATE_ROLL      6
#define STATE_PITCH     7
#define STATE_YAW       8

// IMU spec
/* [Hz], The data sampling rate when calculate ARW and VRW.
* ARW = sigma * sqrt(dt) = sigma * sqrt(1/ODR)
*/
#define RW_ODR      100.0     
/* [rad/sqrt(s)], gyro angular random walk, sampled at 100Hz
* 0.3deg/sqrt(Hr) = 0.3 / 60 * D2R = 8.72664625997165e-05rad/sqrt(s)
*/                 
#define ARW_300ZA   8.73e-5      

#define BIW_300ZA   2.91e-5             /* [rad/s], gyro bias instability
                                         * 6.0deg/Hr = 6.0 / 3600 * D2R = 2.90888208665722e-05rad/s
                                         */
#define MAX_BW      8.73e-3             /* [rad/s], max possible gyro bias
                                         * 0.5deg/s = 0.5 * D2R = 0.00872664625997165rad/s
                                         */
#define VRW_300ZA   1.0e-3              /* [m/s/sqrt(s)], accel velocity random walk, sampled at 100Hz
                                         * 0.06m/s/sqrt(Hr) = 0.06 / 60 = 0.001m/s/sqrt(s)
                                         */
#define BIA_300ZA   10.0e-6 * GRAVITY   /* [m/s/s], accel bias instability
                                         * 10ug = 10.0e-6g * GRAVITY
                                         */
#define MAX_BA      3.0e-3 * GRAVITY    /* [m/s/s], max possible accel bias
                                         * 3mg = 3.0e-3g * GRAVITY
                                         */

// GNSS spec
#define R_VALS_GPS_POS_X                5.0
#define R_VALS_GPS_POS_Y                5.0
#define R_VALS_GPS_POS_Z                7.5

#define R_VALS_GPS_VEL_X                0.025
#define R_VALS_GPS_VEL_Y                0.025
#define R_VALS_GPS_VEL_Z                0.025

// Make these #defines
#define  ROWS_IN_P  16
#define  COLS_IN_P  16

#define  ROWS_IN_H  3
#define  COLS_IN_H  16

#define  ROWS_IN_R  3
#define  COLS_IN_R  ROWS_IN_R

#define  ROWS_IN_K  16

// Size of EKF matrices
#define  ROWS_IN_P  16
#define  COLS_IN_P  16

#define  ROWS_IN_F  16
#define  COLS_IN_F  16

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
    // real magField_B[NUM_AXIS];      // [G]

    // GPS information
    mcu_time_base_t  rovTime;          //gnss time rov->time
    uint16_t week;
    uint32_t itow;
    float age;
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

    uint16_t week;
    uint32_t itow;
    uint8_t gnss_sol_type;
} EKF_OutputDataStruct;

extern EKF_OutputDataStruct gEKFOutput;

typedef struct
{
    BOOL bValid;        // tell if stats are valid
    BOOL bStaticIMU;    // Static period detected by IMU
    BOOL accelErrLimit; // accelErr is set to max/min limit
    real lpfAccel[3];   // [m/s/s], low-pass filtered accel
    real accelNorm;     // [m/s/s], magnitude of current accel
    real accelMean[3];  // [m/s/s], average of past n accel samples
    real accelVar[3];   // [m/s/s]^2, variance of past n accel samples
    real accelErr[3];   // [m/s/s], estimated accel error
    real lpfGyro[3];    // [rad/s], low-pass filtered gyro
    real gyroMean[3];   // [rad/s], average of past n gyro samples
    real gyroVar[3];    // [rad/s]^2, variance of past n gyro samples
} ImuStatsStruct;

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
    uint16_t    week;
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


/******************************************************************************
 * @brief Calculate IMU data stats, and detect zero velocity.
 * TRACE:
 * @param [in]  gyro        [rad/s]
 * @param [in]  accel       [m/s/s]
 * @param [in]  reset       TRUE to reset this process
 * @param [Out] imuStats    results
 * @retval None.
******************************************************************************/
void MotionStatusImu(real *gyro, real *accel, ImuStatsStruct *imuStats, BOOL reset);

/******************************************************************************
 * @brief Using gyro propagation to estimate accel error.
 * g_dot = -cross(w, g), g is gravity and w is angular rate.
 * TRACE:
 * @param [in] accel            Input accel, m/s/s.
 * @param [in] w                Input angular rate, rad/s.
 * @param [in] dt               Sampling interval, sec.
 * @param [in] staticDelay      A Counter. When static period detected, delay [staticDelay] samples before
 *                              lowering accel error. [staticDelay] is also used to reset initial accel that
 *                              is propagated using gyro to estimate future accel.
 * @param [out] gAccelStats     A struct for results storage.
 * @retval None.
******************************************************************************/
void EstimateAccelError(real *accel, real *w, real dt, uint32_t staticDelay, ImuStatsStruct *gAccelStats);

/******************************************************************************
 * @brief Detect motion according to the difference between measured accel magnitude and 1g.
 * Set gAlgorithm.linAccelSwitch to be True if being static for a while.
 * This result is no longer used in current algorithm.
 * TRACE:
 * @param [in] accelNorm Input accel magnitude, g.
 * @param [in] iReset   Reset the procedure.
 * @retval Always true.
******************************************************************************/
BOOL DetectMotionFromAccel(real accelNorm, int iReset);

/******************************************************************************
 * @brief Detect zero velocity using GNSS speed.
 * TRACE:
 * @param [in]  vNED        NED velocity measured by GNSS   [m/s]
 * @param [in]  gnssValid   Indicate if GNSS measurement is valid.
 *                          If valid, vNED will be used to detect zero velocity.
 *                          If not, detection will be reset and FALSE is always
 *                          returned.
 * @retval TRUE if static, otherwise FALSE.
******************************************************************************/
BOOL DetectStaticGnssVelocity(double *vNED, real threshold, BOOL gnssValid);

/******************************************************************************
 * @brief Detect zero velocity using odometer data.
 * TRACE:
 * @param [in]  odo velocity measured by odometer   [m/s]
 * @retval TRUE if static, otherwise FALSE.
******************************************************************************/
BOOL DetectStaticOdo(real odo);


void EKF_PredictionStage(real *filteredAccel);
void GenerateProcessCovariance(void);
void GenerateProcessJacobian(void);


void StabilizeSystem(void);
void InitializeAttitude(void);
void HG_To_LG_Transition_Test(void);
void LG_To_INS_Transition_Test(void);
void INS_To_AHRS_Transition_Test(void);

void DynamicMotion(void);


void EKF_UpdateStage(void);

// Functions to split the INS update across multiple iterations, so the update can
// complete in the required 10 ms
void Update_Pos(void);
void Update_Vel(void);
void Update_Att(void);

void ComputeSystemInnovation_Att(void);
void ComputeSystemInnovation_Pos(void);
void ComputeSystemInnovation_Vel(void);


uint8_t _GenerateObservationJacobian_AHRS(void);

void _GenerateObservationCovariance_AHRS(void);
void _GenerateObservationCovariance_INS(void);


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


#endif
