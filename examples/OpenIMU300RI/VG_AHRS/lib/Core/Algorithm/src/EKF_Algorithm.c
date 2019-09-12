/*
 * File:   EKF_Algorithms.c
 * Author: joemotyka
 *
 * Created on May 8, 2016, 12:35 AM
 */
#include <math.h>       // std::abs
#include <stdlib.h>     // EXIT_FAILURE
#include <stdio.h>

#include "GlobalConstants.h"   // TRUE, FALSE, etc
#include "platformAPI.h"
#include "Indices.h"    // IND

#include "MagAlign.h"
#include "algorithm.h"  // gAlgorithm
#include "AlgorithmLimits.h"
#include "TransformationMath.h"
#include "VectorMath.h"
#include "buffer.h"
#include "SelectState.h"
#include "EKF_Algorithm.h"
#include "PredictFunctions.h"
#include "UpdateFunctions.h"
#include "TimingVars.h"


#ifndef INS_OFFLINE
#ifdef DISPLAY_DIAGNOSTIC_MSG
#include "debug.h"
#endif
#endif

KalmanFilterStruct    gKalmanFilter;
EKF_InputDataStruct   gEKFInputData;
EKF_OutputDataStruct  gEKFOutputData;
AccelStatsStruct      gAccelStats;

//=============================================================================
// Filter variables (Third-Order BWF w/ default 5 Hz Cutoff)
#define FILTER_ORDER 3

#define CURRENT 0
#define PASTx1  1
#define PASTx2  2
#define PASTx3  3
/* Replace this with a fuction that will compute the coefficients so the
 * input is the cutoff frequency in Hertz
 */
#define  NO_LPF              0
#define  TWO_HZ_LPF          1
#define  FIVE_HZ_LPF         2
#define  TEN_HZ_LPF          3
#define  TWENTY_HZ_LPF       4
#define  TWENTY_FIVE_HZ_LPF  5
#define  N_LPF               6

/******************************************************************************
 * @brief Get filter coefficients of a 3rd Butterworth low-pass filter.
 * For now only a few specific cut-off frequencies are supported.
 * TRACE:
 * @param [in] lpfType   Low-pass filter cut-off frequency.
 * @param [in] callingFreq   Sampling frequency, only 100Hz and 200Hz are supported.
 * @param [out] b    coefficients of the numerator of the filter.
 * @param [out] a    coefficients of the denominator of the filter.
 * @retval None.
******************************************************************************/
static void _PopulateFilterCoefficients( uint8_t lpfType, uint8_t callingFreq, real *b, real *a );

/******************************************************************************
 * @brief Process input data through a low-pass Butterworth filter.
 * TRACE:
 * @param [in] accel Input accel
 * @param [in] lpfType   Low-pass filter cut-off frequency.
 * @param [in] callingFreq   Sampling frequency, only 100Hz and 200Hz are supported.
 * @param [out] filteredAccel    Filter accel.
 * @retval None.
******************************************************************************/
static void _AccelLPF( real *accel, uint8_t lpfType, uint8_t callingFreq, real *filteredAccel );

#define DATA_NUM_FOR_STATS  20  // 20 samples can give a relative good estimate of var
/******************************************************************************
 * @brief Compute mean and var of the input data.
 * A recursive mean calculation and a naive var calculation are implemented. A recursive var
 * calculation was also tested but abandoned due to its inherent numerical stability.
 * TRACE:
 * @param [in] accel Input accel, g.
 * @param [in] iReset   Reset the procedure.
 * @param [out] gAccelStats    A struct for results storage.
 * @retval None.
******************************************************************************/
static void ComputeAccelStats( real *accel, AccelStatsStruct *gAccelStats, int iReset );

/******************************************************************************
 * @brief Using gyro propagation to estimate accel error.
 * g_dot = -cross(w, g), g is gravity and w is angular rate.
 * TRACE:
 * @param [in] accel Input accel, g.
 * @param [in] w   Input angular rate, rad/s.
 * @param [in] dt   Sampling interval, sec.
 * @param [out] gAccelStats    A struct for results storage.
 * @retval None.
******************************************************************************/
static void EstimateAccelError( real *accel, real *w, real dt, AccelStatsStruct *gAccelStats);

/******************************************************************************
 * @brief Detect motion according to the difference between measured accel magnitude and 1g.
 * Set gAlgorithm.linAccelSwitch to be True if being static for a while. 
 * This result is no longer used in current algorithm.
 * TRACE:
 * @param [in] accelNorm Input accel magnitude, g.
 * @param [in] iReset   Reset the procedure.
 * @retval Always true.
******************************************************************************/
static BOOL DetectMotionFromAccel(real accelNorm, int iReset);

//=============================================================================

/*  This routine is called at either 100 or 200 Hz based upon the system configuration:
 *   -- Unaided soln: 200 Hz
 *   -- Aided soln: 100 Hz
 */
void EKF_Algorithm(void)
{
    static uint16_t freeIntegrationCounter = 0;

    /* After STABILIZE_SYSTEM, the accel data will first pass a low-pass filter.
     * Stats of the filter accel will then be calculated.
     * According to gAlgorithm.useRawAccToDetectLinAccel,
     * raw or filtered accel is used to detect linear accel.
     */
    if ( gAlgorithm.state > STABILIZE_SYSTEM )
    {
        // Low-pass filter
        _AccelLPF( gEKFInputData.accel_B,
                   gAlgorithm.linAccelLPFType,
                   gAlgorithm.callingFreq,
                   gAccelStats.lpfAccel );
        // Compute accel norm using raw accel data. The norm will be used to detect static periods.
        gAccelStats.accelNorm = sqrtf(gEKFInputData.accel_B[X_AXIS] * gEKFInputData.accel_B[X_AXIS] +
                                      gEKFInputData.accel_B[Y_AXIS] * gEKFInputData.accel_B[Y_AXIS] +
                                      gEKFInputData.accel_B[Z_AXIS] * gEKFInputData.accel_B[Z_AXIS] );
        /* Compute accel mean/var.
         * Compute this after STABILIZE_SYSTEM so there is enough data to fill the buffer
         * Before buffer is full, results are not accurate and not used indeed.
         */
        ComputeAccelStats(gAccelStats.lpfAccel, &gAccelStats, 0);

        // estimate accel error
        if (gAlgorithm.useRawAccToDetectLinAccel)
        {
            EstimateAccelError( gEKFInputData.accel_B,
                                gEKFInputData.angRate_B,
                                gAlgorithm.dt,
                                &gAccelStats);
        }
        else
        {
            EstimateAccelError( gAccelStats.lpfAccel,
                                gEKFInputData.angRate_B,
                                gAlgorithm.dt,
                                &gAccelStats);
        }
        
        // Detect if the unit is static or dynamic
        DetectMotionFromAccel(gAccelStats.accelNorm, 0);
    }

    // Compute the EKF solution if past the stabilization and initialization stages
    if( gAlgorithm.state > INITIALIZE_ATTITUDE )
    {
        // Increment the algorithm itow
        gAlgorithm.itow = gAlgorithm.itow + gAlgorithm.dITOW;

        // Perform EKF Prediction
        EKF_PredictionStage(gAccelStats.lpfAccel);

        /* Update the predicted states if not freely integrating
         * NOTE: free- integration is not applicable in HG AHRS mode.
         */
        if (gAlgorithm.Behavior.bit.freeIntegrate && (gAlgorithm.state > HIGH_GAIN_AHRS))
        {
            /* Limit the free-integration time before reverting to the complete
             * EKF solution (including updates).
             */
            freeIntegrationCounter = freeIntegrationCounter + 1;   // [cycles]
            if (freeIntegrationCounter >= gAlgorithm.Limit.Free_Integration_Cntr) 
            {
                freeIntegrationCounter = 0;
                enableFreeIntegration(FALSE);

#ifdef DISPLAY_DIAGNOSTIC_MSG
                // Display the time at the end of the free-integration period
                TimingVars_DiagnosticMsg("Free integration period ended");
#endif
            }
            // Restart the system in LG AHRS after free integration is complete
            gAlgorithm.insFirstTime = TRUE;
            gAlgorithm.state = LOW_GAIN_AHRS;
            gAlgorithm.stateTimer = gAlgorithm.Duration.Low_Gain_AHRS;
        }
        else 
        {
            enableFreeIntegration(FALSE);
            freeIntegrationCounter = 0;

            // Perform EKF Update
            EKF_UpdateStage();
        }
        /* Save the past attitude quaternion before updating (for use in the
         * covariance estimation calculations).
         */
        gKalmanFilter.quaternion_Past[Q0] = gKalmanFilter.quaternion[Q0];
        gKalmanFilter.quaternion_Past[Q1] = gKalmanFilter.quaternion[Q1];
        gKalmanFilter.quaternion_Past[Q2] = gKalmanFilter.quaternion[Q2];
        gKalmanFilter.quaternion_Past[Q3] = gKalmanFilter.quaternion[Q3];
    }

    /* Select the algorithm state based upon the present state as well as
     * operational conditions (time, sensor health, etc).  Note: This is called
     * after the the above code-block to prevent the transition from occuring
     * until the next time step.
     */
    switch( gAlgorithm.state ) 
    {
        case STABILIZE_SYSTEM:
            StabilizeSystem();
            break;
        case INITIALIZE_ATTITUDE:
            InitializeAttitude();
            break;
        case HIGH_GAIN_AHRS:
            HG_To_LG_Transition_Test();
            break;
        case LOW_GAIN_AHRS:
            LG_To_INS_Transition_Test();
            break;
        case INS_SOLUTION:
            INS_To_AHRS_Transition_Test();
            break;
        default:
#ifdef DISPLAY_DIAGNOSTIC_MSG
#ifdef INS_OFFLINE
            // Shouldn't be able to make it here
            TimingVars_DiagnosticMsg("Uh-oh! Invalid algorithm state in EKF_Algorithm.cpp");
            std::cout << "Press enter to finish ...";
            std::cin.get();
#endif
#endif
            return;
    }

    // Dynamic motion logic (to revert back to HG AHRS)
    DynamicMotion();
}

void enableFreeIntegration(BOOL enable)
{
    gAlgorithm.Behavior.bit.freeIntegrate = enable;
}


BOOL freeIntegrationEnabled()
{
    return gAlgorithm.Behavior.bit.freeIntegrate;
}   

void enableMagInAlgorithm(BOOL enable)
{
    if(1)
    {
        gAlgorithm.Behavior.bit.useMag = enable;
    }
    else
    {
        gAlgorithm.Behavior.bit.useMag = FALSE;
    }
}

BOOL magUsedInAlgorithm()
{
    return gAlgorithm.Behavior.bit.useMag != 0;
}

BOOL gpsUsedInAlgorithm(void)
{
    return gAlgorithm.Behavior.bit.useGPS;
}

void enableGpsInAlgorithm(BOOL enable)
{
    gAlgorithm.Behavior.bit.useGPS = enable;
}



// Getters based on results structure passed to WriteResultsIntoOutputStream()

/* Extract the attitude (expressed as Euler-angles) of the body-frame (B)
 * in the NED-frame (N) in [deg].
 */
void EKF_GetAttitude_EA(real *EulerAngles)
{
    // Euler-angles in [deg]
    EulerAngles[ROLL]  = (real)gEKFOutputData.eulerAngs_BinN[ROLL];
    EulerAngles[PITCH] = (real)gEKFOutputData.eulerAngs_BinN[PITCH];
    EulerAngles[YAW]   = (real)gEKFOutputData.eulerAngs_BinN[YAW];
}


void EKF_GetAttitude_EA_RAD(real *EulerAngles)
{
    // Euler-angles in [rad]
    EulerAngles[ROLL]  = (real)gKalmanFilter.eulerAngles[ROLL];
    EulerAngles[PITCH] = (real)gKalmanFilter.eulerAngles[PITCH];
    EulerAngles[YAW]   = (real) gKalmanFilter.eulerAngles[YAW];
}


/* Extract the attitude (expressed by quaternion-elements) of the body-
 * frame (B) in the NED-frame (N).
 */
void EKF_GetAttitude_Q(real *Quaternions)
{
    Quaternions[Q0] = (real)gEKFOutputData.quaternion_BinN[Q0];
    Quaternions[Q1] = (real)gEKFOutputData.quaternion_BinN[Q1];
    Quaternions[Q2] = (real)gEKFOutputData.quaternion_BinN[Q2];
    Quaternions[Q3] = (real)gEKFOutputData.quaternion_BinN[Q3];
}


/* Extract the angular-rate of the body (corrected for estimated rate-bias)
 * measured in the body-frame (B).
 */
void EKF_GetCorrectedAngRates(real *CorrAngRates_B)
{
    // Angular-rate in [deg/s]
    CorrAngRates_B[X_AXIS] = (real)gEKFOutputData.corrAngRates_B[X_AXIS];
    CorrAngRates_B[Y_AXIS] = (real)gEKFOutputData.corrAngRates_B[Y_AXIS];
    CorrAngRates_B[Z_AXIS] = (real)gEKFOutputData.corrAngRates_B[Z_AXIS];
}


/* Extract the acceleration of the body (corrected for estimated
 * accelerometer-bias) measured in the body-frame (B).
 */
void EKF_GetCorrectedAccels(real *CorrAccels_B)
{
    // Acceleration in [m/s^2]
    CorrAccels_B[X_AXIS] = (real)gEKFOutputData.corrAccel_B[X_AXIS];
    CorrAccels_B[Y_AXIS] = (real)gEKFOutputData.corrAccel_B[Y_AXIS];
    CorrAccels_B[Z_AXIS] = (real)gEKFOutputData.corrAccel_B[Z_AXIS];
}


/* Extract the acceleration of the body (corrected for estimated
 * accelerometer-bias) measured in the body-frame (B).
 */
void EKF_GetEstimatedAngRateBias(real *AngRateBias_B)
{
    // Angular-rate bias in [deg/sec]
    AngRateBias_B[X_AXIS] = (real)gEKFOutputData.angRateBias_B[X_AXIS];
    AngRateBias_B[Y_AXIS] = (real)gEKFOutputData.angRateBias_B[Y_AXIS];
    AngRateBias_B[Z_AXIS] = (real)gEKFOutputData.angRateBias_B[Z_AXIS];
}


/* Extract the acceleration of the body (corrected for estimated
 * accelerometer-bias) measured in the body-frame (B).
 */
void EKF_GetEstimatedAccelBias(real *AccelBias_B)
{
    // Acceleration-bias in [m/s^2]
    AccelBias_B[X_AXIS] = (real)gEKFOutputData.accelBias_B[X_AXIS];
    AccelBias_B[Y_AXIS] = (real)gEKFOutputData.accelBias_B[Y_AXIS];
    AccelBias_B[Z_AXIS] = (real)gEKFOutputData.accelBias_B[Z_AXIS];
}


// Extract the Position of the body measured in the NED-frame (N)
void EKF_GetEstimatedPosition(real *Position_N)
{
    // Position in [m]
    Position_N[X_AXIS] = (real)gEKFOutputData.position_N[X_AXIS];
    Position_N[Y_AXIS] = (real)gEKFOutputData.position_N[Y_AXIS];
    Position_N[Z_AXIS] = (real)gEKFOutputData.position_N[Z_AXIS];
}


// Extract the Position of the body measured in the NED-frame (N)
void EKF_GetEstimatedVelocity(real *Velocity_N)
{
    // Velocity in [m/s]
    Velocity_N[X_AXIS] = (real)gEKFOutputData.velocity_N[X_AXIS];
    Velocity_N[Y_AXIS] = (real)gEKFOutputData.velocity_N[Y_AXIS];
    Velocity_N[Z_AXIS] = (real)gEKFOutputData.velocity_N[Z_AXIS];
}


// Extract the Position of the body measured in the NED-frame (N)
void EKF_GetEstimatedLLA(double *LLA)
{
    // Velocity in [m/s]
    LLA[X_AXIS] = (double)gEKFOutputData.llaDeg[X_AXIS];
    LLA[Y_AXIS] = (double)gEKFOutputData.llaDeg[Y_AXIS];
    LLA[Z_AXIS] = (double)gEKFOutputData.llaDeg[Z_AXIS];
}


/* Extract the Operational Mode of the Algorithm:
 *   0: Stabilize
 *   1: Initialize
 *   2: High-Gain VG/AHRS mode
 *   3: Low-Gain VG/AHRS mode
 *   4: INS operation
 */
void EKF_GetOperationalMode(uint8_t *EKF_OperMode)
{
    *EKF_OperMode = gEKFOutputData.opMode;
}


// Extract the linear-acceleration and turn-switch flags
void EKF_GetOperationalSwitches(uint8_t *EKF_LinAccelSwitch, uint8_t *EKF_TurnSwitch)
{
    *EKF_LinAccelSwitch = gEKFOutputData.linAccelSwitch;
    *EKF_TurnSwitch     = gEKFOutputData.turnSwitchFlag;
}


// SETTERS: for EKF input and output structures

// Populate the EKF input structure with sensor and GPS data (if used)
void EKF_SetInputStruct(double *accels, double *rates, double *mags, gpsDataStruct_t *gps)
{
    // Accelerometer signal is in [g]
    gEKFInputData.accel_B[X_AXIS]    = (real)accels[X_AXIS];
    gEKFInputData.accel_B[Y_AXIS]    = (real)accels[Y_AXIS];
    gEKFInputData.accel_B[Z_AXIS]    = (real)accels[Z_AXIS];

    // Angular-rate signal is in [rad/s]
    gEKFInputData.angRate_B[X_AXIS]  = (real)rates[X_AXIS];
    gEKFInputData.angRate_B[Y_AXIS]  = (real)rates[Y_AXIS];
    gEKFInputData.angRate_B[Z_AXIS]  = (real)rates[Z_AXIS];

    // Magnetometer signal is in [G]
    gEKFInputData.magField_B[X_AXIS] = (real)mags[X_AXIS];
    gEKFInputData.magField_B[Y_AXIS] = (real)mags[Y_AXIS];
    gEKFInputData.magField_B[Z_AXIS] = (real)mags[Z_AXIS];
    real tmp[2];
    tmp[X_AXIS] = gEKFInputData.magField_B[X_AXIS] - gMagAlign.hardIronBias[X_AXIS];
    tmp[Y_AXIS] = gEKFInputData.magField_B[Y_AXIS] - gMagAlign.hardIronBias[Y_AXIS];
    gEKFInputData.magField_B[X_AXIS] = gMagAlign.SF[0] * tmp[X_AXIS] + gMagAlign.SF[1] * tmp[Y_AXIS];
    gEKFInputData.magField_B[Y_AXIS] = gMagAlign.SF[2] * tmp[X_AXIS] + gMagAlign.SF[3] * tmp[Y_AXIS];

    // ----- Input from the GPS goes here -----
    // Validity data
    gEKFInputData.gpsValid   = (BOOL)gps->gpsValid;
    gEKFInputData.gpsUpdate = gps->gpsUpdate;

    // Lat/Lon/Alt data
    gEKFInputData.llaRad[LAT] = gps->latitude * D2R;
    gEKFInputData.llaRad[LON] = gps->longitude * D2R;
    gEKFInputData.llaRad[ALT] = gps->altitude;

    // Velocity data
    gEKFInputData.vNed[X_AXIS] = gps->vNed[X_AXIS];
    gEKFInputData.vNed[Y_AXIS] = gps->vNed[Y_AXIS];
    gEKFInputData.vNed[Z_AXIS] = gps->vNed[Z_AXIS];

    // Course and velocity data
    gEKFInputData.rawGroundSpeed = gps->rawGroundSpeed;
    gEKFInputData.trueCourse     = gps->trueCourse;
    
    // ITOW data
    gEKFInputData.itow = gps->itow;

    // Data quality measures
    gEKFInputData.GPSHorizAcc = gps->GPSHorizAcc;
    gEKFInputData.GPSVertAcc  = gps->GPSVertAcc;
    gEKFInputData.HDOP        = gps->HDOP;
}


// Populate the EKF output structure with algorithm results
void EKF_SetOutputStruct(void)
{
    // ------------------ States ------------------

    // Position in [m]
    gEKFOutputData.position_N[X_AXIS] = gKalmanFilter.Position_N[X_AXIS];
    gEKFOutputData.position_N[Y_AXIS] = gKalmanFilter.Position_N[Y_AXIS];
    gEKFOutputData.position_N[Z_AXIS] = gKalmanFilter.Position_N[Z_AXIS];

    // Velocity in [m/s]
    gEKFOutputData.velocity_N[X_AXIS] = gKalmanFilter.Velocity_N[X_AXIS];
    gEKFOutputData.velocity_N[Y_AXIS] = gKalmanFilter.Velocity_N[Y_AXIS];
    gEKFOutputData.velocity_N[Z_AXIS] = gKalmanFilter.Velocity_N[Z_AXIS];

    // Position in [N/A]
    gEKFOutputData.quaternion_BinN[Q0] = gKalmanFilter.quaternion[Q0];
    gEKFOutputData.quaternion_BinN[Q1] = gKalmanFilter.quaternion[Q1];
    gEKFOutputData.quaternion_BinN[Q2] = gKalmanFilter.quaternion[Q2];
    gEKFOutputData.quaternion_BinN[Q3] = gKalmanFilter.quaternion[Q3];

    // Angular-rate bias in [deg/sec]
    gEKFOutputData.angRateBias_B[X_AXIS] = gKalmanFilter.rateBias_B[X_AXIS] * RAD_TO_DEG;
    gEKFOutputData.angRateBias_B[Y_AXIS] = gKalmanFilter.rateBias_B[Y_AXIS] * RAD_TO_DEG;
    gEKFOutputData.angRateBias_B[Z_AXIS] = gKalmanFilter.rateBias_B[Z_AXIS] * RAD_TO_DEG;

    // Acceleration-bias in [m/s^2]
    gEKFOutputData.accelBias_B[X_AXIS] = gKalmanFilter.accelBias_B[X_AXIS];
    gEKFOutputData.accelBias_B[Y_AXIS] = gKalmanFilter.accelBias_B[Y_AXIS];
    gEKFOutputData.accelBias_B[Z_AXIS] = gKalmanFilter.accelBias_B[Z_AXIS];

    // ------------------ Derived variables ------------------

    // Euler-angles in [deg]
    gEKFOutputData.eulerAngs_BinN[ROLL]  = gKalmanFilter.eulerAngles[ROLL] * RAD_TO_DEG;
    gEKFOutputData.eulerAngs_BinN[PITCH] = gKalmanFilter.eulerAngles[PITCH] * RAD_TO_DEG;
    gEKFOutputData.eulerAngs_BinN[YAW]   = gKalmanFilter.eulerAngles[YAW] * RAD_TO_DEG;

    // Angular-rate in [deg/s]
    gEKFOutputData.corrAngRates_B[X_AXIS] = ( gEKFInputData.angRate_B[X_AXIS] -
                                              gKalmanFilter.rateBias_B[X_AXIS] ) * RAD_TO_DEG;
    gEKFOutputData.corrAngRates_B[Y_AXIS] = ( gEKFInputData.angRate_B[Y_AXIS] -
                                              gKalmanFilter.rateBias_B[Y_AXIS] ) * RAD_TO_DEG;
    gEKFOutputData.corrAngRates_B[Z_AXIS] = ( gEKFInputData.angRate_B[Z_AXIS] -
                                              gKalmanFilter.rateBias_B[Z_AXIS] ) * RAD_TO_DEG;

    // Acceleration in [m/s^2]
    gEKFOutputData.corrAccel_B[X_AXIS] = gEKFInputData.accel_B[X_AXIS] * GRAVITY -
                                            gKalmanFilter.accelBias_B[X_AXIS];
    gEKFOutputData.corrAccel_B[Y_AXIS] = gEKFInputData.accel_B[Y_AXIS] * GRAVITY -
                                            gKalmanFilter.accelBias_B[Y_AXIS];
    gEKFOutputData.corrAccel_B[Z_AXIS] = gEKFInputData.accel_B[Z_AXIS] * GRAVITY -
                                            gKalmanFilter.accelBias_B[Z_AXIS];


    // ------------------ Algorithm flags ------------------
    gEKFOutputData.opMode         = gAlgorithm.state;
    gEKFOutputData.linAccelSwitch = gAlgorithm.linAccelSwitch;
    gEKFOutputData.turnSwitchFlag = gAlgoStatus.bit.turnSwitch;
    gEKFOutputData.gpsMeasurementUpdate = gAlgoStatus.bit.gpsUpdate;

    // ------------------ Latitude and Longitude Data ------------------
    gEKFOutputData.llaDeg[LAT] = gKalmanFilter.llaDeg[LAT];
    gEKFOutputData.llaDeg[LON] = gKalmanFilter.llaDeg[LON];
    gEKFOutputData.llaDeg[ALT] = gKalmanFilter.llaDeg[ALT];
}

static void EstimateAccelError( real *accel, real *w, real dt, AccelStatsStruct *gAccelStats)
{
    static BOOL bIni = false;               // indicate if the procedure is initialized
    static real lastAccel[3];               // accel input of last step
    static real lastGyro[3];                // gyro input of last step
    static float lastEstimatedAccel[3];     // propagated accel of last step
    static uint32_t counter = 0;            // propagation counter
    static uint32_t t[3];
    // initialize
    if ( !bIni )
    {
        bIni = true;
        lastAccel[0] = accel[0];
        lastAccel[1] = accel[1];
        lastAccel[2] = accel[2];
        lastGyro[0] = w[0];
        lastGyro[1] = w[1];
        lastGyro[2] = w[2];
        t[0] = 0;
        t[1] = 0;
        t[2] = 0;
        gAccelStats->accelErr[0] = 0.0;
        gAccelStats->accelErr[1] = 0.0;
        gAccelStats->accelErr[2] = 0.0;
        return;
    }

    /* Using gyro to propagate accel and then to detect accel error can give valid result for a
     * short period of time because the inhere long-term drift of integrating gyro data. 
     * So, after this period of time, a new accel input will be selected.
     * Beside, this method cannot detect long-time smooth linear acceleration. In this case, we
     * can only hope the linear acceleration is large enough to make an obvious diffeerence from
     * the Earth gravity 1g.
     */
    if ( counter==0 )
    {
        lastEstimatedAccel[0] = lastAccel[0];
        lastEstimatedAccel[1] = lastAccel[1];
        lastEstimatedAccel[2] = lastAccel[2];
    }
    counter++;
    if ( counter == gAlgorithm.Limit.linAccelSwitchDelay )
    {
        counter = 0;
    }
    
    // propagate accel using gyro
    //  a(k) = a(k-1) -w x a(k-1)*dt
    real ae[3];
    lastGyro[0] *= -dt;
    lastGyro[1] *= -dt;
    lastGyro[2] *= -dt;
    cross(lastGyro, lastEstimatedAccel, ae);
    ae[0] += lastEstimatedAccel[0];
    ae[1] += lastEstimatedAccel[1];
    ae[2] += lastEstimatedAccel[2];
    
    // save this estimated accel
    lastEstimatedAccel[0] = ae[0];
    lastEstimatedAccel[1] = ae[1];
    lastEstimatedAccel[2] = ae[2];
    
    // err = a(k) - am
    ae[0] -= accel[0];
    ae[1] -= accel[1];
    ae[2] -= accel[2];

    /* If the difference between the propagted accel and the input accel exceeds some threshold,
     * we assume there is linear acceleration and set .accelErr to be a large value (0.1g).
     * If the difference has been within the threshold for a period of time, we start to decrease
     * estimated accel error .accelErr.
     */
    int j;
    gAccelStats->accelErrLimit = false;
    for(j=0; j<3; j++)
    {
        if ( fabs(ae[j]) > 1.0e-2 ) // linear accel detected
        {
            t[j] = 0;
            gAccelStats->accelErr[j] = 0.1f;
        }
        else    // no linear accel detected, start to decrease estimated accel error
        {
            if ( t[j] > gAlgorithm.Limit.linAccelSwitchDelay ) // decrase error  
            {
                gAccelStats->accelErr[j] *= 0.9f;
                gAccelStats->accelErr[j] += 0.1f * ae[j];
            }
            else    // keep previous error value
            {
                t[j]++;
                // gAccelStats->accelErr[j];
            }
        }
        // limit error, not taking effect here since the max accelErr should be 0.1
        if ( gAccelStats->accelErr[j] > 0.5 )
        {
            gAccelStats->accelErr[j] = 0.5;
            gAccelStats->accelErrLimit = true;
        }
        if ( gAccelStats->accelErr[j] < -0.5 )
        {
            gAccelStats->accelErr[j] = -0.5;
            gAccelStats->accelErrLimit = true;
        }
    }
    // record accel for next step
    lastAccel[0] = accel[0];
    lastAccel[1] = accel[1];
    lastAccel[2] = accel[2];
    lastGyro[0] = w[0];
    lastGyro[1] = w[1];
    lastGyro[2] = w[2];
}

static void ComputeAccelStats( real *accel, AccelStatsStruct *gAccelStats, int iReset )
{
    static BOOL bIni = false;                   // indicate the routine is initialized or not
    static real data[3][DATA_NUM_FOR_STATS];    // a section in memoty to store buffer data
    static Buffer bf;                           // a ring buffer using the above memory section
    
    // reset the calculation of motion stats
    if ( iReset )
    {
        bIni = false;
    }

    // initialization
    if ( !bIni )
    {
        bIni = true;
        // reset stats
        gAccelStats->bValid = false;
        gAccelStats->accelMean[0] = 0.0;
        gAccelStats->accelMean[1] = 0.0;
        gAccelStats->accelMean[2] = 0.0;
        gAccelStats->accelVar[0] = 0.0;
        gAccelStats->accelVar[1] = 0.0;
        gAccelStats->accelVar[2] = 0.0;
        // create/reset buffer
        bfNew(&bf, &data[0][0], 3, DATA_NUM_FOR_STATS);
    }

    /* Compute mean/variance from input data.
     * When the input data buffer is full, the stats can be assumed valid.
     */
    // save last stats for recursive calculation
    real lastMean[3];
    real lastVar[3];
    lastMean[0] = gAccelStats->accelMean[0];
    lastMean[1] = gAccelStats->accelMean[1];
    lastMean[2] = gAccelStats->accelMean[2];
    lastVar[0] = gAccelStats->accelVar[0];
    lastVar[1] = gAccelStats->accelVar[1];
    lastVar[2] = gAccelStats->accelVar[2];
    if ( bf.full )
    {
        // Enough data collected, stats can be assumed valid
        gAccelStats->bValid = true;
        /* when buffer is full, the var and mean are computed from
         * all data in the buffer. From now on, the var and mean
         * should be computed by removing the oldest data and including
         * the latest data.
         */
        // Get the oldest data which will be removed by following bfPut
        real oldest[3];
        bfGet(&bf, oldest, bf.num-1);
        // put this accel into buffer
        bfPut(&bf, accel);
        // mean(n+1) = ( mean(n) * n - x(1) + x(n+1) ) / n
        gAccelStats->accelMean[0] = lastMean[0] + ( accel[0] - oldest[0] ) / (real)(bf.num);
        gAccelStats->accelMean[1] = lastMean[1] + ( accel[1] - oldest[1] ) / (real)(bf.num);
        gAccelStats->accelMean[2] = lastMean[2] + ( accel[2] - oldest[2] ) / (real)(bf.num);
        
        // naive var calculation is adopted because recursive method is numerically instable
        real tmpVar[3];
        tmpVar[0] = vecVar(&bf.d[0], gAccelStats->accelMean[0], bf.num);
        tmpVar[1] = vecVar(&bf.d[bf.n], gAccelStats->accelMean[1], bf.num);
        tmpVar[2] = vecVar(&bf.d[2*bf.n], gAccelStats->accelMean[2], bf.num);
        // make var estimation smooth
        real k = 0.99f;
        int i;
        for ( i=0; i<3; i++ )
        {
            if ( tmpVar[i] >= gAccelStats->accelVar[i] )
            {
                gAccelStats->accelVar[i] = tmpVar[i];
            }
            else
            {
                gAccelStats->accelVar[i] = k*gAccelStats->accelVar[i] + (1.0f-k)*tmpVar[i];
            }
        }
    }
    else
    {
        // put this accel into buffer
        bfPut(&bf, accel);
        /* Recursivly include new accel. The data num used to compute mean and
         * var are increasing.
         */
        // mean(n+1) = mean(n) *n / (n+1) + x(n+1) / (n+1)
        gAccelStats->accelMean[0] = lastMean[0] + ( accel[0] - lastMean[0] ) / (real)(bf.num);
        gAccelStats->accelMean[1] = lastMean[1] + ( accel[1] - lastMean[1] ) / (real)(bf.num);
        gAccelStats->accelMean[2] = lastMean[2] + ( accel[2] - lastMean[2] ) / (real)(bf.num);
        gAccelStats->accelVar[0] = lastVar[0] + lastMean[0]*lastMean[0] -
                            gAccelStats->accelMean[0] * gAccelStats->accelMean[0] +
                            ( accel[0]*accel[0] - lastVar[0] -lastMean[0]*lastMean[0] ) / (real)(bf.num);
        gAccelStats->accelVar[1] = lastVar[1] + lastMean[1]*lastMean[1] -
                            gAccelStats->accelMean[1] * gAccelStats->accelMean[1] +
                            ( accel[1]*accel[1] - lastVar[1] -lastMean[1]*lastMean[1] ) / (real)(bf.num);
        gAccelStats->accelVar[2] = lastVar[2] + lastMean[2]*lastMean[2] -
                            gAccelStats->accelMean[2] * gAccelStats->accelMean[2] +
                            ( accel[2]*accel[2] - lastVar[2] -lastMean[2]*lastMean[2] ) / (real)(bf.num);
    }

}

static BOOL DetectMotionFromAccel(real accelNorm, int iReset)
{
    if( iReset )
    {
        gAlgorithm.linAccelSwitch = false;
        gAlgorithm.linAccelSwitchCntr = 0;
    }
    /* Check for times when the acceleration is 'close' to 1 [g].  When thisoccurs,
     * increment a counter.  When it exceeds a threshold (indicating that the system
     * has been at rest for a given period) then decrease the R-values (in the
     * update stage of the EKF), effectively increasing the Kalman gain.
     */
    if (fabs( 1.0 - accelNorm ) < gAlgorithm.Limit.accelSwitch )
    {
        gAlgorithm.linAccelSwitchCntr++;
        if ( gAlgorithm.linAccelSwitchCntr >= gAlgorithm.Limit.linAccelSwitchDelay )
        {
            gAlgorithm.linAccelSwitch = TRUE;
        }
        else
        {
            gAlgorithm.linAccelSwitch = FALSE;
        }
    }
    else
    {
        gAlgorithm.linAccelSwitchCntr = 0;
        gAlgorithm.linAccelSwitch     = FALSE;
    }

    return true;
}

/* Set the accelerometer filter coefficients, which are used to filter the 
 * accelerometer readings prior to determining the setting of the linear-
 * acceleration switch and computing the roll and pitch from accelerometer
 * readings.
 */
static void _PopulateFilterCoefficients( uint8_t lpfType, uint8_t callingFreq, real *b, real *a )
{
    switch( lpfType ) 
    {
        case NO_LPF:
            b[0] = (real)(1.0);
            b[1] = (real)(0.0);
            b[2] = (real)(0.0);
            b[3] = (real)(0.0);

            a[0] = (real)(0.0);
            a[1] = (real)(0.0);
            a[2] = (real)(0.0);
            a[3] = (real)(0.0);
            break;
        case TWO_HZ_LPF:
            if( callingFreq == FREQ_100_HZ) 
            {
                b[0] = (real)(2.19606211225382e-4);
                b[1] = (real)(6.58818633676145e-4);
                b[2] = (real)(6.58818633676145e-4);
                b[3] = (real)(2.19606211225382e-4);

                a[0] = (real)( 1.000000000000000);
                a[1] = (real)(-2.748835809214676);
                a[2] = (real)( 2.528231219142559);
                a[3] = (real)(-0.777638560238080);
            } 
            else 
            {
                b[0] = (real)(2.91464944656705e-5);
                b[1] = (real)(8.74394833970116e-5);
                b[2] = (real)(8.74394833970116e-5);
                b[3] = (real)(2.91464944656705e-5);

                a[0] = (real)( 1.000000000000000);
                a[1] = (real)(-2.874356892677485);
                a[2] = (real)( 2.756483195225695);
                a[3] = (real)(-0.881893130592486);
            }
            break;
        case FIVE_HZ_LPF:
            if( callingFreq == FREQ_100_HZ) 
            {
                b[0] = (real)( 0.002898194633721);
                b[1] = (real)( 0.008694583901164);
                b[2] = (real)( 0.008694583901164);
                b[3] = (real)( 0.002898194633721);

                a[0] = (real)( 1.000000000000000);
                a[1] = (real)(-2.374094743709352);
                a[2] = (real)( 1.929355669091215);
                a[3] = (real)(-0.532075368312092);
            } 
            else 
            {
                b[0] = (real)( 0.000416546139076);
                b[1] = (real)( 0.001249638417227);
                b[2] = (real)( 0.001249638417227);
                b[3] = (real)( 0.000416546139076);

                a[0] = (real)( 1.000000000000000);
                a[1] = (real)(-2.686157396548143);
                a[2] = (real)( 2.419655110966473);
                a[3] = (real)(-0.730165345305723);
            }
            break;
        case TWENTY_HZ_LPF:
            if (callingFreq == FREQ_100_HZ) 
            {
                // [B,A] = butter(3,20/(100/2))
                b[0] = (real)( 0.098531160923927);
                b[1] = (real)( 0.295593482771781);
                b[2] = (real)( 0.295593482771781);
                b[3] = (real)( 0.098531160923927);

                a[0] = (real)( 1.000000000000000);
                a[1] = (real)(-0.577240524806303);
                a[2] = (real)( 0.421787048689562);
                a[3] = (real)(-0.056297236491843);
            } 
            else 
            {
                // [B,A] = butter(3,20/(200/2))
                b[0] = (real)( 0.018098933007514);
                b[1] = (real)( 0.054296799022543);
                b[2] = (real)( 0.054296799022543);
                b[3] = (real)( 0.018098933007514);
                
                a[0] = (real)( 1.000000000000000);
                a[1] = (real)(-1.760041880343169);
                a[2] = (real)( 1.182893262037831);
                a[3] = (real)(-0.278059917634546);
            }
            break;
        case TWENTY_FIVE_HZ_LPF:
            if (callingFreq == FREQ_100_HZ) 
            {
                b[0] = (real)( 0.166666666666667);
                b[1] = (real)( 0.500000000000000);
                b[2] = (real)( 0.500000000000000);
                b[3] = (real)( 0.166666666666667);

                a[0] = (real)( 1.000000000000000);
                a[1] = (real)(-0.000000000000000);
                a[2] = (real)( 0.333333333333333);
                a[3] = (real)(-0.000000000000000);
            } 
            else 
            {
                b[0] = (real)( 0.031689343849711);
                b[1] = (real)( 0.095068031549133);
                b[2] = (real)( 0.095068031549133);
                b[3] = (real)( 0.031689343849711);

                a[0] = (real)( 1.000000000000000);
                a[1] = (real)(-1.459029062228061);
                a[2] = (real)( 0.910369000290069);
                a[3] = (real)(-0.197825187264319);
            }
            break;
        case TEN_HZ_LPF:
        default:
            if( callingFreq == FREQ_100_HZ) 
            {
                b[0] = (real)( 0.0180989330075144);
                b[1] = (real)( 0.0542967990225433);
                b[2] = (real)( 0.0542967990225433);
                b[3] = (real)( 0.0180989330075144);

                a[0] = (real)( 1.0000000000000000);
                a[1] = (real)(-1.7600418803431690);
                a[2] = (real)( 1.1828932620378310);
                a[3] = (real)(-0.2780599176345460);
            } 
            else 
            {
                b[0] = (real)( 0.002898194633721);
                b[1] = (real)( 0.008694583901164);
                b[2] = (real)( 0.008694583901164);
                b[3] = (real)( 0.002898194633721);

                a[0] = (real)( 1.000000000000000);
                a[1] = (real)(-2.374094743709352);
                a[2] = (real)( 1.929355669091215);
                a[3] = (real)(-0.532075368312092);
            }
            break;
    }
}

static void _AccelLPF( real *accel, uint8_t lpfType, uint8_t callingFreq, real *filteredAccel )
{
    // Floating-point filter variables
    static real accelFilt[FILTER_ORDER+1][NUM_AXIS];
    static real accelReading[FILTER_ORDER+1][NUM_AXIS];

    // filter coefficients. y/x = b/a
    static real b_AccelFilt[FILTER_ORDER+1];
    static real a_AccelFilt[FILTER_ORDER+1];

    // Compute filter coefficients and initialize data buffer
    static BOOL initAccelFilt = true;
    if (initAccelFilt) 
    {
        initAccelFilt = false;

        // Set the filter coefficients based on selected cutoff frequency and sampling rate
        _PopulateFilterCoefficients( lpfType, callingFreq, b_AccelFilt, a_AccelFilt);

        // Initialize the filter variables (do not need to populate the 0th element
        //   as it is never used)
        for( int i = PASTx1; i <= PASTx3; i++ ) 
        {
            accelReading[i][X_AXIS] = accel[X_AXIS];
            accelReading[i][Y_AXIS] = accel[Y_AXIS];
            accelReading[i][Z_AXIS] = accel[Z_AXIS];

            accelFilt[i][X_AXIS] = accel[X_AXIS];
            accelFilt[i][Y_AXIS] = accel[Y_AXIS];
            accelFilt[i][Z_AXIS] = accel[Z_AXIS];
        }
    }

    /* Filter accelerometer readings (Note: a[0] =  1.0 and the filter coefficients are symmetric)
     * y = filtered output; x = raw input;
     */
    /* a[0]*y(k) + a[1]*y(k-1) + a[2]*y(k-2) + a[3]*y(k-3) =
     * b[0]*x(k) + b[1]*x(k-1) + b[2]*x(k-2) + b[3]*x(k-3) =
     * b[0]*( x(k) + x(k-3) ) + b[1]*( x(k-1) + x(k-2) )
     */
    accelFilt[CURRENT][X_AXIS] = b_AccelFilt[CURRENT] * accel[X_AXIS] +
                                 b_AccelFilt[PASTx1] * ( accelReading[PASTx1][X_AXIS] +
                                                         accelReading[PASTx2][X_AXIS] ) +
                                 b_AccelFilt[PASTx3] * accelReading[PASTx3][X_AXIS] -
                                 a_AccelFilt[PASTx1] * accelFilt[PASTx1][X_AXIS] -
                                 a_AccelFilt[PASTx2] * accelFilt[PASTx2][X_AXIS] -
                                 a_AccelFilt[PASTx3] * accelFilt[PASTx3][X_AXIS];
    accelFilt[CURRENT][Y_AXIS] = b_AccelFilt[CURRENT] * accel[Y_AXIS] +
                                 b_AccelFilt[PASTx1] * ( accelReading[PASTx1][Y_AXIS] +
                                                         accelReading[PASTx2][Y_AXIS] ) +
                                 b_AccelFilt[PASTx3] * accelReading[PASTx3][Y_AXIS] -
                                 a_AccelFilt[PASTx1] * accelFilt[PASTx1][Y_AXIS] -
                                 a_AccelFilt[PASTx2] * accelFilt[PASTx2][Y_AXIS] -
                                 a_AccelFilt[PASTx3] * accelFilt[PASTx3][Y_AXIS];
    accelFilt[CURRENT][Z_AXIS] = b_AccelFilt[CURRENT] * accel[Z_AXIS] +
                                 b_AccelFilt[PASTx1] * ( accelReading[PASTx1][Z_AXIS] +
                                                         accelReading[PASTx2][Z_AXIS] ) +
                                 b_AccelFilt[PASTx3] * accelReading[PASTx3][Z_AXIS] -
                                 a_AccelFilt[PASTx1] * accelFilt[PASTx1][Z_AXIS] -
                                 a_AccelFilt[PASTx2] * accelFilt[PASTx2][Z_AXIS] -
                                 a_AccelFilt[PASTx3] * accelFilt[PASTx3][Z_AXIS];

    // Update past readings
    accelReading[PASTx3][X_AXIS] = accelReading[PASTx2][X_AXIS];
    accelReading[PASTx2][X_AXIS] = accelReading[PASTx1][X_AXIS];
    accelReading[PASTx1][X_AXIS] = accel[X_AXIS];

    accelReading[PASTx3][Y_AXIS] = accelReading[PASTx2][Y_AXIS];
    accelReading[PASTx2][Y_AXIS] = accelReading[PASTx1][Y_AXIS];
    accelReading[PASTx1][Y_AXIS] = accel[Y_AXIS];

    accelReading[PASTx3][Z_AXIS] = accelReading[PASTx2][Z_AXIS];
    accelReading[PASTx2][Z_AXIS] = accelReading[PASTx1][Z_AXIS];
    accelReading[PASTx1][Z_AXIS] = accel[Z_AXIS];

    accelFilt[PASTx3][X_AXIS] = accelFilt[PASTx2][X_AXIS];
    accelFilt[PASTx2][X_AXIS] = accelFilt[PASTx1][X_AXIS];
    accelFilt[PASTx1][X_AXIS] = accelFilt[CURRENT][X_AXIS];

    accelFilt[PASTx3][Y_AXIS] = accelFilt[PASTx2][Y_AXIS];
    accelFilt[PASTx2][Y_AXIS] = accelFilt[PASTx1][Y_AXIS];
    accelFilt[PASTx1][Y_AXIS] = accelFilt[CURRENT][Y_AXIS];

    accelFilt[PASTx3][Z_AXIS] = accelFilt[PASTx2][Z_AXIS];
    accelFilt[PASTx2][Z_AXIS] = accelFilt[PASTx1][Z_AXIS];
    accelFilt[PASTx1][Z_AXIS] = accelFilt[CURRENT][Z_AXIS];

    // Output filtered accel
    filteredAccel[X_AXIS] = accelFilt[CURRENT][X_AXIS];
    filteredAccel[Y_AXIS] = accelFilt[CURRENT][Y_AXIS];
    filteredAccel[Z_AXIS] = accelFilt[CURRENT][Z_AXIS];
}
