/******************************************************************************
 * @file algorithm.c
 * @brief Top level algorithm configurations and functions.
 * All top-level algorithm configurations and functions are here, including
 * algorithm state, algorithm configurations, algorithm input and output.
 * @author Dong Xiaoguang
 * @date 2019.05.09
 * @version V1.0.0
 *-----------------------------------------------------------------------------
 * Change History
 * <Date>     | <Version> | <Author>       | <Description>
 * ----------------------------------------------------------------------------
 * 2019.05.09 | v1.0.0    | Dong Xiaoguang | Create file
 * ----------------------------------------------------------------------------
******************************************************************************/

#include <math.h>
#include "SensorNoiseParameters.h"
#include "platformAPI.h"
#include "algorithm.h"
#include "AlgorithmLimits.h"
#include "algorithmAPI.h"

AlgorithmStruct gAlgorithm;
AlgoStatus      gAlgoStatus;


void InitializeAlgorithmStruct(uint8_t callingFreq)
{
    gAlgorithm.Behavior.bit.freeIntegrate = FALSE;
    // The calling frequency drives the execution rate of the EKF and dictates
    //   the algorithm constants
    if(callingFreq == 0){
        // IMU case
        callingFreq = FREQ_200_HZ;
    }
    gAlgorithm.callingFreq = callingFreq;

    // Set dt based on the calling frequency of the EKF
    if (gAlgorithm.callingFreq == FREQ_100_HZ)
    {
        gAlgorithm.dt = (real)(0.01);
        gAlgorithm.dITOW = 10;
    }
    else if (gAlgorithm.callingFreq == FREQ_200_HZ)
    {
        gAlgorithm.dt = (real)(0.005);
        gAlgorithm.dITOW = 5;
    }
    else
    {
        while (1);
    }

    // Set up other timing variables
    gAlgorithm.dtOverTwo = (real)(0.5) * gAlgorithm.dt;
    gAlgorithm.dtSquared = gAlgorithm.dt * gAlgorithm.dt;
    gAlgorithm.sqrtDt = sqrtf(gAlgorithm.dt);

    // Set the algorithm duration periods
    gAlgorithm.Duration.Stabilize_System = (uint32_t)(gAlgorithm.callingFreq * STABILIZE_SYSTEM_DURATION);
    gAlgorithm.Duration.Initialize_Attitude = (uint32_t)(gAlgorithm.callingFreq * INITIALIZE_ATTITUDE_DURATION);
    gAlgorithm.Duration.High_Gain_AHRS = (uint32_t)(gAlgorithm.callingFreq * HIGH_GAIN_AHRS_DURATION);
    gAlgorithm.Duration.Low_Gain_AHRS = (uint32_t)(gAlgorithm.callingFreq * LOW_GAIN_AHRS_DURATION);

    // Set the initial state of the EKF
    gAlgorithm.state = STABILIZE_SYSTEM;
    gAlgorithm.stateTimer = gAlgorithm.Duration.Stabilize_System;

    // Turn-switch variable
    gAlgorithm.filteredYawRate = (real)0.0;

    // Tell the algorithm to apply the declination correction to the heading
    //  (at startup in AHRS, do not apply.  After INS becomes healthy, apply,
    //  even in AHRS, but this condition shouldn't last forever.  Question:
    //  how long to keep this set TRUE after GPS in invalid?)
    gAlgorithm.applyDeclFlag = FALSE;

    gAlgorithm.insFirstTime = TRUE;
    gAlgorithm.headingIni = HEADING_UNINITIALIZED;

    //gAlgorithm.magAlignUnderway = FALSE; // Set and reset in mag-align code

    // Increment at 100 Hz in EKF_Algorithm; sync with GPS itow when valid.
    gAlgorithm.itow = 0;

    // Limit is compared to ITOW.  Time must be in [msec].
    gAlgorithm.Limit.maxGpsDropTime = LIMIT_MAX_GPS_DROP_TIME * 1000;
    gAlgorithm.Limit.maxReliableDRTime = LIMIT_RELIABLE_DR_TIME * 1000;

    // Limit is compared to count (incremented upon loop through
    //   taskDataAcquisition).  Time must be in [count] based on ODR.
    gAlgorithm.Limit.Free_Integration_Cntr = gAlgorithm.callingFreq * LIMIT_FREE_INTEGRATION_CNTR;

    // Linear acceleration switch limits (level and time)
    gAlgorithm.Limit.accelSwitch = (real)(0.012);   // [g]
    gAlgorithm.Limit.linAccelSwitchDelay = (uint32_t)(2.0 * gAlgorithm.callingFreq);

    // Innovation error limits for EKF states
    gAlgorithm.Limit.Innov.positionError = (real)270.0;
    gAlgorithm.Limit.Innov.velocityError = (real)27.0;
    gAlgorithm.Limit.Innov.attitudeError = (real)SIX_DEGREES_IN_RAD;
    
    // Five-hertz LPF (corresponding integer value found in PredictFunctions.c)
    // Replace with a function that computes the coefficients.  Value (below) will
    //   then be the cutoff frequency.
    gAlgorithm.linAccelLPFType = 1;

    // Uing raw accel to detect linear acceleration has lower failure rate in small
    //  and smooth linear acceleration. But on some platform, there is large vibration,
    //  uing raw accel to detect linear acceleration will always detect linear accel.
    gAlgorithm.useRawAccToDetectLinAccel = TRUE;

    // Set the turn-switch threshold to a default value in [deg/sec]
    gAlgorithm.turnSwitchThreshold = 6.0;

	// default lever arm and point of interest
	gAlgorithm.leverArmB[X_AXIS] = 0.0;
	gAlgorithm.leverArmB[Y_AXIS] = 0.0;
	gAlgorithm.leverArmB[Z_AXIS] = 0.0;
	gAlgorithm.pointOfInterestB[X_AXIS] = 0.0;
	gAlgorithm.pointOfInterestB[Y_AXIS] = 0.0;
	gAlgorithm.pointOfInterestB[Z_AXIS] = 0.0;

    // For most vehicles, the velocity is always along the body x axis
    gAlgorithm.velocityAlwaysAlongBodyX = TRUE;

    // get IMU specifications
    gAlgorithm.imuSpec.arw = (real)ARW_300ZA;
    gAlgorithm.imuSpec.sigmaW = (real)(1.25 * ARW_300ZA / sqrt(1.0/RW_ODR));
    gAlgorithm.imuSpec.biW = (real)BIW_300ZA;
    gAlgorithm.imuSpec.maxBiasW = (real)MAX_BW;
    gAlgorithm.imuSpec.vrw = (real)VRW_300ZA;
    gAlgorithm.imuSpec.sigmaA = (real)(1.25 * VRW_300ZA / sqrt(1.0/RW_ODR));
    gAlgorithm.imuSpec.biA = (real)BIA_300ZA;
    gAlgorithm.imuSpec.maxBiasA = (real)MAX_BA;

    // default noise level multiplier for static detection
    gAlgorithm.staticDetectParam.staticVarGyro = (real)(gAlgorithm.imuSpec.sigmaW * gAlgorithm.imuSpec.sigmaW);
    gAlgorithm.staticDetectParam.staticVarAccel = (real)(gAlgorithm.imuSpec.sigmaA * gAlgorithm.imuSpec.sigmaA);
    gAlgorithm.staticDetectParam.maxGyroBias = MAX_BW;
    gAlgorithm.staticDetectParam.staticGnssVel = 0.2;
    gAlgorithm.staticDetectParam.staticNoiseMultiplier[0] = 4.0;
    gAlgorithm.staticDetectParam.staticNoiseMultiplier[1] = 4.0;
    gAlgorithm.staticDetectParam.staticNoiseMultiplier[2] = 1.0;
}

void GetAlgoStatus(AlgoStatus *algoStatus)
{
    algoStatus->all = gAlgoStatus.all;
}


void setAlgorithmExeFreq(int freq)
{
    gAlgorithm.callingFreq = freq;

}

void updateAlgorithmTimings(int corr, uint32_t tmrVal )
{
    // Set the counter to a value that corresponds to seconds after TIM2 is
    //   called and just after the sensors are read.
    gAlgorithm.counter = (uint16_t)( 1.25e-3 * ( ( corr + (uint16_t)(1.334489891239070E-05 * tmrVal) ) << 16 ) );
    // Increment the timer output value (FIXME: is this in the right spot?
    //   Should it be in the algorithm if-statement below?)
    gAlgorithm.timer = gAlgorithm.timer + gAlgorithm.dITOW;
}

uint32_t getAlgorithmTimer()
{
    return gAlgorithm.timer;
}

uint16_t getAlgorithmCounter()
{
    return gAlgorithm.counter;
}

uint16_t getAlgorithmFrequency()
{
    return gAlgorithm.callingFreq;
}

uint32_t getAlgorithmITOW()
{
    return gAlgorithm.itow;
}

void setLeverArm( real leverArmBx, real leverArmBy, real leverArmBz )
{
    gAlgorithm.leverArmB[0] = leverArmBx;
    gAlgorithm.leverArmB[1] = leverArmBy;
    gAlgorithm.leverArmB[2] = leverArmBz;
}

void setPointOfInterest( real poiBx, real poiBy, real poiBz )
{
    gAlgorithm.pointOfInterestB[0] = poiBx;
    gAlgorithm.pointOfInterestB[1] = poiBy;
    gAlgorithm.pointOfInterestB[2] = poiBz;
}