/*
* File:   xbowsp_algorithm.c
* Author: t. malerich
*
* Created on 2016-06-06
*
* Purpose is to provide the functions to initialize the AlgorithmStruct
*/

#include "xbowsp_algorithm.h"    // gAlgorithm
#include "GlobalConstants.h"     // TRUE, FALSE, etc

#include "ucb_packet.h"   // for UcbGetSysRange() and _200_DPS_RANGE
#include "scaling.h"      // DEGREES_TO_RADS

#include "AlgorithmLimits.h"
#include "TimingVars.h"

#include <math.h>

#ifdef INS_OFFLINE
#include "C:\Projects\software\sim\INS380_Offline\INS380_Offline\SimulationParameters.h"
#endif

/* ----------------------------------------------------------------------------
* @name InitializeAlgorithmStruct
* @brief initializes the values in the AlgorithmStruct
*
* @retval N/A
---------------------------------------------------------------------------- */
void InitializeAlgorithmStruct(AlgorithmStruct *algorithmStruct)
{
    // The calling frequency drives the execution rate of the EKF and dictates
    //   the algorithm constants
    gAlgorithm.callingFreq = timer.odr;

    // Set dt based on the calling frequency of the EKF
    if(algorithmStruct->callingFreq == ODR_100_HZ) {
        algorithmStruct->dt    = (real)(0.01);
        algorithmStruct->dITOW = 10;
    } else if(algorithmStruct->callingFreq == ODR_200_HZ) {
        algorithmStruct->dt    = (real)(0.005);
        algorithmStruct->dITOW = 5;
    } else {
        ;
    }

    // Set the algorithm duration periods
    gAlgorithm.Duration.Stabilize_System    = (uint32_t)(gAlgorithm.callingFreq * STABILIZE_SYSTEM_DURATION);
    gAlgorithm.Duration.Initialize_Attitude = (uint32_t)(gAlgorithm.callingFreq * INITIALIZE_ATTITUDE_DURATION);
    gAlgorithm.Duration.High_Gain_AHRS      = (uint32_t)(gAlgorithm.callingFreq * HIGH_GAIN_AHRS_DURATION);
    gAlgorithm.Duration.Low_Gain_AHRS       = (uint32_t)(gAlgorithm.callingFreq * LOW_GAIN_AHRS_DURATION);

    // Set up other timing variables
    algorithmStruct->dtOverTwo = (real)(0.5) * algorithmStruct->dt;
    algorithmStruct->dtSquared = algorithmStruct->dt * algorithmStruct->dt;
    algorithmStruct->sqrtDt = sqrt(algorithmStruct->dt);

    // Set the initial state of the EKF
    algorithmStruct->state = STABILIZE_SYSTEM;
    //gAlgorithm.stateTimer = gAlgorithm.Duration.Stabilize_System;
    algorithmStruct->stateTimer = gAlgorithm.Duration.Stabilize_System;

    // Turn-switch variable
    algorithmStruct->filteredYawRatePast = (real)0.0;

    // Tell the algorithm to apply the declination correction to the heading
    //  (at startup in AHRS, do not apply.  After INS becomes healthy, apply,
    //  even in AHRS, but this condition shouldn't last forever.  Question:
    //  how long to keep this set TRUE after GPS in invalid?)
    algorithmStruct->applyDeclFlag = FALSE;

    algorithmStruct->insFirstTime = TRUE;

    //algorithmStruct->magAlignUnderway = FALSE; // Set and reset in mag-align code

    // Increment at 100 Hz in EKF_Algorithm; sync with GPS itow when valid.
    algorithmStruct->itow = 0;

    // Limit is compared to ITOW.  Time must be in [msec].
    gAlgorithm.Limit.Max_GPS_Drop_Time     = LIMIT_MAX_GPS_DROP_TIME * 1000;

    // Limit is compared to count (incremented upon loop through
    //   taskDataAcquisition).  Time must be in [count] based on ODR.
    gAlgorithm.Limit.Free_Integration_Cntr = gAlgorithm.callingFreq * LIMIT_FREE_INTEGRATION_CNTR;

// Change in the latest code.  Doesn't match v18.1.10
    gAlgorithm.Limit.accelSwitch   = (real)(0.0080); //0.0055);   // [g]
    gAlgorithm.Limit.linAccelSwitchDelay = (uint32_t)(4.5 * gAlgorithm.callingFreq);
}


// Define the setter and getter functions
AlgorithmType getAlgoType(void)
{
    return gAlgorithm.algorithmType;
}


void setAlgoType( AlgorithmType inType )
{
    gAlgorithm.algorithmType = inType;
}


void InitializeSensorScaling(void)
{
    //
#ifdef INS_OFFLINE
   // This value is set based on the version string specified in the 
   //   simulation configuration file, ekfSim.cfg
    uint8_t sysRange = gSimulation.sysRange;
#else
    // This value is set based on the version string loaded into the unit
    //   via the system configuration load
    uint8_t sysRange = UcbGetSysRange(); // from system config
#endif

    //
    switch (sysRange) {
        case _200_DPS_RANGE: // same as default
            gAlgorithm.Limit.rateAlarm  = (real)(200.0 * DEGREES_TO_RADS);   // [rad/sec]
            gAlgorithm.Limit.accelAlarm = (real)(0.95 * 4.0);   // [g]
            break;

        case _400_DPS_RANGE:
            gAlgorithm.Limit.rateAlarm  = (real)(400.0 * DEGREES_TO_RADS);
            gAlgorithm.Limit.accelAlarm = (real)(0.95 * 8.0);
            break;

        case _1000_DPS_RANGE:
            gAlgorithm.Limit.rateAlarm  = (real)(600.0 * DEGREES_TO_RADS);
            gAlgorithm.Limit.accelAlarm = (real)(0.95 * 8.0);
            break;
    }
    gAlgorithm.Limit.magAlarm   = (real)(4.0);
}

