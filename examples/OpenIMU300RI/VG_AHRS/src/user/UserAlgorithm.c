/** ***************************************************************************
 * @file   UserAlgorithm.c
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
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

#include <stddef.h>
#include <string.h>

#include "platformAPI.h"
#include "userAPI.h"

#include "Indices.h"
#include "GlobalConstants.h"

#if FAST_MATH
#include "arm_math.h"
#endif

#include "UserConfiguration.h"
#include "algorithmAPI.h"
#include "algorithm.h"
#include "UserAlgorithm.h"
#include "EKF_Algorithm.h"

#ifndef INS_OFFLINE
#include "debug.h"
#endif // !INS_OFFLINE

// Declare the IMU data structure
IMUDataStruct   gIMU;
gpsDataStruct_t gGPS;
odoDataStruct_t gOdo;

int initAlgo = 0;; 

//
static void _Algorithm();

// Initialize algorithm variables
void InitUserAlgorithm()
{
    // Initialize built-in algorithm structure
    InitializeAlgorithmStruct(FREQ_200_HZ, OpenIMU300RI);

    // place additional required initialization here
    /* Set the configuration variables for AHRS solution
     * Enable mag
     */
#ifdef AHRS_ALGORITHM
    enableMagInAlgorithm(TRUE);
#endif
}


void *RunUserNavAlgorithm(double *accels, double *rates, double *mags,
                          gpsDataStruct_t *gps, odoDataStruct_t *odo, BOOL ppsDetected)
{

    // Populate the EKF input data structure
    EKF_SetInputStruct(accels, rates, mags, gps, odo, ppsDetected);

    //   Call the desired algorithm based on the EKF with different
    //   calling rates and different settings.
    _Algorithm();

    // Fill the output data structure with the EKF states and other 
    //   desired information
    EKF_SetOutputStruct();

    // The returned value from this function is unused by external functions.  The
    //   NULL pointer is returned instead of a data structure.
    return NULL;
}

//
static void _Algorithm()
{
    // Aceinna VG/AHRS/INS algorithm
        EKF_Algorithm();
}


BOOL   getAlgorithmLinAccelDetectMode()
{
    return TRUE;  
}

BOOL   getAlgorithmAccelPredictMode()
{
    return FALSE;
}

float   getAlgorithmCoefOfReduceQ()
{
    // 0.0001 to 1 (1 to  10000)
    return (float)10/10000;
}

float   getAlgorithmAccelSwitchDelay()
{
    // 0.01 to 10 (100 to 10000)
    return (float)2000/1000;
}     

float   getAlgorithmRateIntegrationTime()      
{
    // 0.01 to 10 (100 to 10000)
    return (float)2000/1000;
}     
