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

//
static void _Algorithm();
static void _InitAlgo(uint8_t algoType);

// Initialize GPS algorithm variables
void InitUserAlgorithm()
{
    // Initialize built-in algorithm structure
    InitializeAlgorithmStruct(FREQ_100_HZ);

    // place additional required initialization here
    setLeverArm( (real)gUserConfiguration.leverArmBx,
                 (real)gUserConfiguration.leverArmBy,
                 (real)gUserConfiguration.leverArmBz );
    setPointOfInterest( (real)gUserConfiguration.pointOfInterestBx,
                        (real)gUserConfiguration.pointOfInterestBy,
                        (real)gUserConfiguration.pointOfInterestBz );
}


void *RunUserNavAlgorithm(double *accels, double *rates, double *mags,
                          gpsDataStruct_t *gps, odoDataStruct_t *odo, BOOL ppsDetected)
{
    // This can be set at startup based on the packet type selected
    static uint8_t algoType = INS;

    // Initialize variable related to the UserNavAlgorithm
    _InitAlgo(algoType);

    // Populate the EKF input data structure
    EKF_SetInputStruct(accels, rates, mags, gps, odo, ppsDetected);

    // Call the desired algorithm based on the EKF with different
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
static void _InitAlgo(uint8_t algoType)
{
    // Initialize the timer variables
    static uint8_t initAlgo = 1;
    if(initAlgo) {
        // Reset 'initAlgo' so this is not executed more than once.  This
        //   prevents the algorithm from being switched during run-time.
        initAlgo = 0;
        
        // Set the configuration variables for a VG-type solution
        //   (useMags = 0 forces the VG solution)
        gAlgorithm.Behavior.bit.freeIntegrate      = 0;
        gAlgorithm.Behavior.bit.useMag             = 0;
        gAlgorithm.Behavior.bit.useGPS             = 0;
        gAlgorithm.Behavior.bit.useOdo             = 0;
        gAlgorithm.Behavior.bit.restartOnOverRange = 0;
        gAlgorithm.Behavior.bit.dynamicMotion      = 1;

        // Set the system configuration based on system type
        switch( algoType ) {
            case VG:
                // Nothing additional to do (already configured for a VG
                //   solution)
                break;
            case AHRS:
                // Set the configuration variables for AHRS solution
                //   (useMags = 1 and enable mags)
                enableMagInAlgorithm(TRUE);
                break;
            case INS:
                /* Set the configuration variables for INS solution.
                 * (Enable GPS and set algorithm calling frequency to 100Hz)
                 */
                enableMagInAlgorithm(FALSE);
                enableGpsInAlgorithm(TRUE);
                enableOdoInAlgorithm(FALSE);
                gAlgorithm.callingFreq = FREQ_100_HZ;  // redundant; set above
                break;
            default:
                // Nothing to do
                break;
        }
    }
}


//
static void _Algorithm()
{
    // Aceinna VG/AHRS/INS algorithm
    EKF_Algorithm();
}
