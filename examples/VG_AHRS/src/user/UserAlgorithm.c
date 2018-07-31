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

#include "userAPI.h"
#include "gpsAPI.h"
#include "stddef.h"
#include "GlobalConstants.h"
#include "gpsAPI.h"
#include "algorithmAPI.h"

extern void InitializeAlgorithmStruct(void);
#include "algorithm.h"
#include "EKF_Algorithm.h"
#include "platformAPI.h"
#include "BitStatus.h"


//
void _Algorithm(int dacqRate, uint8_t algoType);

void InitUserAlgorithm()
{
    // Initialize built-in algorithm structure
    setAlgorithmExeFreq(FREQ_200_HZ);
    InitializeAlgorithmStruct();
    // place additional required initialization here
}

#include "bsp.h"
#include "debug.h"

void *RunUserNavAlgorithm(double *accels, double *rates, double *mags, gpsDataStruct_t *gps, int dacqRate)
{
    // Initialization variable
    static int initAlgo  = 1;

    // Variables that control the output frequency of the debug statement
    static int debugFlag = 1;
    static uint8_t debugOutputCntr, debugOutputCntrLimit;

    // Initialize the timer variables
    if(initAlgo) {
        // Reset 'initAlgo' so this is not executed more than once.
        initAlgo = 0;

        // Set the variables that control the debug-message output-rate (based on
        //   the desired calling frequency of the debug output)
        debugOutputCntr = 0;

        uint16_t debugOutputFreq = 5;  // [Hz]
        debugOutputCntrLimit = (int)( (float)dacqRate / (float)debugOutputFreq + 0.5 );
    }

    // Populate the EKF input data structure.  Load the GPS data
    //   structure as NULL.
    EKF_SetInputStruct(accels, rates, mags, NULL);

    // Call the desired algorithm based on the EKF with different
    //   calling rates and different settings.
    _Algorithm(dacqRate, VG);

    // Fill the output data structure with the EKF states and other 
    //   desired information
    EKF_SetOutputStruct();

    // Generate the debug output
    if( debugFlag ) {
        debugOutputCntr++;
        if(debugOutputCntr >= debugOutputCntrLimit) {
            debugOutputCntr = 0;
            DebugPrintFloat("Roll: ",    gEKFOutputData.eulerAngs_BinN[ROLL], 5);
            DebugPrintFloat(", Pitch: ", gEKFOutputData.eulerAngs_BinN[PITCH], 5);
            DebugPrintFloat(", aX: ", gEKFInputData.accel_B[X_AXIS], 5);
            DebugPrintFloat(", aY: ", gEKFInputData.accel_B[Y_AXIS], 5);
            DebugPrintFloat(", aZ: ", gEKFInputData.accel_B[Z_AXIS], 5);
            DebugPrintInt(", Mode: ", gEKFOutputData.opMode);
            DebugPrintEndline();
        }
    }

    // The returned value from this function is unused by external functions.  The
    //   NULL pointer is returned instead of a data structure.
    return NULL;
}


//
void _Algorithm(int dacqRate, uint8_t algoType)
{
    //
    static int initAlgo = 1;
    static uint8_t algoCntr = 0, algoCntrLimit = 0;

    // Initialize the configuration variables needed to make the system
    //   generate a VG-type solution.
    if(initAlgo) {
        // Reset 'initAlgo' so this is not executed more than once.  This
        //   prevents the algorithm from being switched during run-time.
        initAlgo = 0;

        // Set the configuration variables for a VG-type solution
        //   (useMags = 0 forces the VG solution)
        gAlgorithm.Behavior.bit.freeIntegrate      = 0;
        gAlgorithm.Behavior.bit.useMag             = 0;
        gAlgorithm.Behavior.bit.useGPS             = 0;
        gAlgorithm.Behavior.bit.stationaryLockYaw  = 0;
        gAlgorithm.Behavior.bit.restartOnOverRange = 0;
        gAlgorithm.Behavior.bit.dynamicMotion      = 1;

        // While not needed, set hasMags to false
        enableMagInAlgorithm(FALSE);

        if(algoType == VG) {
            // Configuration already set for a VG solution
        } else if(algoType == AHRS) {
            // Set the configuration variables for AHRS solution
            //   (useMags = 1 and enable mags)
            enableMagInAlgorithm(TRUE);
            gAlgorithm.Behavior.bit.useMag = 1;
        } else if(algoType == INS) {
            while(1);
        } else {
            while(1);
        }

        algoCntr = 0;
        algoCntrLimit = (int)( dacqRate / (int)gAlgorithm.callingFreq );
        if( algoCntrLimit < 1 ) {
            // If this logic is reached, also need to adjust the algorithm
            //   parameters to match the modified calling freq (or stop the
            //   program to indicate that the user must adjust the program)
            algoCntrLimit = 1;
        }
    }

    // Aceinna VG/AHRS/INS algorithm
    if(algoCntr == 0) {
        EKF_Algorithm();
    }

    // Increment the counter.  If greater than or equal to the limit, reset
    //   the counter to cause the algorithm to run on the next pass through.
    algoCntr++;
    if(algoCntr >= algoCntrLimit) {
        algoCntr = 0;
    }
}

