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

#include "algorithmAPI.h"
#include "gpsAPI.h"
#include "platformAPI.h"
#include "userAPI.h"

#include "Indices.h"
#include "GlobalConstants.h"

#include "math.h"
#include "VectorMath.h"


//
#include "algorithm.h"
#include "UserAlgorithm.h"

#include "bsp.h"
#include "debug.h"

// Declare the leveler data structure
#include "Leveler.h"

//
static void _Algorithm(uint16_t dacqRate, uint8_t algoType);
static void _GenerateDebugMessage(uint16_t dacqRate, uint16_t debugOutputFreq);
static void _InitAlgo(uint8_t algoType);

// Initialize leveler algorithm variables
void InitUserAlgorithm()
{
    // Initialize built-in algorithm structure
    Leveler_InitializeAlgorithmStruct(FREQ_200_HZ);

    // place additional required initialization here
}


void *RunUserNavAlgorithm(double *accels, double *rates, double *mags, gpsDataStruct_t *gps, uint16_t dacqRate)
{
    // This can be set at startup based on the packet type selected
    static uint8_t algoType = IMU;

    // Initialize variable related to the UserNavAlgorithm
    _InitAlgo(algoType);

    // Populate the leveler input data structure.  Load the GPS data
    //   structure as NULL.
    Leveler_SetInputStruct(accels, rates, mags, NULL);

    // Call the desired algorithm based on the EKF with different
    //   calling rates and different settings.
    _Algorithm(dacqRate, algoType);

    // Fill the output data structure with the algorithm  states and other 
    //   desired information
    Leveler_SetOutputStruct();

    // Generate a debug message that provides algorithm output to verify the
    //   algorithm is generating the proper output.
    _GenerateDebugMessage(dacqRate, ZERO_HZ);

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
        gAlgorithm.Behavior.bit.stationaryLockYaw  = 0;
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
                gAlgorithm.Behavior.bit.useMag = 1;
                break;
            case INS:
                // Nothing additional to do (already configured for a VG
                //   solution)
                enableMagInAlgorithm(TRUE);
                gAlgorithm.callingFreq = FREQ_100_HZ;  // redundant; set above
                gAlgorithm.Behavior.bit.useMag = 1;
                gAlgorithm.Behavior.bit.useGPS = 1;
                break;
            default:
                // Nothing to do
                break;
        }
    }
}


//
static void _Algorithm(uint16_t dacqRate, uint8_t algoType)
{
    //
    static uint8_t algoCntr = 0, algoCntrLimit = 0;

    // Initialize the configuration variables needed to make the system
    //   generate a VG-type solution.
    static uint8_t initAlgo = 1;
    if(initAlgo) {
        // Reset 'initAlgo' so this is not executed more than once.  This
        //   prevents the algorithm from being switched during run-time.
        initAlgo = 0;

        //  Set the variables that control the algorithm execution rate
        algoCntrLimit = (uint8_t)( (float)dacqRate / (float)gLeveler.callingFreq + 0.5 );
        if( algoCntrLimit < 1 ) {
            // If this logic is reached, also need to adjust the algorithm
            //   parameters to match the modified calling freq (or stop the
            //   program to indicate that the user must adjust the program)
            algoCntrLimit = 1;
        }
        algoCntr = algoCntrLimit;
    }

    // Increment the counter.  If greater than or equal to the limit, reset
    //   the counter to cause the algorithm to run on the next pass through.
    algoCntr++;
    if(algoCntr >= algoCntrLimit) {
        // Reset counter
        algoCntr = 0;

        // Aceinna VG/AHRS/INS algorithm
        Leveler_Algorithm();
    }
}


//
static void _GenerateDebugMessage(uint16_t dacqRate, uint16_t debugOutputFreq)
{
    // Variables that control the output frequency of the debug statement
    static uint8_t debugOutputCntr, debugOutputCntrLimit;

    // Check debug flag.  If set then generate the debug message to verify
    //   the output of the EKF algorithm
    if( debugOutputFreq > ZERO_HZ ) {
        // Initialize variables used to control the output of the debug messages
        static int initFlag = 1;
        if(initFlag) {
            // Reset 'initFlag' so this section is not executed more than once.
            initFlag = 0;

            // Set the variables that control the debug-message output-rate (based on
            //   the desired calling frequency of the debug output)
            debugOutputCntrLimit = (uint8_t)( (float)dacqRate / (float)debugOutputFreq + 0.5 );
            debugOutputCntr      = debugOutputCntrLimit;
        }

        debugOutputCntr++;
        if(debugOutputCntr >= debugOutputCntrLimit) {
            debugOutputCntr = 0;
            DebugPrintFloat("Time: ",    gLeveler.timerCntr * 0.001, 3);
            DebugPrintFloat(", Roll: ",  gLeveler.output.measuredEulerAngles_BinN[ROLL],  5);
            DebugPrintFloat(", Pitch: ", gLeveler.output.measuredEulerAngles_BinN[PITCH], 5);
            DebugPrintEndline();
        }
    }
}
