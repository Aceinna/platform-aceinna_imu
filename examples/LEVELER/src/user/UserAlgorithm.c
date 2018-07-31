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
#include "Indices.h"
#include "GlobalConstants.h"
#include "gpsAPI.h"
#include "algorithmAPI.h"

#include "math.h"

//#include "xbowsp_configuration.h"
#include "algorithm.h"
#include "UserAlgorithm.h"
#include "platformAPI.h"

#include "VectorMath.h"

// Declare the leveler data structure
LevelerDataStruct gLeveler;

// Initialize leveler algorithm variables
void InitUserAlgorithm()
{
    // 
    Leveler_SetExeFreq(FREQ_50_HZ);
    Leveler_InitializeDataStruct();
}

#include "bsp.h"
#include "debug.h"

void *RunUserNavAlgorithm(double *accels_B, double *rates_B, double *mags_B, gpsDataStruct_t *gps, int dacqRate)
{
    // Initialization variable
    static int initAlgo = 1;

    // Measurement variables
    float accelsFloat_B[3], aHat_B[3];
    volatile float gHat_B[3];

    // Variables that control the output frequency of the debug statement
    static uint8_t debugOutputCntr, debugOutputCntrLimit;

    // The following control the execution rate of the algorithm by forcing the
    //   algorithm to run at a fraction of the data-acquisition task
    static uint8_t algoCntr = 0, algoCntrLimit = 0;

    // Initialize the timer variables
    if(initAlgo) {
        // Reset 'initAlgo' so this is not executed more than once.
        initAlgo = 0;

        //  Set the variables that control the algorithm execution rate
        algoCntrLimit = (int)( dacqRate / (int)gLeveler.callingFreq );
        algoCntr = algoCntrLimit;

        // Set the variables that control the debug-message output-rate (based on
        //   the desired calling frequency of the debug output)
        debugOutputCntr = 0;

        uint16_t debugOutputFreq = 2;  // [Hz]
        debugOutputCntrLimit = gLeveler.callingFreq / debugOutputFreq;
    }

    // ------------ Static-leveler algorithm ------------

    // Increment algoCntr.  If greater than or equal to the limit, execute the
    //   algorithm and reset the counter so the algorithm will not run until
    //   the counter limit is once again reached.
    algoCntr++;
    if(algoCntr >= algoCntrLimit) {
        // Reset counter
        algoCntr = 0;

        // Increment the timerCntr
        gLeveler.timerCntr = gLeveler.timerCntr + gLeveler.dTimerCntr;

        // Compute the acceleration unit-vector
        accelsFloat_B[X_AXIS] = (float)accels_B[X_AXIS];
        accelsFloat_B[Y_AXIS] = (float)accels_B[Y_AXIS];
        accelsFloat_B[Z_AXIS] = (float)accels_B[Z_AXIS];

        VectorNormalize( accelsFloat_B, aHat_B );

        // Form the gravity vector in the body-frame (negative of the
        //   accelerometer measurement)
        gHat_B[X_AXIS] = -aHat_B[X_AXIS];
        gHat_B[Y_AXIS] = -aHat_B[Y_AXIS];
        gHat_B[Z_AXIS] = -aHat_B[Z_AXIS];

        // Form the roll and pitch angles (in radians) from the gravity
        //   unit-vector.  Formulation is based on a 321-rotation sequence
        //   and assumption that the gravity-vector is constant.
        gLeveler.measuredEulerAngles_BinN[ROLL]  = (real)( atan2( gHat_B[Y_AXIS],
                                                                  gHat_B[Z_AXIS] ) );
        gLeveler.measuredEulerAngles_BinN[PITCH] = (real)( -asin( gHat_B[X_AXIS] ) );

        // Generate the debug output
        debugOutputCntr++;
        if(debugOutputCntr >= debugOutputCntrLimit) {
            debugOutputCntr = 0;
            DebugPrintFloat("Time: ", gLeveler.timerCntr * 0.001, 3);
            DebugPrintFloat(", Roll: ", gLeveler.measuredEulerAngles_BinN[ROLL] * RAD_TO_DEG, 5);
            DebugPrintFloat(", Pitch: ", gLeveler.measuredEulerAngles_BinN[PITCH] * RAD_TO_DEG, 5);
            DebugPrintEndline();
        }
    }

    return NULL;
}


// Extract the attitude (expressed in Euler-angles) of the body-frame (B)
//   in the NED-frame (N) in degrees.
void Leveler_GetAttitude_EA(real *EulerAngles)
{
    EulerAngles[ROLL]  = gLeveler.measuredEulerAngles_BinN[ROLL]  * RAD_TO_DEG;
    EulerAngles[PITCH] = gLeveler.measuredEulerAngles_BinN[PITCH] * RAD_TO_DEG;
}


// Setter used to specify the leveler execution frequency at initialization
void Leveler_SetExeFreq(uint16_t freq)
{
    gLeveler.callingFreq = freq;
}


// Initialization routine used to set up the timing output variables
void Leveler_InitializeDataStruct(void)
{
    // Compute dt from the calling frequency of the function, compute the
    //   timer delta-value from dt (account for rounding), and initialize
    //   the timer.
    gLeveler.dt = 1.0 / (real)gLeveler.callingFreq;
    gLeveler.dTimerCntr = (uint32_t)( 1000.0 * gLeveler.dt + 0.5 );
    gLeveler.timerCntr = 0;
}




