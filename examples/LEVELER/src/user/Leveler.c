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

#include "Leveler.h"

#include "VectorMath.h"
#include "math.h"

#include "gpsAPI.h"

#include "MagAlign.h"

static void _Leveler_IncrementTimer(void);

// Variables and functions for the leveler algorithm
LevelerDataStruct gLeveler;

// Extract the attitude (expressed in Euler-angles) of the body-frame (B)
//   in the NED-frame (N) in degrees.
void Leveler_GetAttitude_EA(real *EulerAngles)
{
    EulerAngles[ROLL]  = gLeveler.measuredEulerAngles_BinN[ROLL]  * RAD_TO_DEG;
    EulerAngles[PITCH] = gLeveler.measuredEulerAngles_BinN[PITCH] * RAD_TO_DEG;
}


// Replace this with algorithm_setExeFreq
// Setter used to specify the leveler execution frequency at initialization
void Leveler_SetExeFreq(uint16_t freq)
{
    gLeveler.callingFreq = freq;
}


// Initialization routine used to set up the timing output variables
void Leveler_InitializeAlgorithmStruct(uint16_t callingFreq)
{
    // Compute dt from the calling frequency of the function, compute the
    //   timer delta-value from dt (account for rounding), and initialize
    //   the timer.
    gLeveler.callingFreq = callingFreq;

    gLeveler.dt = 1.0 / (real)callingFreq;
    gLeveler.dTimerCntr = (uint32_t)( 1000.0 * gLeveler.dt + 0.5 );
    gLeveler.timerCntr = 0;
}


//
void Leveler_Algorithm(void)
{
    // Measurement variables
    float accelsFloat_B[NUM_AXIS], aHat_B[NUM_AXIS];
    volatile float gHat_B[NUM_AXIS];

    //
    _Leveler_IncrementTimer();

    // -------- Compute the roll/pitch values from accelerometer readings --------
    // Compute the acceleration unit-vector
    accelsFloat_B[X_AXIS] = (float)gLeveler.input.accel_B[X_AXIS];
    accelsFloat_B[Y_AXIS] = (float)gLeveler.input.accel_B[Y_AXIS];
    accelsFloat_B[Z_AXIS] = (float)gLeveler.input.accel_B[Z_AXIS];

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
}


static void _Leveler_IncrementTimer(void)
{
    // Increment the timerCntr
    gLeveler.timerCntr = gLeveler.timerCntr + gLeveler.dTimerCntr;
}


// Populate the EKF input structure with sensor and GPS data (if used)
void Leveler_SetInputStruct(double *accels, double *rates, double *mags, gpsDataStruct_t *gps)
{
    // Accelerometer signal is in [g]
    gLeveler.input.accel_B[X_AXIS] = accels[X_AXIS];
    gLeveler.input.accel_B[Y_AXIS] = accels[Y_AXIS];
    gLeveler.input.accel_B[Z_AXIS] = accels[Z_AXIS];

    // Angular-rate signal is in [rad/s]
    gLeveler.input.angRate_B[X_AXIS] = rates[X_AXIS];
    gLeveler.input.angRate_B[Y_AXIS] = rates[Y_AXIS];
    gLeveler.input.angRate_B[Z_AXIS] = rates[Z_AXIS];

    // Magnetometer signal is in [G]
    gLeveler.input.magField_B[X_AXIS] = mags[X_AXIS];
    gLeveler.input.magField_B[Y_AXIS] = mags[Y_AXIS];
    gLeveler.input.magField_B[Z_AXIS] = mags[Z_AXIS];
}


// Populate the compass output structure with algorithm results
void Leveler_SetOutputStruct(void)
{
    // Euler-angles in [deg]
    gLeveler.output.measuredEulerAngles_BinN[ROLL]  = gLeveler.measuredEulerAngles_BinN[ROLL]  * RAD_TO_DEG;
    gLeveler.output.measuredEulerAngles_BinN[PITCH] = gLeveler.measuredEulerAngles_BinN[PITCH] * RAD_TO_DEG;
}
