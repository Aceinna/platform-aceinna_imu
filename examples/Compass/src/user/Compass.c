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

#include "Compass.h"

#include "VectorMath.h"
#include "math.h"

#include "gpsAPI.h"

#include "MagAlign.h"

static void _Compass_IncrementTimer(void);

// SETTERS: for compass structures
CompassDataStruct gCompass;

// Extract the attitude (expressed in Euler-angles) of the body-frame (B)
//   in the NED-frame (N) in degrees.
void Compass_GetAttitude_EA(real *EulerAngles)
{
    EulerAngles[ROLL]  = gCompass.measuredEulerAngles_BinN[ROLL]  * RAD_TO_DEG;
    EulerAngles[PITCH] = gCompass.measuredEulerAngles_BinN[PITCH] * RAD_TO_DEG;
    EulerAngles[YAW]   = gCompass.measuredEulerAngles_BinN[YAW] * RAD_TO_DEG;
}


// Replace this with algorithm_setExeFreq
// Setter used to specify the leveler execution frequency at initialization
void Compass_SetExeFreq(uint16_t freq)
{
    gCompass.callingFreq = freq;
}


// Initialization routine used to set up the timing output variables
void Compass_InitializeAlgorithmStruct(uint16_t callingFreq)
{
    // Compute dt from the calling frequency of the function, compute the
    //   timer delta-value from dt (account for rounding), and initialize
    //   the timer.
    gCompass.callingFreq = callingFreq;

    gCompass.dt = 1.0 / (real)callingFreq;
    gCompass.dTimerCntr = (uint32_t)( 1000.0 * gCompass.dt + 0.5 );
    gCompass.timerCntr = 0;
}


//
void Compass_Algorithm(void)
{
    // Measurement variables
    float accelsFloat_B[NUM_AXIS], aHat_B[NUM_AXIS];
    volatile float gHat_B[NUM_AXIS];

    //
    _Compass_IncrementTimer();

    // -------- Compute the roll/pitch values from accelerometer readings --------
    // Compute the acceleration unit-vector
    accelsFloat_B[X_AXIS] = (float)gCompass.input.accel_B[X_AXIS];
    accelsFloat_B[Y_AXIS] = (float)gCompass.input.accel_B[Y_AXIS];
    accelsFloat_B[Z_AXIS] = (float)gCompass.input.accel_B[Z_AXIS];

    VectorNormalize( accelsFloat_B, aHat_B );

    // Form the gravity vector in the body-frame (negative of the
    //   accelerometer measurement)
    gHat_B[X_AXIS] = -aHat_B[X_AXIS];
    gHat_B[Y_AXIS] = -aHat_B[Y_AXIS];
    gHat_B[Z_AXIS] = -aHat_B[Z_AXIS];

    // Form the roll and pitch angles (in radians) from the gravity
    //   unit-vector.  Formulation is based on a 321-rotation sequence
    //   and assumption that the gravity-vector is constant.
    gCompass.measuredEulerAngles_BinN[ROLL]  = (real)( atan2( gHat_B[Y_AXIS],
                                                              gHat_B[Z_AXIS] ) );
    gCompass.measuredEulerAngles_BinN[PITCH] = (real)( -asin( gHat_B[X_AXIS] ) );

    // -------- Compute the heading (yaw) values from magnetometer readings --------
    // Form R_BinPerp from roll and pitch and level the magnetometer measurement
    real sinRoll, cosRoll;
    real sinPitch, cosPitch;

    //
    sinRoll  = (real)(sin( gCompass.measuredEulerAngles_BinN[ROLL] ));
    cosRoll  = (real)(cos( gCompass.measuredEulerAngles_BinN[ROLL] ));
    sinPitch = (real)(sin( gCompass.measuredEulerAngles_BinN[PITCH] ));
    cosPitch = (real)(cos( gCompass.measuredEulerAngles_BinN[PITCH] ));

    real temp;
    temp = sinRoll * (float)gCompass.input.magField_B[Y_AXIS] + 
           cosRoll * (float)gCompass.input.magField_B[Z_AXIS];

    float mags_Perp[NUM_AXIS];
    mags_Perp[X_AXIS] =  cosPitch * (float)gCompass.input.magField_B[X_AXIS] + sinPitch * temp;
    mags_Perp[Y_AXIS] =  cosRoll  * (float)gCompass.input.magField_B[Y_AXIS] - sinRoll  * (float)gCompass.input.magField_B[Z_AXIS];
    mags_Perp[Z_AXIS] = -sinPitch * (float)gCompass.input.magField_B[X_AXIS] + cosPitch * temp;

    // Correct for hard/soft-iron effects in the body-frame
    float magCorr[2], tmp[2];
    tmp[X_AXIS] = mags_Perp[X_AXIS] - gMagAlign.hardIronBias[X_AXIS];
    tmp[Y_AXIS] = mags_Perp[Y_AXIS] - gMagAlign.hardIronBias[Y_AXIS];

    magCorr[X_AXIS] = gMagAlign.SF[0] * tmp[X_AXIS] + gMagAlign.SF[1] * tmp[Y_AXIS];
    magCorr[Y_AXIS] = gMagAlign.SF[2] * tmp[X_AXIS] + gMagAlign.SF[3] * tmp[Y_AXIS];

    // Form the heading from the leveled magnetometer readings
    // The negative of the angle the vector makes with the unrotated (psi = 0)
    //   frame is the yaw-angle of the initial frame.
    gCompass.measuredEulerAngles_BinN[YAW] = (real)( -atan2( magCorr[Y_AXIS], magCorr[X_AXIS] ) );
}


static void _Compass_IncrementTimer(void)
{
    // Increment the timerCntr
    gCompass.timerCntr = gCompass.timerCntr + gCompass.dTimerCntr;
}


// Populate the EKF input structure with sensor and GPS data (if used)
void Compass_SetInputStruct(double *accels, double *rates, double *mags, gpsDataStruct_t *gps)
{
    // Accelerometer signal is in [g]
    gCompass.input.accel_B[X_AXIS] = accels[X_AXIS];
    gCompass.input.accel_B[Y_AXIS] = accels[Y_AXIS];
    gCompass.input.accel_B[Z_AXIS] = accels[Z_AXIS];

    // Angular-rate signal is in [rad/s]
    gCompass.input.angRate_B[X_AXIS] = rates[X_AXIS];
    gCompass.input.angRate_B[Y_AXIS] = rates[Y_AXIS];
    gCompass.input.angRate_B[Z_AXIS] = rates[Z_AXIS];

    // Magnetometer signal is in [G]
    gCompass.input.magField_B[X_AXIS] = mags[X_AXIS];
    gCompass.input.magField_B[Y_AXIS] = mags[Y_AXIS];
    gCompass.input.magField_B[Z_AXIS] = mags[Z_AXIS];
}


// Populate the compass output structure with algorithm results
void Compass_SetOutputStruct(void)
{
    // Euler-angles in [deg]
    gCompass.output.measuredEulerAngles_BinN[ROLL]  = gCompass.measuredEulerAngles_BinN[ROLL]  * RAD_TO_DEG;
    gCompass.output.measuredEulerAngles_BinN[PITCH] = gCompass.measuredEulerAngles_BinN[PITCH] * RAD_TO_DEG;
    gCompass.output.measuredEulerAngles_BinN[YAW]   = gCompass.measuredEulerAngles_BinN[YAW]   * RAD_TO_DEG;
}
