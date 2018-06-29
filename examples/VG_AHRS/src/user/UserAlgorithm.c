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

void _PopulateEKFInputStruct(double *accels, double *rates, double *mags, gpsDataStruct_t *gps);
void _PopulateEKFOutputStruct(void);


void _Algorithm(int dacqRate, uint8_t algoType);

void InitUserAlgorithm()
{
    // Initialize built-in algorithm structure
    setAlgorithmExeFreq(FREQ_200_HZ);
    InitializeAlgorithmStruct();
    // place additional required initialization here
}

#include "bsp.h"

void *RunUserNavAlgorithm(double *accels, double *rates, double *mags, gpsDataStruct_t *gps, int dacqRate)
{
    //set_IO3Pin(1);

    // Populate the EKF input data structure
    _PopulateEKFInputStruct(accels, rates, mags, gps);

    // Call the desired algorithm based on the EKF with different
    //   calling rates and different settings.
    _Algorithm(dacqRate, VG);

    // Fill the output data structure with the EKF states and other 
    //   desired information
    _PopulateEKFOutputStruct();

    return &gEKFOutputData;
}


//
void _PopulateEKFInputStruct(double *accels, double *rates, double *mags, gpsDataStruct_t *gps)
{
    // Accelerometer signal is in [g]
    gEKFInputData.accel_B[X_AXIS]    = accels[X_AXIS];
    gEKFInputData.accel_B[Y_AXIS]    = accels[Y_AXIS];
    gEKFInputData.accel_B[Z_AXIS]    = accels[Z_AXIS];

    // Angular-rate signal is in [rad/s]
    gEKFInputData.angRate_B[X_AXIS]  = rates[X_AXIS];
    gEKFInputData.angRate_B[Y_AXIS]  = rates[Y_AXIS];
    gEKFInputData.angRate_B[Z_AXIS]  = rates[Z_AXIS];

    // Magnetometer signal is in [G]
    gEKFInputData.magField_B[X_AXIS] = mags[X_AXIS];
    gEKFInputData.magField_B[Y_AXIS] = mags[Y_AXIS];
    gEKFInputData.magField_B[Z_AXIS] = mags[Z_AXIS];
}


//
void _Algorithm(int dacqRate, uint8_t algoType)
{
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
            gAlgorithm.Behavior.bit.useMag = 1;
             enableMagInAlgorithm(TRUE);
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


//
void _PopulateEKFOutputStruct(void)
{
    // ------------------ States ------------------

    // Position in [m]
    gEKFOutputData.position_N[0] = gKalmanFilter.Position_N[0];
    gEKFOutputData.position_N[1] = gKalmanFilter.Position_N[1];
    gEKFOutputData.position_N[2] = gKalmanFilter.Position_N[2];

    // Velocity in [m/s]
    gEKFOutputData.velocity_N[0] = gKalmanFilter.Velocity_N[0];
    gEKFOutputData.velocity_N[1] = gKalmanFilter.Velocity_N[1];
    gEKFOutputData.velocity_N[2] = gKalmanFilter.Velocity_N[2];

    // Position in [N/A]
    gEKFOutputData.quaternion_BinN[0] = gKalmanFilter.quaternion[0];
    gEKFOutputData.quaternion_BinN[1] = gKalmanFilter.quaternion[1];
    gEKFOutputData.quaternion_BinN[2] = gKalmanFilter.quaternion[2];
    gEKFOutputData.quaternion_BinN[3] = gKalmanFilter.quaternion[3];

    // Angular-rate bias in [deg/sec]
    gEKFOutputData.angRateBias_B[0] = gKalmanFilter.rateBias_B[0] * 57.295779513082323;
    gEKFOutputData.angRateBias_B[1] = gKalmanFilter.rateBias_B[1] * 57.295779513082323;
    gEKFOutputData.angRateBias_B[2] = gKalmanFilter.rateBias_B[2] * 57.295779513082323;

    // Acceleration-bias in [m/s^2]
    gEKFOutputData.accelBias_B[0] = gKalmanFilter.accelBias_B[0] * 9.80655;
    gEKFOutputData.accelBias_B[1] = gKalmanFilter.accelBias_B[1] * 9.80655;
    gEKFOutputData.accelBias_B[2] = gKalmanFilter.accelBias_B[2] * 9.80655;

    // ------------------ Derived variables ------------------
    
    // Euler-angles in [deg]
    gEKFOutputData.eulerAngs_BinN[0] = gKalmanFilter.eulerAngles[0] * 57.295779513082323;
    gEKFOutputData.eulerAngs_BinN[1] = gKalmanFilter.eulerAngles[1] * 57.295779513082323;
    gEKFOutputData.eulerAngs_BinN[2] = gKalmanFilter.eulerAngles[2] * 57.295779513082323;

    // Angular-rate in [deg/s]
    gEKFOutputData.corrAngRates_B[0] = ( gEKFInputData.angRate_B[0] - gKalmanFilter.rateBias_B[0] ) * 57.295779513082323;
    gEKFOutputData.corrAngRates_B[1] = ( gEKFInputData.angRate_B[1] - gKalmanFilter.rateBias_B[1] ) * 57.295779513082323;
    gEKFOutputData.corrAngRates_B[2] = ( gEKFInputData.angRate_B[2] - gKalmanFilter.rateBias_B[2] ) * 57.295779513082323;

    // Acceleration in [m/s^2]
    gEKFOutputData.corrAccel_B[0] = ( gEKFInputData.accel_B[0] - gKalmanFilter.accelBias_B[0] ) * 9.80655;
    gEKFOutputData.corrAccel_B[1] = ( gEKFInputData.accel_B[1] - gKalmanFilter.accelBias_B[1] ) * 9.80655;
    gEKFOutputData.corrAccel_B[2] = ( gEKFInputData.accel_B[2] - gKalmanFilter.accelBias_B[2] ) * 9.80655;


    // ------------------ Algorithm flags ------------------
    gEKFOutputData.opMode         = gAlgorithm.state;
    gEKFOutputData.linAccelSwitch = gAlgorithm.linAccelSwitch;
    gEKFOutputData.turnSwitchFlag = (uint8_t)gAlgorithm.bitStatus.swStatus.bit.turnSwitch;
}

