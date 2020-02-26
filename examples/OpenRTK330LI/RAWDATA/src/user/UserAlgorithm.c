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

#ifndef INS_OFFLINE
#include "debug.h"
#endif // !INS_OFFLINE

// Declare the IMU data structure
//IMUDataStruct   gIMU;
extern gpsDataStruct_t gGPS;

//
static void _Algorithm(uint16_t dacqRate, uint8_t algoType);
static void _InitAlgo(uint8_t algoType);

#ifndef INS_OFFLINE
static void _GenerateDebugMessage(uint16_t dacqRate, uint16_t debugOutputFreq);
#endif // !INS_OFFLINE

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


void *RunUserNavAlgorithm(double *accels, double *rates, double *mags, gpsDataStruct_t *gps, uint16_t dacqRate)
{
    // This can be set at startup based on the packet type selected
    static uint8_t algoType = INS0;

    // Initialize variable related to the UserNavAlgorithm
    _InitAlgo(algoType);

    // Populate the EKF input data structure
    EKF_SetInputStruct(accels, rates, mags, gps);

    // Call the desired algorithm based on the EKF with different
    //   calling rates and different settings.
    _Algorithm(dacqRate, algoType);

    // Fill the output data structure with the EKF states and other 
    //   desired information
    EKF_SetOutputStruct();

#ifndef INS_OFFLINE
	// Generate a debug message that provides algorithm output to verify the
	//   algorithm is generating the proper output.
	_GenerateDebugMessage(dacqRate, ZERO_HZ);
#endif // !INS_OFFLINE

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
                // enableMagInAlgorithm(TRUE);
                break;
            case INS0:
                /* Set the configuration variables for INS solution.
                 * (Enable GPS and set algorithm calling frequency to 100Hz)
                 */
                // enableMagInAlgorithm(FALSE);
                enableGpsInAlgorithm(TRUE);
                // gAlgorithm.callingFreq = FREQ_100_HZ;  // redundant; set above
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
    static int algoCntr = 0, algoCntrLimit = 0;

    // Initialize the configuration variables needed to make the system
    //   generate a VG-type solution.
    static uint8_t initAlgo = 1;
    if(initAlgo) {
        // Reset 'initAlgo' so this is not executed more than once.  This
        //   prevents the algorithm from being switched during run-time.
        initAlgo = 0;

        // Set the variables that control the algorithm execution rate
        algoCntrLimit = (uint8_t)( (float)dacqRate / (float)gAlgorithm.callingFreq + 0.5 );
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

        // Log GPS measurement update
        if (gEKFInput.gpsUpdate == 1)
        {
            gAlgoStatus.bit.gpsUpdate = 1;
        }
        else
        {
            gAlgoStatus.bit.gpsUpdate = 0;
        }
        // Aceinna VG/AHRS/INS algorithm
        EKF_Algorithm();
    }
} 

#ifndef INS_OFFLINE
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
#if 1
            // For testing, the output packet should contain:
            //  1) ITOW
            //  2) Euler Angles
            //  3) Ang-rate (meas)
            //  4) Est ang-rate bias
            //  5) Accel (meas)
            //  6) Est accel bias
            //  7) LLA (est)
            //  8) NED-Vel (est)
            //  9) Oper mode

            DebugPrintLongInt("", gPtrGnssSol->itow);

            DebugPrintFloat(",   ", gEKFOutput.eulerAngs_BinN[ROLL],  4);
            DebugPrintFloat(",   ", gEKFOutput.eulerAngs_BinN[PITCH], 4);
            DebugPrintFloat(",   ", gEKFOutput.eulerAngs_BinN[YAW],   4);

            DebugPrintFloat(",   ", gEKFInput.angRate_B[X_AXIS], 5);
            DebugPrintFloat(",   ", gEKFInput.angRate_B[Y_AXIS], 5);
            DebugPrintFloat(",   ", gEKFInput.angRate_B[Z_AXIS], 5);

            DebugPrintFloat(",   ", gEKFOutput.angRateBias_B[X_AXIS], 5);
            DebugPrintFloat(",   ", gEKFOutput.angRateBias_B[Y_AXIS], 5);
            DebugPrintFloat(",   ", gEKFOutput.angRateBias_B[Z_AXIS], 5);

            DebugPrintFloat(",   ", gEKFInput.accel_B[X_AXIS], 5);
            DebugPrintFloat(",   ", gEKFInput.accel_B[Y_AXIS], 5);
            DebugPrintFloat(",   ", gEKFInput.accel_B[Z_AXIS], 5);

            DebugPrintFloat(",   ", gEKFOutput.accelBias_B[X_AXIS], 5);
            DebugPrintFloat(",   ", gEKFOutput.accelBias_B[Y_AXIS], 5);
            DebugPrintFloat(",   ", gEKFOutput.accelBias_B[Z_AXIS], 5);

            DebugPrintFloat(",   ", gEKFOutput.llaDeg[LAT], 8);
            DebugPrintFloat(",   ", gEKFOutput.llaDeg[LON], 8);
            DebugPrintFloat(",   ", gEKFOutput.llaDeg[ALT], 5);

            DebugPrintFloat(",   ", gEKFOutput.velocity_N[X_AXIS], 5);
            DebugPrintFloat(",   ", gEKFOutput.velocity_N[Y_AXIS], 5);
            DebugPrintFloat(",   ", gEKFOutput.velocity_N[Z_AXIS], 5);
#if 1
            DebugPrintInt(",   ", gEKFOutput.opMode);
#else
            switch(gEKFOutput.opMode) {
                case 0:
                    DebugPrintString(",   Stab");
                    break;
                case 1:
                    DebugPrintString(",   Init");
                    break;
                case 2:
                    DebugPrintString(",   HG");
                    break;
                case 3:
                    DebugPrintString(",   LG");
                    break;
                case 4:
                    DebugPrintString(",   INS");
                    break;
            }
#endif            
            DebugPrintEndline();
#else
            //
            DebugPrintLongInt("ITOW: ", gGPS.itow );
            // LLA
            DebugPrintFloat(", valid: ", (float)gGPS.gpsFixType, 1);
            // LLA
            DebugPrintFloat(", Lat: ", (float)gGPS.latitude, 8);
            DebugPrintFloat(", Lon: ", (float)gGPS.longitude, 8);
            DebugPrintFloat(", Alt: ", (float)gGPS.altitude, 5);
            // Velocity
            //DebugPrintFloat(", vN: ", (float)gGPS.vNed[X_AXIS], 5);
            //DebugPrintFloat(", vE: ", (float)gGPS.vNed[Y_AXIS], 5);
            //DebugPrintFloat(", vD: ", (float)gGPS.vNed[Z_AXIS], 5);
            // v^2
            //double vSquared = gGPS.vNed[X_AXIS] * gGPS.vNed[X_AXIS] + 
            //                  gGPS.vNed[Y_AXIS] * gGPS.vNed[Y_AXIS];
            //DebugPrintFloat(", vSq: ", (float)vSquared, 5);
            //DebugPrintFloat(", vSq: ", (float)gGPS.rawGroundSpeed * (float)gGPS.rawGroundSpeed, 5);
            // Newline
            DebugPrintEndline();
#endif
        }
    }
}
#endif // !INS_OFFLINE