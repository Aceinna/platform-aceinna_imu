/*****************************************************************************
 * @file   dataProcessingAndPresentation.c
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * contains of data post-processing  framework
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
#include "sensorsAPI.h"
#include "platformAPI.h"
#include "UserConfiguration.h"

#include "Indices.h"   // For X_AXIS and Z_AXIS
#include "debug.h"     // For debug commands

// Local-function prototypes
static void _IncrementIMUTimer(uint16_t dacqRate);
static void _GenerateDebugMessage(uint16_t dacqRate, uint16_t debugOutputFreq);
static void _IMUDebugMessage(void);
BOOL fUpdateBias = FALSE;


/*                                    *
                        ****************** 
                                      *
                Sensors data processing flow (from left to right)

Raw  *************** Filtered  * ************ Calibrated ****************   *********************  Data
Data * Built-in    * Raw Data  *            *   Data     * User Filter s*   * Algorithm         *  Output
****** LP  Filter  ************* Calibration ************* (if desired) ***** (Kalman Filter    ********
     *             *           *            *            *              *   * or user Algorithm)*
     ************* *           **************            ****************   *********************
        ^^^^^^^
    Can be selected during
    initialization or turned
            OFF
*///<--------------------- Cyclical processing at 100 or 200 Hz in Data Acquisition Task --------------> 


// Next function is common for all platforms, but implementation of the methods inside is platform-dependent
// Call to this function made from DataAcquisitionTask during initialization phase
// All user algorithm and data structures should be initialized here, if used
void initUserDataProcessingEngine()
{
    InitUserAlgorithm();         // default implementation located in file user_algorithm.c
    // magnetometer data filter defaulted t 5Hz but can be disabled
//    platformEnableMagFilter(FALSE);
}


// Notes:
// 1) 'inertialAndPositionDataProcessing' is common for all platforms, but implementation
//    of the methods inside is platform and user-dependent.
// 2) 'DataAcquisitionTask' calls this function after retrieving samples of current
//    sensors data and applying corresponding calibration 
// 3) 'dacqRate' is rate with which new set of sensors data arrives 
void inertialAndPositionDataProcessing(uint16_t dacqRate)
{  
    // Increment the IMU timer by the calling rate of the data-acquisition task
    _IncrementIMUTimer(dacqRate);

    // Compensate rate sensors for bias. if desired
    // CompensateSensorsDataForTiltAndBias();

    // Obtain accelerometer data [g]
    GetAccelData_g(gIMU.accel_g);

    // Obtain rate-sensor data [rad/sec]
    GetRateData_radPerSec(gIMU.rate_radPerSec);
    GetRateData_degPerSec(gIMU.rate_degPerSec);

    // Obtain magnetometer data [G]
    GetMagData_G(gIMU.mag_G);

    // Obtain board temperature data [degC]
    GetBoardTempData(&gIMU.temp_C);

    // Generate a debug message that provide sensor data in order to verify the
    //   algorithm input data is as expected.
    _GenerateDebugMessage(dacqRate, ZERO_HZ);


    // Run User Algorithm
    RunUserNavAlgorithm(gIMU.accel_g, gIMU.rate_radPerSec, gIMU.mag_G, NULL, dacqRate);

}


//
static void _IncrementIMUTimer(uint16_t dacqRate)
{
    // Initialize timer variables (used to control the output of the debug
    //   messages and the IMU timer value)
    static int initFlag = 1;
    if(initFlag) {
        // Reset 'initFlag' so this section is not executed more than once.
        initFlag = 0;

        // Set the IMU output delta-counter value
        gIMU.dTimerCntr = (uint32_t)( 1000.0 / (float)(dacqRate) + 0.5 );
    }

    // Increment the timer-counter by the sampling period equivalent to the
    //   rate at which inertialAndPositionDataProcessing is called
    gIMU.timerCntr = gIMU.timerCntr + gIMU.dTimerCntr;
}


//
static void _GenerateDebugMessage(uint16_t dacqRate, uint16_t debugOutputFreq)
{
    // Variables that control the output frequency of the debug statement
    static uint8_t debugOutputCntr, debugOutputCntrLimit;

    // Check debug flag.  If set then generate the debug message to verify
    //   the loading of the GPS data into the GPS data structure
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

            // Reset 'new GPS data' flag (this should be done in UpdateFunctions
            //   to ensure the EKF can use the data)
            //gGPS.updateFlag = 0;  <-- This would make a difference as the input 
            //                          to the algorithm isn't set yet.

            // Create message here
            static uint8_t msgType = 1;
            switch( msgType )
            {
                case 0:
                    // None
                    break;
                    
                case 1:
                    // IMU data
                    _IMUDebugMessage();
                    break;
                    
                case 2:
                    // Other messages go here
                    break;
            }
        }
    }
}

    
static void _IMUDebugMessage(void)
{
    // IMU Data
    DebugPrintFloat("Time: ", 0.001 * (real)gIMU.timerCntr, 3);
    DebugPrintFloat(",   a: [ ", (float)gIMU.accel_g[X_AXIS], 3);
    DebugPrintFloat("   ", (float)gIMU.accel_g[Y_AXIS], 3);
    DebugPrintFloat("   ", (float)gIMU.accel_g[Z_AXIS], 3);
    DebugPrintFloat(" ],   w: [ ", (float)gIMU.rate_radPerSec[X_AXIS] * RAD_TO_DEG, 3);
    DebugPrintFloat("   ", (float)gIMU.rate_radPerSec[Y_AXIS] * RAD_TO_DEG, 3);
    DebugPrintFloat("   ", (float)gIMU.rate_radPerSec[Z_AXIS] * RAD_TO_DEG, 3);
    DebugPrintFloat(" ],   m: [ ", (float)gIMU.mag_G[X_AXIS], 3);
    DebugPrintFloat("   ", (float)gIMU.mag_G[Y_AXIS], 3);
    DebugPrintFloat("   ", (float)gIMU.mag_G[Z_AXIS], 3);
    DebugPrintString(" ]");
    DebugPrintEndline();
}
