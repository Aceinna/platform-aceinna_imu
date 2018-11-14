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
#include "gpsAPI.h"
#include "UserMessaging.h"

#include "Indices.h"   // For X_AXIS and Z_AXIS
#include "debug.h"     // For debug commands

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
}


// Notes:
// 1) 'inertialAndPositionDataProcessing' is common for all platforms, but implementation
//    of the methods inside is platform and user-dependent.
// 2) 'DataAcquisitionTask' calls this function after retrieving samples of current
//    sensors data and applying corresponding calibration 
// 3) 'dacqRate' is rate with which new set of sensors data arrives 
void inertialAndPositionDataProcessing(uint16_t dacqRate)
{  
    // 
    void          *results;

    // Initialization variable
    static int initFlag  = 1;

    // Variables that control the output frequency of the debug statement
    static int debugFlag = 0;
    static uint8_t debugOutputCntr, debugOutputCntrLimit;

    // Initialize timer variables (used to control the output of the debug
    //   messages and the IMU timer value)
    if(initFlag) {
        // Reset 'initFlag' so this section is not executed more than once.
        initFlag = 0;

        // Set the variables that control the debug-message output-rate (based on
        //   the desired calling frequency of the debug output)
        debugOutputCntr = 0;

        uint16_t debugOutputFreq = 4;  // [Hz]
        debugOutputCntrLimit = dacqRate / debugOutputFreq;

        // Set the IMU output delta-counter value
        gIMU.dTimerCntr = (uint32_t)( 1000.0 / (float)(dacqRate) + 0.5 );
    }

    // Increment the timer-counter by the sampling period equivalent to the rate at which
    //   inertialAndPositionDataProcessing is called
    gIMU.timerCntr = gIMU.timerCntr + gIMU.dTimerCntr;

    // Obtain accelerometer data [g]
    GetAccelData_g(gIMU.accel_g);

    // Obtain rate-sensor data [rad/sec]
    GetRateData_radPerSec(gIMU.rate_radPerSec);

    // Obtain magnetometer data [G]
    GetMagData_G(gIMU.mag_G);

    // Obtain board temperature data [degC]
    GetBoardTempData(&gIMU.temp_C);

    // Generate the debug output to test the loading of the sensor data
    //   into the IMU data structure
    if( debugFlag ) {
        debugOutputCntr++;
        if(debugOutputCntr >= debugOutputCntrLimit) {
            debugOutputCntr = 0;
            DebugPrintFloat("Time: ", 0.001 * (real)gIMU.timerCntr, 3);
            DebugPrintFloat(", AccelZ: ", gIMU.accel_g[Z_AXIS], 3);
            DebugPrintFloat(", RateZ: ", gIMU.rate_radPerSec[Z_AXIS] * RAD_TO_DEG, 3);
            DebugPrintFloat(", MagX: ", gIMU.mag_G[X_AXIS], 3);
            DebugPrintFloat(", Temp: ", gIMU.temp_C,2);
            DebugPrintEndline();
        }
    }

    // Execute user algorithm (default implementation located in file user_algorithm.c)
    results = RunUserNavAlgorithm(gIMU.accel_g, gIMU.rate_radPerSec, gIMU.mag_G, NULL, dacqRate);

    // add current result to output queue for subsequent sending out as continuous packet                                                                                                                                                     // returns pointer to user-defined results structure
    WriteResultsIntoOutputStream(results) ;   // default implementation located in file file UserMessaging.c
}
