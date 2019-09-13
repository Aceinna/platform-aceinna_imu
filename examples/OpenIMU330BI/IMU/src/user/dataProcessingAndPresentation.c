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
#include "UserAlgorithm.h"

#ifdef DISPLAY_ALGORITHM_MSG
#include "debug.h"     // For debug commands
#endif

// Local-function prototypes
static void _GenerateDebugMessage(double *accels, double *rates, uint16_t dacqRate, uint16_t debugOutputFreq);

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
void inertialAndPositionDataProcessing(int dacqRate)
{  
    double accels[3];
    double rates[3];
    double temp;

    // Obtain accelerometer data [g]
    GetAccelData_g(accels);

    // Obtain rate-sensor data [deg/sec]
    GetRateData_degPerSec(rates);

    // Obtain board temperature data [degC]
    GetBoardTempData(&temp);

    // Generate a debug message that provide sensor data in order to verify the
    // algorithm input data is as expected.
    _GenerateDebugMessage(accels, rates, dacqRate, ZERO_HZ);

    // Execute user algorithm 
    RunUserNavAlgorithm(accels, rates, dacqRate);

}


//
static void _GenerateDebugMessage(double *accels, double *rates, uint16_t dacqRate, uint16_t debugOutputFreq)
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
#ifdef DISPLAY_ALGORITHM_MSG
            // IMU Data
            DebugPrintFloat("Time: ", counter*5, 3);
            DebugPrintFloat(",   a: [ ", (float)accels[0], 3);
            DebugPrintFloat("   ",       (float)accels[1], 3);
            DebugPrintFloat("   ",       (float)accels[2], 3);
            DebugPrintFloat(" ],   w: [ ", (float)rates[0] * RAD_TO_DEG, 3);
            DebugPrintFloat("   ",         (float)rates[0] * RAD_TO_DEG, 3);
            DebugPrintFloat("   ",         (float)rates[0] * RAD_TO_DEG, 3);
            DebugPrintString(" ]");
            DebugPrintEndline();
#endif
        }
    }
}


