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

#include <string.h>
#include "userAPI.h"
#include "sensorsAPI.h"
#include "gpsAPI.h"
#include "ekfAPI.h"
#include "odoAPI.h"
#include "UserAlgorithm.h"
#include "EcuSettings.h"

gpsDataStruct_t      gGPS;
IMUDataStruct        gIMU;
odoDataStruct_t      gOdo;

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
    // Initializa global variables of sensor data
    memset(&gGPS, 0, sizeof(gGPS));
    memset(&gIMU, 0, sizeof(gIMU));

    InitUserAlgorithm();         // default implementation located in file user_algorithm.c
}


// Notes:
// 1) 'inertialAndPositionDataProcessing' is common for all platforms, but implementation
//    of the methods inside is platform and user-dependent.
// 2) 'DataAcquisitionTask' calls this function after retrieving samples of current
//    sensors data and applying corresponding calibration 
// 3) 'dacqRate' is rate with which new set of sensors data arrives 
void inertialAndPositionDataProcessing(int32_t const dacqRate)
{  
    static uint8_t initAlgo = 1U;
    static uint8_t algoCntr = 0U;
    static uint8_t algoCntrLimit = 0U;

    if(!UseAlgorithm()){
        return;
    }


    if (initAlgo) 
    {
        // Reset 'initAlgo' so this is not executed more than once.  This
        //   prevents the algorithm from being switched during run-time.
        initAlgo = 0U;
        uint32_t const  algoFreq = EKF_GetCallingFreq();
        
        // Set the variables that control the algorithm execution rate
        float32_t const algoCntrLimit_f = ((float32_t)dacqRate / (float32_t)algoFreq) + 0.5F;
        algoCntrLimit = (uint8_t)(algoCntrLimit_f);

        if (algoCntrLimit < 1U) 
        {
            // If this logic is reached, also need to adjust the algorithm
            //   parameters to match the modified calling freq (or stop the
            //   program to indicate that the user must adjust the program)
            algoCntrLimit = 1U;
        }
        algoCntr = algoCntrLimit;
    }
    // call the algorithm
    algoCntr++;

    if (algoCntr >= algoCntrLimit) 
    {

        // Reset counter
        algoCntr = 0U;

        // Obtain accelerometer data [g]
        sens_GetAccelData_g(gIMU.accel_g); 

        // Obtain rate-sensor data [rad/sec]
        sens_GetRateData_radPerSec(gIMU.rate_radPerSec);

        // Obtain magnetometer data [G]
        sens_GetMagData_G(gIMU.mag_G);

        // Obtain board temperature data [degC]
        sens_GetBoardTempData(&gIMU.temp_C);

        // Obtain aiding signal data.
        GetOdometerData(&gOdo);

        // Execute user algorithm (default implementation located in file user_algorithm.c)
        RunUserNavAlgorithm(gIMU.accel_g, gIMU.rate_radPerSec, gIMU.mag_G, &gGPS, &gOdo, FALSE);
    }
}



