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


// Next function is common for all platforms, but implementation of the methods inside is platform and user-dependent
// Call to this function made from DataAcquisitionTask after retrieving samples of current sensors data and
// applying corresponding calibration 
// dacqRate is rate with which new set of sensors data arrives 
void inertialAndPositionDataProcessing(int dacqRate)
{  

    double         accels[3];       // in G
    double         rates[3];        // in rad/s
    double         mags[3];         // Gauss
    double         accelTemps[3];   // deg C
    double         rateTemps[3];    // deg C
    double         boardTemp;       // deg C
    gpsDataStruct_t gps;        
    void          *results;    

// get accels data or comment out
    GetAccelsData_g(accels);
                                              
// get rates data or comment out
    GetRatesData_radPerSec(rates); 

// get mags data or comment out
    GetMagsData_G(mags);  

// get acels temperature data or comment out
    GetAccelsTempData(accelTemps);

// get rates temperature data or comment out
    GetRatesTempData(rateTemps);

// get board temperature data or comment out
    GetBoardTempData(&boardTemp);

// get GPS data or comment out
    GetGPSData(&gps);  

// execute user algorithm or remove
    results = RunUserNavAlgorithm(accels, rates, mags, &gps, dacqRate); // default implementation located in file user_algorithm.c

// add current result to output queue for subsequent sending out as continious packet                                                                                                                                                     // returns pointer to user-defined results structure
    WriteResultsIntoOutputStream(results) ;   // default implementation located in file file UserMessaging.c

}



