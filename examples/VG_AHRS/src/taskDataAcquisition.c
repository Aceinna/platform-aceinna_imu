/*****************************************************************************
 * @file   taskDataAcquisition.c
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * sensor data acquisition task runs at 100Hz, gets the data for each sensor
 * and applies available calibration
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

#include "taskDataAcquisition.h"
#include "taskUserCommunication.h"
#include "UserConfiguration.h"
#include "magAPI.h"
#include "bitAPI.h"
//#include "boardAPI.h"
#include "bsp.h"
#include "algorithmAPI.h"
#include "Indices.h"
#include "qmath.h"
#include "osapi.h"
#include "osresources.h"
#include "userAPI.h"
#include "platformAPI.h"

uint32_t dacqTimer = 0;

uint32_t getDacqTime()
{
    return dacqTimer;
}


/** ***************************************************************************
 * @name TaskDataAcquisition() CALLBACK main loop
 * @brief Get the sensor data at the specified frequency (based on the
 *        configuration of the accelerometer rate-sensor). Process and provide
 *        information to the user via the UART or SPI.
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void TaskDataAcquisition(void const *argument)
{
 //   uint8_t functionStatus;
    int  res, dacqRate;
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
    BOOL overRange  = FALSE;        //uncomment this line if overrange processing required
#pragma GCC diagnostic warning "-Wunused-but-set-variable"


    // This routine sets up the timer. Can use the structure below this point.
    TaskDataAcquisition_Init();
    // Initialize user algorithm parameters, if needed
    initUserDataProcessingEngine();
    // set the mag align scale factors
    InitMagAlignParams();   
    // Start sensors data acquisition 
    DataAquisitionStart();
    // determine the period of data acquisition task
    dacqRate = DACQ_200_HZ;



    //**************  Add user initialization here, if needed ****************


    /// Data is provided by the primary sensors (accelerometer and rate-sensor)
    ///   as fast as possible.  Data is obtained from secondary sensors at a
    ///   lower rate.  The rate-sensor data read is synced to TIM5 or an
    ///   external signal.  When the read is commanded and in the data buffer,
    ///   The data-event flag is set and the wait is bypassed.  Data is obtained
    ///   from the buffer, calibrated, filtered and provided to the user for 
    ///   subsequent processing.

    while( 1 )
    {
        // *****************************************************************
        // NOTE: This task loop runs at 100 or 200 Hz (default 200 Hz)
        //       user can choose period of this task by 
        // *****************************************************************
        // Handle Timing vard, watchdog and BIT
        PrepareToNewDacqTick();
        //  Wait for next tick 
        // Upon timeout of TIM5 (or user sync), let the process continue
        
        res = osSemaphoreWait(dataAcqSem, 1000);
        if(res != osOK){
            // Wait timeout expired. Something wrong wit the dacq system
            // Process timeout here
        }
        
        // inform user, that new data set is being prepared (if required)
        // in case of UART communication interface sets pin IO2 high
        setIO2Pin (1);

        // Get calibrated sensor data:
        //   Inside this function the sensor data is filtered by a second-order low-pass
        //   Butterworth filter, with a cutoff frequency selected by the user (or zero to
        //   disable).  The cutoff is selected using the following:
        //
        //       Select_LP_filter(rawSensor_e sensorType, eFilterType filterType)
        //
        //   Refer to UserConfiguration.c for implementation and to the enumerator structure 
        //   'eFilterType' in file filter.h for available selections.
        //
        //   Low pass filtering is followed by application of the unit calibration parameters
        //   - scaling, temperature compensation, bias removal, and misalignment.
        //
        //   Results are placed in the structure of type double. The pointer to this
        //   structure is 'pScaledSensors'.  Ordering of sensors data in this structure
        //   defined by 'rawSensor_e' enumerator in the file indices.h
        GetSensorsData();

        // Check if sensors data over range
        // If overRange is TRUE - limit values are put into sensors data based 
        // on chosen sensors sensitivity 
        overRange = handleOverRange();          

        // *****************************************************************
        // At this point sensors data is in next units
        // Acceleration - g's 
        // Rates        - rad/s
        // Magnetometer - Gauss
        // Temperature  - degrees C
        //******************************************************************
        
        
        // BIT status. May have inadvertently changed this during an update.
        updateBITandStatus();

        // **********************  Algorithm ************************
        // In AHRS or INS mode due to CPU performance concerns algorithm better be
        // performed at 100 or less Hz based on complexity and number calculations
        // involved.  Incorporate timing logic inside algorithm function, if desired   
        // For the initial simplicity use pScaledSensors array as input 
        // and output of algorithm.
        // Use is100Hz variable to determine the rate of this task loop 
        inertialAndPositionDataProcessing(dacqRate);
        
        //Uncomment next line if there is intention of using S0 or S1 xbow packets
        //for continuous data output
        //*****************************************************************
        // applyNewScaledSensorsData();
        //*****************************************************************

        // Inform user, that new data set is ready (if required)
        // in case of UART communication interface clears pin IO2
        // Pin IO2 can be used for timing of data processing 
        setIO2Pin(0);

        if (platformHasMag() ) {
            // Mag Alignment (follows Kalman filter or user algorithm as the innovation routine
            // calculates the euler angles and the magnetic vector in the
            // NED-frame)
            MagAlign();   // only does this on align
        }

        if(getUnitCommunicationType() != UART_COMM){
            // Perform interface - specific processing here
        } else {
            // Process commands and output continuous packets to UART
            // Processing of user commands always goes first
            ProcessUserCommands ();
            SendContinuousPacket(dacqRate);
        }
    }
}

