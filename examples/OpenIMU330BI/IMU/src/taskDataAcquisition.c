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
#include "UserCommunicationSPI.h"
#include "osapi.h"
#include "board.h"
#include "ucb_packet.h"
#include "userAPI.h"
#include "hwAPI.h"
#include "sensorsAPI.h"
#include "calibrationAPI.h"
#include "commAPI.h"


int cycleError = 0; 
/** ***************************************************************************
 * @name TaskDataAcquisition() CALLBACK main loop
 * @brief Get the sensor data at the specified frequency (based on the
 *        configuration of the accelerometer rate-sensor). Process and provide
 *        information to the user via the UART or SPI.
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void TaskDataAcquisition()
{

    InitSensorsData();
    
    // Frequency 200 Hz. Do not change
    StartDacqTimer(200);

    initUserDataProcessingEngine();

 
     while( 1 )
    {

        if(TIMER_IsDacqOverrun()){
            cycleError++;
        }
        
        TIMER_WaitForNewDacqTick();

        HW_IO3_On();
        
        if(fSPI){
            // deactivate data ready - set hi 
            // add timing logic here to match SPI data rate
            HW_DRDY_Off();             
        }

        SampleSensorsData();
        
        ApplyFactoryCalibration();

        HW_IO3_Off();

        // *****************************************************************
        // At this we have calibrated sensors data in next units
        // Acceleration - g's 
        // Rates        - rad/s
        //******************************************************************


        HW_IO2_On();

        // **********************  Algorithm ************************
        inertialAndPositionDataProcessing(200);
        
        HW_IO2_Off();

        if(fSPI){
            // Perform interface - specific processing here
            FillSPIDataBuffer();
            HW_DRDY_On(); // activate data ready - set low 
       } else {
            // Process commands and output continuous packets to UART
            // Processing of user commands always goes first
            ProcessUserCommands ();
            SendContinuousPacket(200);
        }
    }
}
