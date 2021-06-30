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
#include "osapi.h"
#include "userAPI.h"
#include "sensorsAPI.h"
#include "calibrationAPI.h"
#include "serialAPI.h"
#include "halAPI.h"
#include "bitAPI.h"
#include "canJ1939API.h"
#include "EcuSettings.h"

/** ***************************************************************************
 * @name TaskDataAcquisition() CALLBACK main loop
 * @brief Get the sensor data at the specified frequency (based on the
 *        configuration of the accelerometer rate-sensor). Process and provide
 *        information to the user via the UART or SPI.
 * @param N/A
 *  
 ******************************************************************************/
void TaskDataAcquisition()
{
    sens_InitDataStructures();
    initUserDataProcessingEngine();
    
    BIT_BegintPowerCheck();

    while( 1 )
    {
      

        if(BIT_NeedResetFifo()){
           BOOL const res = sens_ResetFifo();
           if(res == TRUE){
                BIT_ClearFifoResetEvents();
           }
        }

        BIT_PerformPeriodicTest();

        if(OS_IsDacqOverrun()){
            BIT_DacqOverrun(TRUE);
        }else{
            BIT_DacqOverrun(FALSE);
        }
        

        OS_WaitForDacqTick();
        HW_TP1_On();

        TIMER_DacqStarted();

        BIT_UpdateDaqCycleStartFlag();
        HW_FeedWatchdog();

        sens_SampleData();
        cal_Apply();
        
        // *****************************************************************
        // At this point sensors data is in next units
        // Acceleration - g's 
        // Rates        - rad/s
        //******************************************************************
        inertialAndPositionDataProcessing(200);
        HW_TP1_Off();

        // Process commands and output continuous packets to UART
        // Processing of user commands always goes first
        ProcessUserCommands ();
        SendContinuousPacket(200);
        ProcessCANMessages();
        UpdateEcuSettings();
    }
}
