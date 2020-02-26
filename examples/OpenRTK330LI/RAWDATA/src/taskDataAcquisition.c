/*****************************************************************************
 * @file   taskDataAcquisition.c
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * sensor data acquisition task runs at 50Hz, gets the data for each sensor
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
*******************************************************************************
* 16/12/2019  |                                             | Daich
* Description: add json cmd parse
*******************************************************************************/
#include <string.h>
#include <math.h>

#include "taskDataAcquisition.h"
#include "TimingVars.h"
#include "sensors_data.h"
#include "ucb_packet.h"
#include "userAPI.h"
#include "hwAPI.h"
#include "sensorsAPI.h"
#include "calibrationAPI.h"
#include "configurationAPI.h"
#include "configuration.h"
#include "UserConfiguration.h"

#include "osapi.h"
#include "osresources.h"
#include "commAPI.h"
#include "algorithmAPI.h"
#include "uart.h"
#include "TransformationMath.h"
#include "bsp.h"
#ifdef DEBUG_INS
#include "TransformationMath.h"
#endif
#include "shell.h"

extern EKF_OutputDataStruct gEKFOutput;
mcu_time_base_t IMU_start_time;
volatile mcu_time_base_t next_obs_time;
uint8_t gpsUpdateCpy = 0;
/** ***************************************************************************
 * @name TaskDataAcquisition() CALLBACK main loop
 * @brief Get the sensor data at the specified frequency (based on the
 *        configuration of the accelerometer rate-sensor). Process and provide
 *        information to the user via the UART.
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void TaskDataAcquisition(void const *argument)
{
    int res;
    uint16_t dacqRate;

#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
    BOOL overRange = FALSE; //uncomment this line if overrange processing required
#pragma GCC diagnostic warning "-Wunused-but-set-variable"

    res = InitSensors();
    InitSensorsData();

    // Initialize user algorithm parameters, if needed
    initUserDataProcessingEngine();

    // Set the sampling frequency of data acquisition task
    dacqRate = DACQ_200_HZ;

    while (1)
    {
        //UBaseType_t uxHighWaterMark;
        //uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
        TimingVars_Increment();

        res = osSemaphoreWait(dataAcqSem, 1000);
        if (res != osOK)
        {
            // Wait timeout expired. Something wrong wit the dacq system
            // Process timeout here
        }

        NSS_Toggle();
        SampleSensorsData();
        ApplyFactoryCalibration();

        ProcessUserCommands();
#if SHELL_ENABLE == true
        parse_json_cmd();
#endif
        parse_debug_cmd();
#ifndef OFFLINE_DEBUG
    //    SendContinuousPacket(dacqRate);
#endif
        //uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
        //printf("uxhigh=%d\r\n",TASKIMU_STACK-uxHighWaterMark);
    }
}
