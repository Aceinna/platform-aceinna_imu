/*****************************************************************************
 * @file   imu_data_acq_task.c
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
Copyright 2020 ACEINNA, INC

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

#include "sensorsAPI.h"
#include "calibrationAPI.h"
#include "configurationAPI.h"
#include "commAPI.h"
#include "uart.h"
#include "bsp.h"
#include "timer.h"

mcu_time_base_t imu_time;

/** ***************************************************************************
 * @name TaskDataAcquisition()
 * @brief Get the sensor data at the specified frequency (based on the
 *        configuration of the accelerometer rate-sensor). Process and provide
 *        information to the user via the UART.
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void TaskDataAcquisition(void const *argument)
{
    int res;

    res = InitSensors();
    InitSensorsData();

    while (1)
    {
        //UBaseType_t uxHighWaterMark;
        //uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);

        res = osSemaphoreWait(g_sem_imu_data_acq, 1000);
        if (res != osOK)
        {
            continue;
            // Wait timeout expired. Something wrong wit the data acq system
            // Process timeout here
        }
        imu_time = g_MCU_time;

        NSS_Toggle();
        SampleSensorsData();
        ApplyFactoryCalibration();

        /*
            Add the INS algorithm here
                double   accels[3];
                GetAccelData_g_AsDouble(accels);
                double rates[3];
                GetRateData_radPerSec_AsDouble(rates);
                imu_time:imu time

                INS_Algorithm();
        */
        
        ProcessUserCommands();

        SendContinuousPacket();

        //uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
        //printf("uxhigh=%d\r\n",TASK_IMU_DATA_ACQ_STACK-uxHighWaterMark);
    }
}

