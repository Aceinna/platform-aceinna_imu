/*****************************************************************************
 * @file   imu_data_acq_task.c
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
#include <string.h>

#include "sensorsAPI.h"
#include "calibrationAPI.h"
#include "configurationAPI.h"
#include "commAPI.h"
#include "uart.h"
#include "bsp.h"
#include "timer.h"
#include "user_message.h"
#include "car_data.h"
#include "gnss_data_api.h"
#include "ins_interface_API.h"


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
    memset(&g_status,0,sizeof(status_t));
    g_status.status_bit.power = 1;
    g_status.status_bit.MCU_status = 1;

    while (1) {

        res = osSemaphoreWait(g_sem_imu_data_acq, 1000);
        if (res != osOK)
        {
            // Wait timeout expired. Something wrong wit the data acq system
            // Process timeout here
        }


        DRDY_OFF();
        
        ins_gnss_time_update();
        SampleSensorsData();
        ApplyFactoryCalibration();

        /* GNSS/INS fusion */
        ins_fusion();

        ProcessUserCommands();

        debug_com_process();

        /* solution packets output from UART*/
        send_continuous_packet();

        /* solution packets output from bluetooth*/
        send_ins_to_bt();

    }
}

