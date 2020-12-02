/** ***************************************************************************
 * @file rtk_task.c 
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *****************************************************************************/
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
limitations under the License.*/


#include "main.h"
#include "gnss_data_api.h"
#include "rtk_api.h"


/** ***************************************************************************
 * @name RTKTask()
 * @brief RTK Algorithm task
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void RTKTask(void const *argument)
{
    // UBaseType_t uxHighWaterMark;
    // uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
    obs_t* ptr_rover_obs = &g_ptr_gnss_data->rtcm.obs[0];

    uint8_t res = osSemaphoreWait(g_sem_rtk_start, 0);
    while (1)
    {
        uint8_t res = osSemaphoreWait(g_sem_rtk_start, 10);
        if (res != osOK)
        {
            osSemaphoreRelease(g_sem_rtk_finish);
            continue;
            /* Wait timeout expired. Something wrong wit the dacq system
            Process timeout here */
        }
        // LED1_Toggle();

        // rtk_algorithm();

        // use rover obs to make gga and pull base rtcm data
        // print_pos_gga_util(ptr_rover_obs->time, ptr_rover_obs->pos, ptr_rover_obs->n, 1, 1.0, 0.0, gga_buff);
        
        //uart_write_bytes(UART_DEBUG, gga_buff, strlen(gga_buff), 1);
        //send gga from Bluetooth
        // uart_write_bytes(UART_BT, gga_buff, strlen(gga_buff), 1);
        //send gga from Internet
        // ntrip_push_tx_data((uint8_t*)gga_buff, strlen(gga_buff));

        osSemaphoreRelease(g_sem_rtk_finish);

    }
}
