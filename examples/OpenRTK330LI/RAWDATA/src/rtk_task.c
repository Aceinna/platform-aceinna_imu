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

#include <string.h>

#include "gnss_data_api.h"
#include "uart.h"
#include "led.h"
#include "ntripClient.h"

extern gnss_raw_data_t *g_ptr_gnss_data;


static void RTKAlgorithm(void);
char gga_buff[200] = "$GPGGA,,,N,,E,,,,,M,,M,,*\r\n";

/** ***************************************************************************
 * @name RTKTask()
 * @brief  RTK Algorithm task
 * @param  N/A
 * @retval N/A
 ******************************************************************************/
void RTKTask(void const *argument)
{
    // UBaseType_t uxHighWaterMark;
    // uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);

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
        LED1_Toggle();

        RTKAlgorithm();

        //send gga from Bluetooth
        uart_write_bytes(UART_BT, gga_buff, strlen(gga_buff), 1);
        //send gga from Internet
        ntrip_push_tx_data((uint8_t*)gga_buff, strlen(gga_buff));

        osSemaphoreRelease(g_sem_rtk_finish);

        // uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
        // printf("uxhigh=%d\r\n",TASKRTD_STACK-uxHighWaterMark); 
    }
}

/** ***************************************************************************
 * @name RTKAlgorithm()
 * @brief  RTK Algorithm
 *         Rover and base data are stored in g_ptr_gnss_data
 *         nav_t *nav = &g_ptr_gnss_data->nav;
*          obs_t *rov = &g_ptr_gnss_data->rov;
 * @param  N/A
 * @retval N/A
 ******************************************************************************/
static void RTKAlgorithm(void)
{

}
