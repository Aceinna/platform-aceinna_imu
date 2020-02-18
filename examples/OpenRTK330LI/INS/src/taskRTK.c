/** ***************************************************************************
 * @file taskRTK.c 
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *****************************************************************************/
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
* HISTORY***********************************************************************
* 16/10/2019  |                                             | Daich
* Description: package some _hal function
* 28/10/2019  |                                             | Daich
* Description: modify the position of "osSemaphoreWait(RTDStartSem, 1000)"
* 06/11/2019  | changed file name back to taskRTK.c         | DW
*******************************************************************************/
#include <math.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#include "stm32f4xx_hal.h"
#include "osapi.h"

#include "uart.h"
#include "led.h"
#include "timer.h"

#include "gpsAPI.h"
#include "UserConfiguration.h"
#include "taskRTK.h"
#ifndef _USE_PPP_
#else
#include "ppp_eng.h"
#endif
#include "osresources.h"

#if APP_MODE == WEB_APP
#include "ntripClient.h"
#endif


#include "taskDataAcquisition.h"

extern volatile mcu_time_base_t g_MCU_time;

gpsDataStruct_t gGnssSol = {0};
gpsDataStruct_t *gPtrGnssSol = &gGnssSol;




extern GpsData_t *gGpsDataPtr;

static void RTKAlgorithm(void);

#define MEAS_RUN_TIME

void RTKTask(void const *argument)
{
    UBaseType_t uxHighWaterMark;
    uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);

    uint8_t res = osSemaphoreWait(RTKStartSem, 0);
    while (1)
    {
        uint8_t res = osSemaphoreWait(RTKStartSem, 10);
        if (res != osOK)
        {
            osSemaphoreRelease(RTKFinishSem);
            continue;
            // Wait timeout expired. Something wrong wit the dacq system
            // Process timeout here
        }
        LED1_Toggle();

        /* FOR DEBUG, to get running time measure */
        time_t time_old = get_time_of_msec();
        RTKAlgorithm();
        osSemaphoreRelease(RTKFinishSem);

#ifndef DEBUG_INS
#ifdef nMEAS_RUN_TIME
        time_t time_new = get_time_of_msec();
        int32_t time_sub = (int32_t)(time_new - time_old);
        static char buf_test[256] = {0};
        // if (time_sub > 20)
        {
            // printf("time = %d\r\n",time_sub);
            int week = (int)floor(gRcvRtk.time / SECONDS_IN_WEEK);
            sprintf(buf_test, "time = %6ld,%10.3f,%12.3f,%3i,%3i,%3i,%3i,%3i,%3i,%3i\r\n", time_sub, gRcvRtk.tt, gRcvRtk.time - week * SECONDS_IN_WEEK * 1.0,
                    gGpsDataPtr->rov.n, gGpsDataPtr->ref.n, gGpsDataPtr->nav.n,
                    gGpsDataPtr->nav.n_gps, gGpsDataPtr->nav.ng, gGpsDataPtr->nav.n_gal, gGpsDataPtr->nav.n_bds);
            HAL_UART_Transmit_DMA(&huart_debug, (uint8_t *)buf_test, strlen((const char *)buf_test));
        }
#endif
#endif

        uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
        // printf("uxhigh=%d\r\n",TASKRTD_STACK-uxHighWaterMark);
        // OS_Delay(10);
    }
}


static void RTKAlgorithm(void)
{

}
