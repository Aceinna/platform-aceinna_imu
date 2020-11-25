/*****************************************************************************
 * @file   eth_task.c
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
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

#include "cmsis_os.h"
#include "osapi.h"
#include "FreeRTOS.h"
#include "station_tcp.h"
#include "aceinna_client_api.h"
#include "tcp_driver.h"
#include "calibrationAPI.h"

/** ***************************************************************************
 * @name EthTask()
 * @brief Embedded server,sending and receiving data from Internet
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void EthTask(void const *argument)
{
	ethernet_init(); // init ethernet, maybe delay util spp

    aceinna_client_init(GetUnitSerialNum());

    while (g_ptr_gnss_sol->gnss_fix_type != 1)
    {
        OS_Delay(1000);
    }
	
	while (1)
	{
#ifdef  USE_TCP_DRIVER
        driver_interface();
#endif
        station_tcp_interface();
	}
}


void TcpDriverTask(void const *argument)
{
    tcp_driver_fifo_init();
    tcp_driver_data_fifo_init();	
	while (1)
	{
#ifdef  USE_TCP_DRIVER
        driver_interface();
        driver_output_data_interface();
#endif
	}
}
