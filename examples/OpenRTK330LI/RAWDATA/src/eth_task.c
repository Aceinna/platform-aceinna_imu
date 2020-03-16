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

#include "FreeRTOS.h"
#include "lwip_comm.h"
#include "ntrip_client.h"


/** ***************************************************************************
 * @name EthTask()
 * @brief Embedded server,sending and receiving data from Internet
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void EthTask(void const *argument)
{
	fifo_init(&ntrip_tx_fifo, ntripTxBuf, NTRIP_TX_BUFSIZE);
	fifo_init(&ntrip_rx_fifo, ntripRxBuf, NTRIP_RX_BUFSIZE);

	ethernet_init(); // init ethnernet
	
	while (1)
	{
        NTRIP_interface();
	}
}
