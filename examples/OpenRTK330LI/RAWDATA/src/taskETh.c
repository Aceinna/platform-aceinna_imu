/*****************************************************************************
 * @file   taskEth.c
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
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
#include "taskEth.h"
#include "stm32f4xx_hal.h"
#include "osapi.h"
#include "osresources.h"
#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "LwipComm.h"
#include "ntripClient.h"
#include "RingBuffer.h"

#include "uart.h"
#include "cJSON.h"
#include "UserConfiguration.h"
#include "shell.h"

#define DEBUG_UART_RX_SIZE 256
// "{\"username\":\"ymj_123\",\"password\":\"SIGEMZOOMQ1JDJI3\",\"ntripType\":\"NTRIPWAY_BLUETOOTH\",\"rtkType\":\"LocalRTK\",\"ip\":[106, 12, 40, 121],\"port\":2201,\"mountPoint\":\"/RTK\"}";
uint8_t debug_rx_buf[DEBUG_UART_RX_SIZE];
uint16_t debug_rx_len = 0;

static void debug_uart_ntrip_setting(void)
{
    cJSON *item_username, *item_password, *item_rtkType;
    cJSON *item_ip, *item_port, *item_mountPoint;
    cJSON *root;

#if SHELL_ENABLE == false
    //debug_rx_len = uart_read_bytes(UART_DEBUG, debug_rx_buf, DEBUG_UART_RX_SIZE, 0);
#endif
    if (debug_rx_len)
    {
        debug_rx_buf[debug_rx_len] = 0;
        root = cJSON_Parse((const char *)debug_rx_buf);
        if (root != NULL)
        {
            item_rtkType = cJSON_GetObjectItem(root, "rtkType");
            item_ip = cJSON_GetObjectItem(root, "ip");
            item_port = cJSON_GetObjectItem(root, "port");
            item_mountPoint = cJSON_GetObjectItem(root, "mountPoint");
            item_username = cJSON_GetObjectItem(root, "username");
            item_password = cJSON_GetObjectItem(root, "password");

            if (item_username != NULL && item_password != NULL && item_rtkType != NULL && item_ip != NULL && item_port != NULL && item_mountPoint != NULL)
            {
                strcpy((char *)gUserConfiguration.username, (const char *)item_username->valuestring);
                strcpy((char *)gUserConfiguration.password, (const char *)item_password->valuestring);

                if (!strcmp(item_rtkType->valuestring, "LocalRTK"))
                {
                    gUserConfiguration.rtkType = LocalRTK;
                }
                else if (!strcmp(item_rtkType->valuestring, "CloudRTK"))
                {
                    gUserConfiguration.rtkType = CloudRTK;
                }

                int urlLen = cJSON_GetArraySize(item_ip);
                if (urlLen == 4)
                {
                    gUserConfiguration.ip[0] = cJSON_GetArrayItem(item_ip, 0)->valueint;
                    gUserConfiguration.ip[1] = cJSON_GetArrayItem(item_ip, 1)->valueint;
                    gUserConfiguration.ip[2] = cJSON_GetArrayItem(item_ip, 2)->valueint;
                    gUserConfiguration.ip[3] = cJSON_GetArrayItem(item_ip, 3)->valueint;
                }

                gUserConfiguration.port = item_port->valueint;
                strcpy((char *)gUserConfiguration.mountPoint, (const char *)item_mountPoint->valuestring);

                // printf("%s\r\n", item_username->valuestring);
                // printf("%s\r\n", item_password->valuestring);
                // printf("%s\r\n", item_rtkType->valuestring);
                // printf("%d.%d.%d.%d\r\n", cJSON_GetArrayItem(item_ip, 0)->valueint, cJSON_GetArrayItem(item_ip, 1)->valueint,
                //        cJSON_GetArrayItem(item_ip, 2)->valueint, cJSON_GetArrayItem(item_ip, 3)->valueint);
                // printf("%d\r\n", item_port->valueint);
                // printf("%s\r\n", item_mountPoint->valuestring);

                SaveUserConfig();

                NTRIP_client_start = NTRIP_START_ON;
            }

            cJSON_Delete(root);
        }
    }
}

void EthTask(void const *argument)
{
	FifoInit(&ntrip_tx_fifo, ntripTxBuf, NTRIP_TX_BUFSIZE);
	FifoInit(&ntrip_rx_fifo, ntripRxBuf, NTRIP_RX_BUFSIZE);

	ethernet_init(); // init ethnernet

	while (1)
	{
        debug_uart_ntrip_setting();
        NTRIP_interface();
	}
}
