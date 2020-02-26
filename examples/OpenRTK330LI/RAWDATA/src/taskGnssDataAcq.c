/** ***************************************************************************
 * @file taskGnssDataAcq.c handle GPS data, make sure the GPS handling function gets
 *  called on a regular basis
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
* 17/10/2019  |                                             | Daich
* Description: new line: update_fifo_in(UART_GPS);
               remove gpsUartBuf etc.
* 06/01/2020  |                                             | Daich
* Description: parse bt data
*******************************************************************************/

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>

#include "stm32f4xx_hal.h"
#include "osapi.h"
#include "gpsAPI.h"
#include "uart.h"
#include "rtklib_core.h"
#include "timer.h"
#include "ntripClient.h"
#include "UserConfiguration.h"
#include "led.h"
#include "shell.h"



//#define ETH_ROVER			//default:close

#define CCMRAM __attribute__((section(".ccmram")))

extern volatile mcu_time_base_t g_MCU_time;
extern osMutexId bt_mutex;
static uint16_t GpsRxLen;
static uint16_t CloudRxLen;

extern uint8_t g_pps_flag;

CCMRAM GpsData_t gGpsData = {0};
GpsData_t *gGpsDataPtr = &gGpsData;
uint8_t gnss_signal_flag = 0;
uint8_t g_next_time_available = 0; //0:not available 1:available 2:uncertain

uint8_t stnID = 0;
volatile mcu_time_base_t g_obs_rcv_time;

/* output GGA to COM2 to request RTK data */
static void _handleGpsMessages(u8 *RtcmBuff, int length)
{
    int pos = 0;
    gnss_rtcm_t *rtcm = &gGpsData.rtcm;
    obs_t *obs = rtcm->obs + stnID;
    rtcm_t *rcv = rtcm->rcv + stnID;

    int8_t ret_val = 0;
    static uint8_t base_cnt = 0; 

    base_cnt ++;
    if(base_cnt > 200)
    {
        LED_RTCM_OFF;   //No base station signal, led2 off
    }
    while (length)
    {
        length--;
        ret_val = input_rtcm3(RtcmBuff[pos++], stnID, rtcm);
        /* relocated from rtcm.c */
        if (stnID == BASE && ret_val == 1) //Base station data reception completed
        {
            //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
            ntripStreamCount = 0;
            LED_RTCM_TOOGLE();
            base_cnt = 0;
        }

        if (ret_val == 1)
        {
            if (g_pps_flag == 0)
            {
                g_MCU_time.time = obs->time.time;
                g_MCU_time.msec = (obs->time.sec * 1000);
            }

            if (obs->pos[0] == 0.0 || obs->pos[1] == 0.0 || obs->pos[2] == 0.0)
            {
                /* do not output */
            }
            else if (stnID == ROVER)
            {
                g_obs_rcv_time = g_MCU_time;
                static gtime_t timeCpy;
                if ((timeCpy.sec == obs->time.sec && timeCpy.time == obs->time.time) || obs->n < 4)
                {
                    gnss_signal_flag = 0;
                    g_next_time_available = 0;
                }
                else
                {
                    timeCpy = obs->time;
                    if (obs->n > 10)
                        gnss_signal_flag = 1;
                }
                uint8_t res = osSemaphoreWait(RTKFinishSem, 0);
                if (res == osOK && gnss_signal_flag)
                {
                        gGpsDataPtr->rov = *obs;
                        gGpsDataPtr->ref = *(rtcm->obs + BASE);
                        gGpsDataPtr->nav = rtcm->nav;
                        osSemaphoreRelease(RTKStartSem);
                        g_next_time_available = 1;
                }
                // static char buff[100];
                // volatile double mcuTime = 0;
                // mcuTime = obs->time.sec + obs->time.time;
                // sprintf(buff,"%f\r\n",mcuTime);
                // uart_write_bytes(UART_USER, (char *)&buff, sizeof(buff), 1);
            }
        }
    }
}
/* end _handleGpsMessages */
void GnssDataAcqTask(void const *argument)
{
    uint8_t Gpsbuf[GPS_BUFF_SIZE];
    uint8_t bt_buff[GPS_BUFF_SIZE];
    int ret = 0;
    while (1)
    {

        UBaseType_t uxHighWaterMark;
        uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
#ifndef ETH_ROVER
        if (NTRIP_client_state != NTRIP_STATE_INTERACTIVE)
        {
            int BtLen = 0;
#if SHELL_ENABLE == true
            osMutexWait(bt_mutex, portMAX_DELAY);
#endif
            BtLen = uart_read_bytes(UART_BT, bt_buff, GPS_BUFF_SIZE, 0);

#if SHELL_ENABLE == true
            osMutexRelease(bt_mutex);
#endif
            if (BtLen)
            {

#if SHELL_ENABLE == true
                bt_uart_parse(bt_buff);
#endif
                if(uart_sem_wait(UART_BT,0) == RTK_SEM_OK)
                {
                    ret = bt_uart_parse(bt_buff);
                }
                if (ret != RTK_JSON)
                {
#ifdef OFFLINE_DEBUG
                    uart_write_bytes(UART_DEBUG, bt_buff, BtLen, 1);
#endif
                    stnID = BASE;
                    _handleGpsMessages(bt_buff, BtLen);
                }
            }
        }
        else
        {
            int ethRxLen = 0;
            ethRxLen = FifoGet(&ntrip_rx_fifo, bt_buff, GPS_BUFF_SIZE);
            if (ethRxLen)
            {
                if (NTRIP_base_stream == BSAE_ON)
                {
#ifdef OFFLINE_DEBUG
                    uart_write_bytes(UART_DEBUG, bt_buff, ethRxLen, 1);
#endif
                    stnID = BASE;
                    _handleGpsMessages(bt_buff, ethRxLen);
                }
            }
        }

#ifndef CES_DEMO
        update_fifo_in(UART_GPS);
        GpsRxLen = uart_read_bytes(UART_GPS, Gpsbuf, GPS_BUFF_SIZE, 0);
#else
        update_fifo_in(UART_DEBUG);
        GpsRxLen = uart_read_bytes(UART_DEBUG, Gpsbuf, GPS_BUFF_SIZE, 0);    
#endif            
        if (GpsRxLen)
        {
            stnID = ROVER;
            _handleGpsMessages(Gpsbuf, GpsRxLen);
        }
#else
        if (NTRIP_client_state == NTRIP_STATE_INTERACTIVE)
        {
            int BtLen = uart_read_bytes(UART_BT, bt_buff, GPS_BUFF_SIZE, 0);
            if (BtLen)
            {
                if (ret != RTK_JSON)
                {
                    stnID = BASE;
                    _handleGpsMessages(bt_buff, BtLen);
                }
            }
            int ethRxLen = 0;
            GpsRxLen = FifoGet(&ntrip_rx_fifo, Gpsbuf, GPS_BUFF_SIZE);
        }
        static int base_no_rev_flag = 0;
        if (GpsRxLen)
        {
            stnID = ROVER;
            _handleGpsMessages(Gpsbuf, GpsRxLen);
        }
        else 
        {
            base_no_rev_flag ++;
            if(base_no_rev_flag == 50)
            {
                #define test_str "$GPGGA\r\n"
                uart_write_bytes(UART_BT,test_str,strlen(test_str),1);
                base_no_rev_flag = 0;
                OS_Delay(1000);
            }
        }
#endif
        uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
        // printf("uxhigh=%d\r\n",TASKGPS_STACK-uxHighWaterMark);
        OS_Delay(10);
    }
}

void initGPSDataStruct(void)
{
    memset(gGpsDataPtr, 0, sizeof(GpsData_t));

    gGpsData.GPSbaudRate = 460800;
    gGpsData.GPSProtocol = RTCM3;
}

time_t get_obs_time()
{
    return gGpsDataPtr->rtcm.obs[0].time.time;
}

uint8_t get_g_next_time_available()
{
    return g_next_time_available;
}

uint8_t get_gnss_signal_flag()
{
    return gnss_signal_flag;
}
