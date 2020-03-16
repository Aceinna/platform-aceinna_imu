/** ***************************************************************************
 * @file gnss_data_acq_task.c handle GPS data, make sure the GPS handling function gets
 *  called on a regular basis
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
limitations under the License.
************************************************************************/
#include <string.h>

#include "gnss_data_api.h"
#include "uart.h"
#include "ntrip_client.h"
#include "led.h"

#define CCMRAM __attribute__((section(".ccmram")))

extern volatile mcu_time_base_t g_MCU_time;
static uint16_t GpsRxLen;

extern uint8_t g_pps_flag;

CCMRAM gnss_raw_data_t g_gnss_data = {0};
gnss_raw_data_t *g_ptr_gnss_data = &g_gnss_data;
uint8_t gnss_signal_flag = 0;   //1:Satellite signal availability

uint8_t stnID = 0;
volatile mcu_time_base_t g_obs_rcv_time;

/** ***************************************************************************
 * @name _handleGpsMessages()
 * @brief Decode RTCM data and store it in g_gnss_data
 * @param uint8_t *RtcmBuff:RTCM data flow
 *        int length:RTCM data lenth
 * @retval N/A
 ******************************************************************************/
static void _handleGpsMessages(uint8_t *RtcmBuff, int length)
{
    int pos = 0;
    gnss_rtcm_t *rtcm = &g_gnss_data.rtcm;
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
            clear_ntrip_stream_count();
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
                }
                else
                {
                    timeCpy = obs->time;
                    if (obs->n > 10)
                        gnss_signal_flag = 1;
                }
                uint8_t res = osSemaphoreWait(g_sem_rtk_finish, 0);
                if (res == osOK && gnss_signal_flag)
                {
                        g_ptr_gnss_data->rov = *obs;
                        g_ptr_gnss_data->ref = *(rtcm->obs + BASE);
                        g_ptr_gnss_data->nav = rtcm->nav;
                        osSemaphoreRelease(g_sem_rtk_start);
                }
            }
        }
    }
}
/** ***************************************************************************
 * @name GnssDataAcqTask()
 * @brief  Satellite data acquisition task,include base and rover
 *         Get the base data from UART_GPS,and get the base data from UART_BT or
 *         ntrip_rx_fifo.
 * @param  N/A
 * @retval N/A
 ******************************************************************************/
void GnssDataAcqTask(void const *argument)
{
    int ret = 0;
    uint8_t Gpsbuf[GPS_BUFF_SIZE];
    uint8_t bt_buff[GPS_BUFF_SIZE];

    memset(g_ptr_gnss_data, 0, sizeof(gnss_raw_data_t));

    while (1)
    {
        // UBaseType_t uxHighWaterMark;
        // uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
        if (!is_ntrip_interactive())
        {
            int BtLen = 0;
            BtLen = uart_read_bytes(UART_BT, bt_buff, GPS_BUFF_SIZE, 0);

            if (BtLen)
            {
                if(uart_sem_wait(UART_BT,0) == RTK_SEM_OK)
                {
                    //ret = bt_uart_parse(bt_buff);
                }
                if (ret != RTK_JSON)
                {
                    stnID = BASE;
                    _handleGpsMessages(bt_buff, BtLen);
                    //send base from DEBUG port
                    uart_write_bytes(UART_DEBUG, (char*)bt_buff, BtLen, 1);
                }
            }
        }
        else
        {
            int ethRxLen = 0;
            ethRxLen = fifo_get(&ntrip_rx_fifo, bt_buff, GPS_BUFF_SIZE);
            if (ethRxLen)
            {
                stnID = BASE;
                _handleGpsMessages(bt_buff, ethRxLen);
                //send base from DEBUG port
                uart_write_bytes(UART_DEBUG, (char*)bt_buff, ethRxLen, 1);
            }
        }

        update_fifo_in(UART_GPS);
        GpsRxLen = uart_read_bytes(UART_GPS, Gpsbuf, GPS_BUFF_SIZE, 0);         
        if (GpsRxLen)
        {
            stnID = ROVER;
            _handleGpsMessages(Gpsbuf, GpsRxLen);
        }
        // uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
        // printf("uxhigh=%d\r\n",TASK_GNSS_DATA_STACK-uxHighWaterMark);
        OS_Delay(10);
    }
}

/** ***************************************************************************
 * @name get_obs_time()
 * @brief  Acquisition of satellite time
 * @param  N/A
 * @retval N/A
 ******************************************************************************/
time_t get_obs_time()
{
    return g_ptr_gnss_data->rtcm.obs[0].time.time;
}

/** ***************************************************************************
 * @name get_gnss_signal_flag()
 * @brief  obtain get_gnss_signal_flag
 * @param  N/A
 * @retval N/A
 ******************************************************************************/
uint8_t get_gnss_signal_flag()
{
    return gnss_signal_flag;
}
