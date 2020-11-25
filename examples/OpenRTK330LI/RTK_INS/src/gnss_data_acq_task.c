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
#include "bt_packet.h"
#include "led.h"
#include "crc16.h"
#include "app_version.h"
#include "stdlib.h"
#include "ntrip_server.h"
#include "aceinna_client_api.h"
#include "station_tcp.h"
#include "tcp_driver.h"
#include "ins_interface_API.h"
#define CCMRAM __attribute__((section(".ccmram")))

extern volatile mcu_time_base_t g_MCU_time;
static uint16_t GpsRxLen;

extern uint8_t g_pps_flag;
extern uint8_t debug_com_log_on;

CCMRAM gnss_raw_data_t g_gnss_raw_data = {0};
gnss_raw_data_t *g_ptr_gnss_data = &g_gnss_raw_data;
uint8_t gnss_signal_flag = 0;   //1:Satellite signal availability

uint8_t stnID = 0;
volatile mcu_time_base_t g_obs_rcv_time;

char gga_buff[120] = {0};
char gsa_buff[500]  = "GSA\r\n";
char rmc_buff[200]  = "RMC\r\n";
char zda_buff[50] = "ZDA\r\n";
char nema_update_flag = 0;

fifo_type fifo_user_uart;
uint8_t fifo_user_uart_buf[2000];
uint8_t gnss_result_cnt = 0;
static uint8_t base_cnt = 0; 

gnss_solution_t g_gnss_sol = {0};
gnss_solution_t *g_ptr_gnss_sol = &g_gnss_sol;
// char gsv_buff[1000] = "GSV\r\n";
/** ***************************************************************************
 * @name _handleGpsMessages()
 * @brief Decode RTCM data and store it in g_gnss_raw_data
 * @param uint8_t *RtcmBuff:RTCM data flow
 *        int length:RTCM data lenth
 * @retval N/A
 ******************************************************************************/
static void _handleGpsMessages(uint8_t *RtcmBuff, int length)
{
    int pos = 0;
    gnss_rtcm_t *rtcm = &g_gnss_raw_data.rtcm;
    obs_t *obs = rtcm->obs + stnID;
    rtcm_t *rcv = rtcm->rcv + stnID;
    int8_t ret_val = 0;
    uint32_t crc = 0;

    while (length)
    {
        length--;
        ret_val = input_rtcm3(RtcmBuff[pos++], stnID, rtcm);
        /* relocated from rtcm.c */
        if (stnID == BASE && ret_val == 1) //Base station data reception completed
        {
            station_tcp_clear_stream_timeout();
            LED_RTCM_TOOGLE();
            base_cnt = 0;
        }

        if (rtcm_decode_completion == 1){
            if (stnID == ROVER) 
            {
                //printf("time: %lld, %lf\r\n", rtcm->rcv[ROVER].time.time, rtcm->rcv[ROVER].time.sec);
                // ntrip server push rtcm data
                if (get_station_mode() == MODE_NTRIP_SERVER && base_station_get_run_status() == 1) {
                    if (rtcm->rcv[ROVER].time.sec < 0.1)
                    {
                        if (rtcm->rcv[ROVER].type == 1077 || rtcm->rcv[ROVER].type == 1097 || rtcm->rcv[ROVER].type == 1117
                            || rtcm->rcv[ROVER].type == 1127 || rtcm->rcv[ROVER].type == 1087)
                        {
                            setbitu(rtcm->rcv[ROVER].buff, 36, 12, get_station_id());

                            crc = rtk_crc24q(rtcm->rcv[ROVER].buff, rtcm_decode_length - 3);
                            setbitu(rtcm->rcv[ROVER].buff, (rtcm_decode_length - 3) * 8, 24, crc);

                            station_tcp_send_data(rtcm->rcv[ROVER].buff, rtcm_decode_length);
                            // uart_write_bytes(UART_USER, (const char*)rtcm->rcv[ROVER].buff, rtcm_decode_length, 1);
                            station_tcp_clear_stream_timeout();

                            //printf("rtcm{%lld, %lf} %d\r\n", rtcm->rcv[ROVER].time.time, rtcm->rcv[ROVER].time.sec, rtcm->rcv[ROVER].type);
                        }
                    }
                }

                // else
                // {
                //     //printf("r %d\r\n", rtcm->rcv[ROVER].type);
                // }
            }
            rtcm_decode_completion = 0;
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
extern client_s driver_data_client;
static int input_nema_data(unsigned char data)
{
    static char nema_r[1000];
    static uint8_t frame_rev_flag = 0;
    
    static int num = 0;
    static int end_flag = 0;

    static uint8_t len_buf[10];
    static int8_t len_i = 0;    
    static int len = 0;
    if (data == '#'&&frame_rev_flag == 0)
    {
        memset(nema_r, 0, 1000);
        num = 0;
        end_flag = 0;
        frame_rev_flag = 1;
        return 0;
    }
    switch (frame_rev_flag)
    {
        case 1:
            if (data == 'n')
            {
                frame_rev_flag = 2;
            }
            else
            {
                frame_rev_flag = 0;
            }
            break;
        case 2:
            if (data == 'e')
            {
                frame_rev_flag = 3;
            }
            else
            {
                frame_rev_flag = 0;
            }
            break;
        case 3:
            if (data == 'm')
            {
                frame_rev_flag = 4;
            }
            else
            {
                frame_rev_flag = 0;
            }
            break;
        case 4:
            if (data == 'a')
            {
                frame_rev_flag = 5;
            }
            else
            {
                frame_rev_flag = 0;
            }
            break;
        case 5:

            len_buf[len_i++] = (uint8_t)data;
            if(len_i >= 5)
            {
                frame_rev_flag = 6;
                len = (len_buf[0]-48)*100 + (len_buf[1]-48)*10 + (len_buf[2]-48)*1;
                if(len < 0 || len > 5000)
                    len = 0;
                // sscanf(len_buf,"%d",&len);
                len_i = 0;
                //frame_rev_flag = 0;
                // uart_write_bytes(UART_DEBUG, (char *)data, strlen(data), 1);
            }
            break;
        case 6:
            nema_r[num++] = data;
            if(num == len-10)
            {
                frame_rev_flag = 0;
                char *ptr1,*ptr2,*ptr3;
                ptr1 = strstr(nema_r,"$GPRMC"); 
                ptr2 = strstr(nema_r,"$GPGSA");
                ptr3 = strstr(nema_r,"$GPZDA");

                if(ptr1!=NULL && ptr2!=NULL && ptr3!=NULL)
                {
                    memset(gga_buff,0,sizeof(gga_buff));
                    memset(rmc_buff,0,sizeof(rmc_buff));
                    memset(gsa_buff,0,sizeof(gsa_buff));
                    memset(zda_buff,0,sizeof(zda_buff));

                    strncpy(gga_buff,nema_r,ptr1-nema_r);
#ifdef DEBUG_ALL
                    //uart_write_bytes(UART_DEBUG,(char*)nema_r,num,1);
                    if (debug_com_log_on) {
                        uart_write_bytes(UART_DEBUG,gga_buff,strlen(gga_buff),1);
                    }
                    if(get_tcp_data_driver_state() == CLIENT_STATE_INTERACTIVE)
                    {
                        //driver_data_push(gga_buff,strlen(gga_buff));
                        client_write_data(&driver_data_client,gga_buff,strlen(gga_buff),0x01);
                    }
#endif
                    strncpy(rmc_buff,ptr1,ptr2-ptr1);
                    strncpy(gsa_buff,ptr2,ptr3-ptr2);

                    strncpy(zda_buff,ptr3,len-10-(ptr3-nema_r));
                    nema_update_flag = 1;

                    if (get_station_mode() == MODE_OPENARC_CLIENT) {
                        // aceinna_client_push_nmea(nema_r, len-10);
                        aceinna_client_push_nmea(gga_buff, ptr1-nema_r);
                    }

                    uart_write_bytes(UART_BT, (char *)gga_buff, strlen(gga_buff), 1);

                    if (get_station_mode() == MODE_NTRIP_CLIENT ||
                        (get_station_mode() == MODE_NTRIP_SERVER && base_station_get_run_status() == 3)) {
                        station_tcp_send_data((uint8_t *)gga_buff, strlen(gga_buff));
                        // printf("%s", gga_buff);
                    }
                }
            }
            break;
    }

    return 1;
}


uint8_t frame_data[2048];
static uint8_t crc_rev[2] = {0};
gnss_solution_t *gps_data_from_sta;
uint8_t point_one_lla_buf[32];


static int input_gnss_data(unsigned char data)
{
    static uint8_t frame_rev_flag = 0;
    static uint16_t frame_data_len = 0;

    static uint16_t data_rev_index = 0;

    static uint16_t crc_rev_index = 0;
    static uint64_t input_gnss_time[3];
    static uint8_t time_cnt = 0;
    if (frame_rev_flag == 0)
    {
        if (data == 's')
        {
            frame_rev_flag = 1;
            frame_data_len = 0;
            data_rev_index = 0;
            crc_rev_index = 0;
            return 0;
        }
    }
    switch (frame_rev_flag)
    {
    case 1:
        if (data == 't')
        {
            frame_rev_flag = 2;
        }
        else
        {
            frame_rev_flag = 0;
        }
        break;
    case 2:
        if (data == 'a')
        {
            frame_rev_flag = 3;
        }
        else
        {
            frame_rev_flag = 0;
        }
        break;
    case 3:
        frame_data_len = data;
        frame_rev_flag = 4;
        break;
    case 4:
        frame_data_len = ((uint16_t)data << 8) + frame_data_len;
        if(frame_data_len > 1200)
        {
            frame_rev_flag = 0;
            frame_data_len = 0;
            data_rev_index = 0;
            crc_rev[0] = 0;
            crc_rev[1] = 0;
            crc_rev_index = 0;
        }
        else
        {
            frame_rev_flag = 5;
        }
        break;
    case 5:
        frame_data[data_rev_index++] = data;
        if (data_rev_index == frame_data_len - 5 - 2)
        {
            frame_rev_flag = 6;
        }
        break;
    case 6:
        crc_rev[crc_rev_index++] = data;
        if (crc_rev_index == 2)
        {
            uint16_t crc_check = CalculateCRC(frame_data, frame_data_len - 5 - 2);
            //printf("crc_ check = %x,crc_rev[0] = %x,crc_rev[1] = %x\r\n",crc_check,crc_rev[0],crc_rev[1]);
            gps_data_from_sta = (gnss_solution_t *)(frame_data);
            if (crc_check == ((crc_rev[1] << 8) | crc_rev[0]))
            {
                // printf("gps_data_from_sta->gps_week = %d\r\n",gps_data_from_sta->gps_week);
                // printf("gps_data_from_sta->gps_tow = %d\r\n",gps_data_from_sta->gps_tow);
                // printf("gps_data_from_sta->latitude = %lf\r\n",gps_data_from_sta->latitude);
                // printf("gps_data_from_sta->longitude = %lf\r\n",gps_data_from_sta->longitude);
                gnss_result_cnt ++;
                if(gnss_result_cnt >= 100)
                    gnss_result_cnt = 100;
                    
                if (gps_data_from_sta->gps_week > 0 && gps_data_from_sta->height >= -1000 &&
                    gps_data_from_sta->latitude * RAD_TO_DEG >= -90.0 && gps_data_from_sta->latitude * RAD_TO_DEG <= 90.0 &&
                    gps_data_from_sta->longitude * RAD_TO_DEG >= -180.0 && gps_data_from_sta->longitude * RAD_TO_DEG <= 180.0) {
                    g_gnss_sol = *gps_data_from_sta;

                    copy_gnss_result(&g_gnss_sol);

                    if (g_gnss_sol.gnss_fix_type == 4 || g_gnss_sol.gnss_fix_type == 5 || g_gnss_sol.gnss_fix_type == 1)
                    {
                        gtime_t gt = gpst2time(g_gnss_sol.gps_week, g_gnss_sol.gps_tow * 0.001);


                        if (time_cnt >=3)
                        {
                            input_gnss_time[0] = input_gnss_time[1];
                            input_gnss_time[1] = input_gnss_time[2];
                            input_gnss_time[2] = g_gnss_sol.gps_tow;
                        }
                        else
                        {
                            input_gnss_time[time_cnt] = g_gnss_sol.gps_tow;
                            time_cnt ++;
                        }
                        
                        // if(gt.time-g_MCU_time.time>2 || gt.time-g_MCU_time.time<-2)
                        // {
                        //     static char ggaBuff[100];
                        //     sprintf(ggaBuff,"time = %lld,week = %d,gps_tow = %d\r\n",gt.time,g_gnss_sol.gps_week,g_gnss_sol.gps_tow);
                        //     uart_write_bytes(UART_DEBUG, ggaBuff, strlen(ggaBuff), 1);                        
                        // }
                        if(input_gnss_time[2] - input_gnss_time[1] == 1000 && input_gnss_time[1]-input_gnss_time[0] == 1000)
                            g_MCU_time.time = gt.time;

#ifndef ACEINNA_CLIENT_PO_TEST_POS
                        if (get_station_mode() == MODE_OPENARC_CLIENT) {
                            int32_t lat = g_gnss_sol.latitude * RAD_TO_DEG * 1e+07;
                            int32_t lon = g_gnss_sol.longitude * RAD_TO_DEG * 1e+07;
                            int32_t alt = g_gnss_sol.height * 1e+03;

                            //printf("[%lf %lf %lf]\r\n", g_gnss_sol.latitude*RAD_TO_DEG, g_gnss_sol.longitude*RAD_TO_DEG, g_gnss_sol.height);
                            uint16_t lla_mesg_len = aceinna_client_po_lla_message(lat, lon, alt, point_one_lla_buf);
                        
                            station_tcp_send_data(point_one_lla_buf, lla_mesg_len);
                        }
#endif

                        if (get_station_mode() == MODE_NTRIP_SERVER && get_base_position_type() != BASE_POSITION_REFERENCE) {
                            station_position_calc(g_gnss_sol.latitude, g_gnss_sol.longitude, g_gnss_sol.height, g_gnss_sol.gnss_fix_type);
                        }
                        
                    }
                    // g_gnss_sol.gnss_update = 1;
                    LED1_Toggle();
                    if (g_gnss_sol.gnss_fix_type != 0)
                    {
                        g_status.status_bit.gnss_signal_status = 1;
                    }
                    else
                    {
                        g_status.status_bit.gnss_signal_status = 0;
                    }   
                } else {
                    // printf("err %d %lf %lf %lf\r\n", gps_data_from_sta->gps_week, gps_data_from_sta->latitude * RAD_TO_DEG, gps_data_from_sta->longitude * RAD_TO_DEG, gps_data_from_sta->height);
                }
            }
            frame_rev_flag = 0;
            frame_data_len = 0;
            data_rev_index = 0;
            crc_rev[0] = 0;
            crc_rev[1] = 0;
            crc_rev_index = 0;
        }
        break;
    }
    return 1;
}

#pragma pack(1)
typedef struct rtcm_filter_t_
{
    char frame_head[6];
    unsigned char rtcm_len[4];
    char buff[1500];
}rtcm_filter_t;
#pragma pack()

extern client_s driver_data_client;
static int input_rover_data(unsigned char data)
{
    static unsigned char buff_get[1300];
    static uint8_t frame_rev_flag = 0;
    static uint8_t frame_data_len[4] = {0};
    static uint8_t frame_data_index = 0;
    static uint16_t data_rev_index = 0;
    static rtcm_filter_t rtcm_data_rev;
    memcpy(rtcm_data_rev.frame_head,"#GNROV",strlen("#GNROV"));
    static uint32_t len = 0 ;
    if (frame_rev_flag == 0)
    {
        if (data == 'r')
        {
            frame_rev_flag = 1;
            return 0;
        }
    }
    switch (frame_rev_flag)
    {
    case 1:
        if (data == 'o')
        {
            frame_rev_flag = 2;
        }
        else
        {
            frame_rev_flag = 0;
        }
        break;
    case 2:
        if (data == 'v')
        {
            frame_rev_flag = 3;
        }
        else
        {
            frame_rev_flag = 0;
        }
        break;
    case 3:
        if (data == 'e')
        {
            frame_rev_flag = 4;
        }
        else
        {
            frame_rev_flag = 0;
        }
        break;
    case 4:
        if (data == 'r')
        {
            frame_rev_flag = 5;
        }
        else
        {
            frame_rev_flag = 0;
        }
        break;
    case 5:
        frame_data_len[frame_data_index] = data;
        rtcm_data_rev.rtcm_len[frame_data_index++] = data;
        if(frame_data_index >= 4)
        {
            frame_data_index = 0;
            frame_rev_flag = 6;
            len = frame_data_len[0] + \
                             (frame_data_len[1] << 8) + \
                             (frame_data_len[2] << 16) + \
                             (frame_data_len[3] << 24);
            if((len <= 0) || (len > 1200))
            {
                frame_rev_flag = 0;
                memset(frame_data_len,0,4);
                frame_data_index = 0;
                data_rev_index = 0;                
            }
        }
        break;
    case 6:
        rtcm_data_rev.buff[data_rev_index++] = data;
        if (data_rev_index >= len)
        {
            if (get_station_mode() == MODE_NTRIP_SERVER && base_station_get_run_status() == 1) {

                stnID = ROVER;
                _handleGpsMessages((uint8_t*)rtcm_data_rev.buff, len);
            }

#ifdef DEBUG_ALL
            uint8_t rover_data_buf[2000] = {0};
            //uint32_t len_to_debug = len + sizeof(rtcm_data_rev.frame_head) + sizeof(rtcm_data_rev.rtcm_len);
            uint32_t rtcm_len = len;
            uint32_t data_len = rtcm_len + 5*sizeof(char) + 1*sizeof(char);      //sizeof(",%02x\r\n") 5 sizeof(',') 1
            double gga_time = get_gnss_time();
            uint32_t head_len = sprintf((const char*)rover_data_buf,"$GPROV,%6.2f,%04u,",gga_time,data_len);
            memcpy(rover_data_buf + head_len,rtcm_data_rev.buff,rtcm_len);
            uint32_t all_bytes_to_sum = head_len + rtcm_len;
            char sum = 0;
            for(uint32_t i = 0;i < all_bytes_to_sum;i++)
            {
                sum ^= rover_data_buf[i];
            }
            uint32_t end_len = sprintf((const char*)rover_data_buf + head_len + rtcm_len,",%02x\r\n",sum);
            if (debug_com_log_on) {
                uart_write_bytes(UART_DEBUG,(const char*)rover_data_buf,head_len + rtcm_len + end_len,1);
            }
            if(get_tcp_data_driver_state() == CLIENT_STATE_INTERACTIVE)
            {
                //driver_data_push((const char*)rover_data_buf,head_len + rtcm_len + end_len);
                client_write_data(&driver_data_client,(const char*)rover_data_buf,head_len + rtcm_len + end_len,0x01);
            }
#endif

            frame_rev_flag = 0;
            memset(frame_data_len,0,4);
            frame_data_index = 0;
            data_rev_index = 0;
        }
        break;
    }
    return 1;
}


static void parse_gnss_data(uint8_t *RtcmBuff, int length)
{
    int pos = 0;
    while (length)
    {
        length--;
        int ret_gga = input_nema_data(RtcmBuff[pos]);
        int ret_rtcm = input_rover_data(RtcmBuff[pos]);
        int ret = input_gnss_data(RtcmBuff[pos++]);
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
uint8_t Gpsbuf[GPS_BUFF_SIZE];
uint8_t bt_buff[GPS_BUFF_SIZE];
extern uint32_t debug_p1_log_delay;
void GnssDataAcqTask(void const *argument)
{
    int ret = 0;

    memset(g_ptr_gnss_data, 0, sizeof(gnss_raw_data_t));

    while (1)
    {
        // UBaseType_t uxHighWaterMark;
        // uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
        base_cnt ++;
        if(base_cnt > 200)
        {
            /* If no base station signal, RED LED is off */
            LED_RTCM_OFF;  
        }


        if (!station_tcp_is_interactive()) {
            int BtLen = 0;
                BtLen = uart_read_bytes(UART_BT, bt_buff, GPS_BUFF_SIZE, 0);
            if(BtLen == 0)
                BtLen = fifo_get(&fifo_user_uart,bt_buff,GPS_BUFF_SIZE);
            if (BtLen)
            {
                if(uart_sem_wait(UART_BT,0) == RTK_SEM_OK)
                {
                    ret = bt_uart_parse(bt_buff);
                }
                if (ret != RTK_JSON && gnss_result_cnt >= 5)
                {
                    //send base from UART_GPS port
                    uart_write_bytes(UART_GPS, (char*)bt_buff, BtLen, 1);

#ifdef DEBUG_BASE_RTCM
                    if (!debug_p1_log_delay) {
                        uart_write_bytes(UART_DEBUG, (char*)bt_buff, BtLen, 1);
                    }
#endif // DEBUG

                    stnID = BASE;
                    _handleGpsMessages(bt_buff, BtLen);
                }
            }
        }
        else
        {
            int ethRxLen = 0;
            ethRxLen = station_tcp_get_data(bt_buff, GPS_BUFF_SIZE);
            
            if (ethRxLen && gnss_result_cnt >= 5)
            {
                if (ethRxLen > 500) {
#ifdef  STATION_TCP_DEBUG
    printf("NET: get data len %d\r\n", ethRxLen);
#endif
                }
                //send base from UART_GPS port
                uart_write_bytes(UART_GPS, (char*)bt_buff, ethRxLen, 1);

#ifdef DEBUG_BASE_RTCM
                if (!debug_p1_log_delay) {
                    uart_write_bytes(UART_DEBUG, (char*)bt_buff, ethRxLen, 1);
                }
#endif // DEBUG

                stnID = BASE;
                _handleGpsMessages(bt_buff, ethRxLen);
            }
        }

        update_fifo_in(UART_GPS);
        GpsRxLen = uart_read_bytes(UART_GPS, Gpsbuf, GPS_BUFF_SIZE, 0);         

        static uint8_t app_flag = 0;
        if (app_flag == 0)
        {
            char buff_to_spilt[30];
            memcpy(buff_to_spilt,Gpsbuf,GpsRxLen > 30?30:GpsRxLen);
            if (strstr((char *)APP_VERSION_STRING, "RAWDATA"))
            {
                char app_buf[10] = "raw";
                uart_write_bytes(UART_GPS, app_buf, strlen(app_buf), 1);
                if (strstr((char *)Gpsbuf, "raw"))
                {
                    sscanf((const char*)buff_to_spilt,"%*[^/]/%s",st_sdk_version);
                    HAL_GPIO_WritePin(GPIOC,ST_PROG_BUF_CTL_PIN,GPIO_PIN_SET);
                    app_flag = 1;
                }
            }
            else
            {
                char app_buf[10] = "rtk";
                uart_write_bytes(UART_GPS, app_buf, strlen(app_buf), 1);
                if (strstr((char *)Gpsbuf, "rtk") || strstr((char *)Gpsbuf, "ins"))
                {
                    sscanf((const char*)buff_to_spilt,"%*[^/]/%s",st_sdk_version);
                    HAL_GPIO_WritePin(GPIOC,ST_PROG_BUF_CTL_PIN,GPIO_PIN_RESET);
                    app_flag = 1;
                }
            }
        }
        static uint8_t no_rx = 0;
        if (GpsRxLen) //TODO:
        {
            // stnID = ROVER;
            // _handleGpsMessages(Gpsbuf, GpsRxLen);
            // uart_write_bytes(UART_BT,(char *)Gpsbuf,GpsRxLen,1);

            parse_gnss_data(Gpsbuf, GpsRxLen);
            no_rx = 0;
        }
        else
        {
            no_rx++;
        }

        if (no_rx >= 250)
        {
            g_status.status_bit.gnss_data_status = 0;
            g_status.status_bit.gnss_signal_status = 0;
            no_rx = 250;
        }
        else
        {
            g_status.status_bit.gnss_data_status = 1;
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

