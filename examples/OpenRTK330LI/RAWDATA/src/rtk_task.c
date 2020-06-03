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
#include "ntrip_client.h"
#include "utils.h"
#include "nav_math.h"

extern gnss_raw_data_t *g_ptr_gnss_data;

gnss_solution_t g_gnss_sol = {0};

char gga_buff[200] = "$GPGGA,,,N,,E,,,,,M,,M,,*\r\n";


/** ***************************************************************************
 * @name copy_pvt_result()
 * @brief store PVT result to the global PVT struct
 * @param in: rover PVT [ST internal; RTK algorithm output; INS algorithm output]
 *        out: gnss solution
 * @retval N/A
 ******************************************************************************/
static void copy_pvt_result(const st_pvt_type999_t* rov, gnss_solution_t* gnss_sol)
{
    double blh[3] = {0};

    if (gnss_sol != NULL)
    {
        ecef2pos(rov->pos_xyz, blh);
        gnss_sol->gps_week = rov->ext_gps_week_number;
        gnss_sol->gps_tow = (uint32_t)(rov->gnss_epoch_time);
        gnss_sol->latitude = blh[0];
        gnss_sol->longitude = blh[1];
        gnss_sol->height = blh[2];

        memcpy(gnss_sol->pos_ecef, rov->pos_xyz, 3*sizeof(double));

        double C_en[3][3] = { {0.0} }, vel[3];
        blh2C_en(blh, C_en);
        for (int i = 0; i < 3; ++i) {
            gnss_sol->vel_ned[i] = 0.0f;
            for (int j = 0; j < 3; ++j) {
                gnss_sol->vel_ned[i] += C_en[i][j]*rov->vel_xyz[j];
            }
        }
        
        gnss_sol->sol_age = 0.0;

        gnss_sol->num_sats = rov->num_sat_in_use;
        gnss_sol->gnss_fix_type = 1;
        gnss_sol->dops[2] = rov->hdop;
        gnss_sol->gnss_update = 1;
    }
}

/** ***************************************************************************
 * @name rtk_algorithm()
 * @brief  GNSS RTK algorithm entry
 *         Rover and base data are stored in g_ptr_gnss_data
 *         nav_t *nav = &g_ptr_gnss_data->nav;
*          obs_t *rov = &g_ptr_gnss_data->rov;
 * @param  N/A
 * @retval N/A
 ******************************************************************************/
static void rtk_algorithm(void)
{

}

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

        rtk_algorithm();

        /* use ST internal SPP solution as an example for GNSS PVT solution 
        * For User reference, copy your RTK PVT to g_gnss_sol struct
        */
        st_pvt_type999_t* st_pvt = &g_ptr_gnss_data->rtcm.rcv[ROVER].st_pvt;
        if (st_pvt->flag_gnss_update) {
            copy_pvt_result(st_pvt, &g_gnss_sol);
        
            /* use rover position to make NMEA GGA message and pull base rtcm data */
            gtime_t time = gpst2time(g_gnss_sol.gps_week, g_gnss_sol.gps_tow*0.001);
            print_pos_gga(time, g_gnss_sol.pos_ecef, g_gnss_sol.num_sats, g_gnss_sol.gnss_fix_type, g_gnss_sol.dops[2], g_gnss_sol.sol_age, gga_buff);
            
            /* send NMEA GGA position to NTRIP server using Bluetooth */
            uart_write_bytes(UART_BT, gga_buff, strlen(gga_buff), 1);
            /* send NMEA GGA position to NTRIP server using Ethernet */
            ntrip_push_tx_data((uint8_t*)gga_buff, strlen(gga_buff));

            st_pvt->flag_gnss_update = 0;
        }
        else {
            g_gnss_sol.gnss_update = 0;
        }

        osSemaphoreRelease(g_sem_rtk_finish);

        // uxHighWaterMark = uxTaskGetStackHighWaterMark(NULL);
        // printf("uxhigh=%d\r\n",TASKRTD_STACK-uxHighWaterMark); 
    }
}