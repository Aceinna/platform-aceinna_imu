/** ***************************************************************************
 * @file   user_message.c
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
#include <stdint.h>
#include <stdio.h>
#include "app_version.h"
#include "platformAPI.h"
#include "configurationAPI.h"
#include "sensorsAPI.h"
#include "calibrationAPI.h"
#include "ucb_packet.h"
#include "user_message.h"
#include "user_config.h"
#include "timer.h"
#include "rtklib_core.h"
#include "rtcm.h"
#include "gnss_data_api.h"
#include "configuration.h"
#include "Indices.h"
#include "ins_interface_API.h"
#include "uart.h"
#include "crc16.h"

#ifndef RAD2DEG
#define  RAD2DEG (57.295779513082320)
#endif // !RAD2DEG
// extern epoch_t gRov;
// extern GnssInsSystem mGnssInsSystem;

/// List of allowed packet codes 
usr_packet_t userInputPackets[] = {
    {USR_IN_NONE,               {0,0}},
    {USR_IN_PING,               "pG"},
    {USR_IN_GET_VERSION,        "gV"}, 
    {USR_IN_UPDATE_PARAM,       "uP"},
    {USR_IN_SAVE_CONFIG,        "sC"},
    {USR_IN_GET_ALL,            "gA"},
    {USR_IN_RESTORE_DEFAULTS,   "rD"},
    {USR_IN_GET_BLOCK,          "gB"},
    {USR_IN_UPDATE_BLOCK,       "uB"},
    {USR_IN_CAR_SPEED,          "cA"},
// place new input packet code here, before USR_IN_MAX
    {USR_IN_MAX,                {0xff, 0xff}},   //  "" 
};


// packet codes here should be unique - 
// should not overlap codes for input packets and system packets
// First byte of Packet code should have value  >= 0x61  
usr_packet_t userOutputPackets[] = {	
//   Packet Type                Packet Code
    {USR_OUT_NONE,              {0x00, 0x00}}, 
    {USR_OUT_RAWIMU,            "s1"},
    {USR_OUT_BESTGNSS,          "g1"},
    {USR_OUT_INSPVAX,           "i1"},
    {USR_OUT_ODO,               "o1"},
    {USR_OUT_SATELLITES,        "y1"},

// place new type and code here
    {USR_OUT_MAX,               {0xff, 0xff}},   //  "" 
};

static int _inputPacketType = USR_IN_MAX;
status_t g_status;

int checkUserInPacketType(uint16_t receivedCode)
{
    usr_packet_t *packet  = &userInputPackets[1];
    uint16_t code;

    // validate packet code here and memorise for further processing
    while (packet->packetType != USR_IN_MAX) {
        code = (packet->packetCode[0] << 8) | packet->packetCode[1];
        if (code == receivedCode) {
            _inputPacketType = packet->packetType;
            return UCB_USER_IN;
        }
        packet++;
    }
    return UCB_ERROR_INVALID_TYPE;
}

int checkUserOutPacketType(uint16_t receivedCode)
{
    usr_packet_t *packet  = &userOutputPackets[1];
    uint16_t code;

    while (packet->packetType != USR_OUT_MAX) {
        code = (packet->packetCode[0] << 8) | packet->packetCode[1];
        if (code == receivedCode) {
            return UCB_USER_OUT;
        }
        packet++;
    }
    return UCB_ERROR_INVALID_TYPE;
}

void   user_inpacket_type_to_bytes(uint8_t bytes[])
{
    if(_inputPacketType && _inputPacketType <  USR_IN_MAX){
        // response to request. Return same packet code
        bytes[0] = userInputPackets[_inputPacketType].packetCode[0];
        bytes[1] = userInputPackets[_inputPacketType].packetCode[1];
        _inputPacketType = USR_IN_MAX;  // wait for next input packet
    } else {
        bytes[0] = 0;
        bytes[1] = 0;
    }

}

/******************************************************************************
 * @name  FillPingPacketPayload - API call ro prepare user output packet
 * @brief general handler
 * @param [in] payload pointer to put user data to
 * @param [in/out] number of bytes in user payload
 * @retval N/A
 ******************************************************************************/
static BOOL Fill_PingPacketPayload(uint8_t *payload, uint8_t *payloadLen)
{
    int len; 
    uint8_t *model = (uint8_t*)GetUnitVersion();
    uint8_t *rev = (uint8_t*)platformBuildInfo();
    unsigned int serialNum = GetUnitSerialNum();
    len = snprintf((char*)payload, 250, "%s %s %s SN:%u", PRODUCT_NAME_STRING, model, rev, serialNum );
    *payloadLen = len;
    return TRUE;
}

/******************************************************************************
 * @name FillVersionPacketPayload - API call ro prepare user output packet
 * @brief general handler
 * @param [in] payload pointer to put user data to
 * @param [in/out] number of bytes in user payload
 * @retval N/A
 ******************************************************************************/
static BOOL Fill_VersionPacketPayload(uint8_t *payload, uint8_t *payloadLen)
{
   int len = snprintf((char*)payload, 250, "%s, %s", APP_VERSION_STRING, get_boot_version());
   *payloadLen = len;
    return TRUE;
}

/******************************************************************************
 * @name user_car_speed_handler
 * @brief general handler
 * @param [in] payload pointer to put user data to
 * @param [in/out] number of bytes in user payload
 * @retval N/A
 ******************************************************************************/
static BOOL user_car_speed_handler(uint8_t *payload, uint8_t *payloadLen)
{
    float speed;
    BOOL ret = FALSE;

    if (*payloadLen == 4) {
        
        speed = *(float *)payload;

        ins_user_odo_handler(speed);

        ret = TRUE;
    }

    return ret;
}

/******************************************************************************
 * @name HandleUserInputPacket - API
 * @brief general handler
 * @param [out] packetPtr - filled in packet from the mapped physical port
 * @retval N/A
 ******************************************************************************/
int HandleUserInputPacket(UcbPacketStruct *ptrUcbPacket)
{
    BOOL valid = TRUE;
    int ret = USER_PACKET_OK;

    /// call appropriate function based on packet type
	switch (_inputPacketType) {
		case USR_IN_PING:
            {
                uint8_t len;
                Fill_PingPacketPayload(ptrUcbPacket->payload, &len);
                ptrUcbPacket->payloadLength = len;
            }
            // leave all the same - it will be bounced back unchanged
            break;
		case USR_IN_GET_VERSION:
            {
                uint8_t len;
                Fill_VersionPacketPayload(ptrUcbPacket->payload, &len);
                ptrUcbPacket->payloadLength = len;
            }
            break;
        case USR_IN_UPDATE_PARAM:
             update_user_param((userParamPayload*)ptrUcbPacket->payload, &ptrUcbPacket->payloadLength);
             break;
        case USR_IN_SAVE_CONFIG:
            // payload length does not change
             if(!SaveUserConfig()){
                valid = FALSE;
             }
             break;
        case USR_IN_GET_ALL:
            //  if(!get_all_user_params(ptrUcbPacket->payload, &ptrUcbPacket->payloadLength)){
            valid = FALSE;
            //  }
             break;
        case USR_IN_RESTORE_DEFAULTS:
            valid = RestoreDefaultUserConfig();
            break;
        case USR_IN_GET_BLOCK:
            get_block_user_params(ptrUcbPacket->payload, &ptrUcbPacket->payloadLength);
            break;
        case USR_IN_UPDATE_BLOCK:
            update_block_user_params(ptrUcbPacket->payload, &ptrUcbPacket->payloadLength);
            break;
        case USR_IN_CAR_SPEED:
            valid = user_car_speed_handler(ptrUcbPacket->payload, &ptrUcbPacket->payloadLength);
            break;
        default:
             /// default handler - unknown packet
             valid = FALSE;
             break;
        }

        if(!valid){
             ptrUcbPacket->payloadLength = 0;
             ret = USER_PACKET_ERROR;
        }
        
        return ret;
}

static void _send_user_packet(uint16_t const port, UcbPacketStruct *const ptr_usrpacket, uint8_t const packet_type) 
{
    uint16_t crc;

    if (ptr_usrpacket != NULL && packet_type && packet_type < USR_OUT_MAX) {
        // packet head
        ptr_usrpacket->sync_MSB = 0x55;
        ptr_usrpacket->sync_LSB = 0x55;

        // packet type
        ptr_usrpacket->code_MSB = userOutputPackets[packet_type].packetCode[0];
        ptr_usrpacket->code_LSB = userOutputPackets[packet_type].packetCode[1];

        // packet len and payload
        // ptr_usrpacket->payloadLength = datalen;
        // memcpy(ptr_usrpacket->payload, ptr_data, datalen);

        // packet crc
        crc = CalculateCRC((uint8_t *)&ptr_usrpacket->code_MSB, ptr_usrpacket->payloadLength + 3);
        ptr_usrpacket->payload[ptr_usrpacket->payloadLength + 1] = (crc >> 8) & 0xff;
        ptr_usrpacket->payload[ptr_usrpacket->payloadLength] = crc & 0xff;

        // send packet data
        uart_write_bytes(port, (const char *)&ptr_usrpacket->sync_MSB, ptr_usrpacket->payloadLength + 7, 1);
    }
}

void send_rawimu_packet(uint16_t const port, UcbPacketStruct *const ptr_usrpacket)
{
    output_rawimu_struct *pld = (output_rawimu_struct *)ptr_usrpacket->payload;
    double* acc = get_imu_acc();
    double* gyro = get_imu_gyro();

    ptr_usrpacket->payloadLength = sizeof(output_rawimu_struct);
    
    pld->gps_week        =   get_imu_week();
    pld->gps_millisecs   =   get_imu_timestamp() * 1000;
    pld->x_acceleration  =   *acc++;
    pld->y_acceleration  =   *acc++;
    pld->z_acceleration  =   *acc;
    pld->x_gyro_rate     =   (*gyro++) * RAD2DEG;
    pld->y_gyro_rate     =   (*gyro++) * RAD2DEG;
    pld->z_gyro_rate     =   (*gyro) * RAD2DEG;

    _send_user_packet(port, ptr_usrpacket, USR_OUT_RAWIMU);
}

void send_bestgnss_packet(uint16_t const port, UcbPacketStruct *const ptr_usrpacket, gnss_solution_t *const ptr_bestgnss_sol)
{
    output_bestgnss_struct *pld = (output_bestgnss_struct *)ptr_usrpacket->payload;

    ptr_usrpacket->payloadLength = sizeof(output_bestgnss_struct);

    pld->gps_week                           =   ptr_bestgnss_sol->gps_week;
    pld->gps_millisecs                      =   ptr_bestgnss_sol->gps_tow;
    pld->position_type                      =   ptr_bestgnss_sol->gnss_fix_type;
    pld->latitude                           =   ptr_bestgnss_sol->latitude * RAD_TO_DEG;
    pld->longitude                          =   ptr_bestgnss_sol->longitude * RAD_TO_DEG;
    pld->height                             =   ptr_bestgnss_sol->height;
    pld->latitude_standard_deviation        =   ptr_bestgnss_sol->std_lat;
    pld->longitude_standard_deviation       =   ptr_bestgnss_sol->std_lon;
    pld->height_standard_deviation          =   ptr_bestgnss_sol->std_hgt;
    pld->number_of_satellites               =   ptr_bestgnss_sol->rov_n;
    pld->number_of_satellites_in_solution   =   ptr_bestgnss_sol->num_sats;
    pld->hdop                               =   ptr_bestgnss_sol->dops[2];
    pld->diffage                            =   ptr_bestgnss_sol->sol_age;
    pld->north_vel                          =   ptr_bestgnss_sol->vel_ned[0];
    pld->east_vel                           =   ptr_bestgnss_sol->vel_ned[1];
    pld->up_vel                             =   ptr_bestgnss_sol->vel_ned[2];
    pld->north_vel_standard_deviation       =   ptr_bestgnss_sol->std_vn;
    pld->east_vel_standard_deviation        =   ptr_bestgnss_sol->std_ve;
    pld->up_vel_standard_deviation          =   ptr_bestgnss_sol->std_vd;

    _send_user_packet(port, ptr_usrpacket, USR_OUT_BESTGNSS);
}

void send_inspvax_packet(uint16_t const port, UcbPacketStruct *const ptr_usrpacket, ins_solution_t *const ptr_ins_sol)
{
    output_inspvax_struct *pld = (output_inspvax_struct *)ptr_usrpacket->payload;

    ptr_usrpacket->payloadLength = sizeof(output_inspvax_struct);

    pld->gps_week               =   ptr_ins_sol->gps_week;
    pld->gps_millisecs          =   ptr_ins_sol->gps_millisecs;
	pld->ins_status             =   ptr_ins_sol->ins_status;
    pld->ins_position_type      =   ptr_ins_sol->pos_type;
	pld->latitude               =   ptr_ins_sol->latitude;
	pld->longitude              =   ptr_ins_sol->longitude;
	pld->height                 =   ptr_ins_sol->height;
	pld->north_velocity         =   ptr_ins_sol->north_velocity;
	pld->east_velocity          =   ptr_ins_sol->east_velocity;
	pld->up_velocity            =   ptr_ins_sol->up_velocity;
	pld->roll                   =   ptr_ins_sol->roll;
	pld->pitch                  =   ptr_ins_sol->pitch;
	pld->heading                =   ptr_ins_sol->azimuth;
    pld->latitude_std           =   ptr_ins_sol->latitude_std;
	pld->longitude_std          =   ptr_ins_sol->longitude_std;
	pld->height_std             =   ptr_ins_sol->altitude_std; 
	pld->north_velocity_std     =   ptr_ins_sol->north_velocity_std; 
	pld->east_velocity_std      =   ptr_ins_sol->east_velocity_std; 
	pld->up_velocity_std        =   ptr_ins_sol->up_velocity_std; 
	pld->roll_std               =   ptr_ins_sol->roll_std; 
	pld->pitch_std              =   ptr_ins_sol->pitch_std; 
	pld->heading_std            =   ptr_ins_sol->azimuth_std; 

    _send_user_packet(port, ptr_usrpacket, USR_OUT_INSPVAX);
}

void send_odospeed_packet(uint16_t const port, UcbPacketStruct *const ptr_usrpacket)
{
    output_odo_speed_struct *pld = (output_odo_speed_struct *)ptr_usrpacket->payload;

    ptr_usrpacket->payloadLength = sizeof(output_odo_speed_struct);

    pld->gps_week           =   get_odo_gps_week();
    pld->gps_millisecs      =   get_odo_gps_timestamp() * 1000;
	pld->mode               =   get_odo_mode();
	pld->speed              =   get_odo_speed();
    pld->fwd                =   get_odo_fwd();
	pld->wheel_tick         =   get_odo_wheel_tick();

    _send_user_packet(port, ptr_usrpacket, USR_OUT_ODO);
}

void send_satellites_packet(uint16_t const port, UcbPacketStruct *const ptr_usrpacket, gnss_solution_t *const ptr_sta)
{
    uint8_t i, j, n;
    output_satellites_struct *pld = (output_satellites_struct *)ptr_usrpacket->payload;
    uint8_t packets_count = ptr_sta->rov_n / 10;

    if (ptr_sta->rov_n % 10 != 0) {
        packets_count++;
    }

    for (i = 0; i < packets_count; i++) {
        if (i < packets_count - 1) {
            n = (i + 1) * 10;
        } else {
            n = ptr_sta->rov_n;
        }

        pld = (output_satellites_struct *)ptr_usrpacket->payload;
        ptr_usrpacket->payloadLength = sizeof(output_satellites_struct) * (n - i * 10);

        for (j = i * 10; j < n; j++) {
            pld->gps_week       = ptr_sta->gps_week;
            pld->gps_millisecs  = ptr_sta->gps_tow;
            pld->satelliteId    = ptr_sta->rov_satellite[j].satelliteId;
            pld->systemId       = ptr_sta->rov_satellite[j].systemId;
            pld->antennaId      = ptr_sta->rov_satellite[j].antennaId;
            pld->l1cn0          = ptr_sta->rov_satellite[j].l1cn0;
            pld->l2cn0          = ptr_sta->rov_satellite[j].l2cn0;
            pld->azimuth        = ptr_sta->rov_satellite[j].azimuth;
            pld->elevation      = ptr_sta->rov_satellite[j].elevation;

            pld++;
        }

        _send_user_packet(port, ptr_usrpacket, USR_OUT_SATELLITES);
    }
}
