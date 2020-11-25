/** ***************************************************************************
 * @file   user_config.c
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

#include "stm32f4xx_hal.h"
#include "string.h"
#include "app_version.h"
#include "platformAPI.h"
#include "eepromAPI.h"
#include "user_config.h"
#include "Indices.h"
#include "configurationAPI.h"
#include "osapi.h"
#include "crc16.h"
#include "sae_j1939.h"
#include "ins_interface_API.h"
#include "lwip_comm.h"
#include "calibrationAPI.h"
#include "uart.h"
#include <stdlib.h>
#include "ntrip_server.h"
#include "configuration.h"

// Default user configuration structure
// Do Not remove - just add extra parameters if needed
// Change default settings  if desired
const UserConfigurationStruct gDefaultUserConfig = {
    .dataCRC             =  0,
    .dataSize            =  sizeof(UserConfigurationStruct),

    // add default parameter values here, if desired
    .rawimu_packet_rate     = 100,
    .bestgnss_packet_rate   = 1,
    .inspvax_packet_rate    = 100,
    .odospeed_packet_rate   = 10,
    .satellites_packet_rate = 1,
    .nmea_ins_rate          = 10,
    .nmea_gga_rate          = 1,
    .nmea_rmc_rate          = 1,
    .nmea_pashr_rate        = 1,
    .nmea_gsa_rate          = 1,
    .nmea_zda_rate          = 1,
    .nmea_vtg_rate          = 1,

    .pri_lever_arm_bx       = 0.0,
    .pri_lever_arm_by       = 0.0,
    .pri_lever_arm_bz       = 0.0,
    .vrp_lever_arm_bx       = 0.0,
    .vrp_lever_arm_by       = 0.0,
    .vrp_lever_arm_bz       = 0.0,
    .user_lever_arm_bx      = 0.0,
    .user_lever_arm_by      = 0.0,
    .user_lever_arm_bz      = 0.0,
    .rotation_rbvx          = 0.0,
    .rotation_rbvy          = 0.0,
    .rotation_rbvz          = 0.0,
    
    .eth_mode           = ETHMODE_DHCP,
    .static_ip          = {192, 168, 137, 110},
	.static_netmask     = {255, 255, 255, 0},
	.static_gateway     = {192, 168, 137, 1},
	.mac                = {2, 0, 0, 0, 0, 0},

    .station_mode       = MODE_NTRIP_CLIENT,

    .ntrip_client_ip            = "47.116.1.17",
	.ntrip_client_port          = 2201,
	.ntrip_client_mount_point   = "WX02",
    .ntrip_client_username      = "AceinnaRTK",
	.ntrip_client_password      = "123456",

    .aceinna_client_ip          = "openarc.aceinna.com",
	.aceinna_client_port        = 8011,
	.aceinna_client_mount_point = "RTK",
    .aceinna_client_username    = "AceinnaRTK",
	.aceinna_client_password    = "123456",

    .ntrip_server_ip            = "47.116.1.17",
	.ntrip_server_port          = 2201,
    .ntrip_server_mount_point   = "BASE01",
    .ntrip_server_password      = "123456",
    
    .base_position_type         = BASE_POSITION_SPP,
    .station_id                 = 0,
    .antenna_height             = 0,
    .reference_latitude         = 0.0,
    .reference_longitude        = 0.0,
    .reference_height           = 0.0,

    .can_ecu_address        = 128,
    .can_baudrate           = _ECU_500K,
    .can_packet_type        = 0x0f,
    .can_packet_rate        = 10,       // 10Hz
    .can_termresistor       = 0,
    .can_baudrate_detect    = 0,

    .wheeltick_pin_mode     = 0,
    .can_mode               = 0,
    .gears                  = {0, 0, 0, 0},
    .odo_mesg[0].usage      = 0,
    .odo_mesg[1].usage      = 0,
    .odo_mesg[2].usage      = 0,
};

extern uint8_t rawimu_packet_rate;
extern uint8_t bestgnss_packet_rate;
extern uint8_t inspvax_packet_rate;
extern uint8_t odospeed_packet_rate;
extern uint8_t satellites_packet_rate;
extern uint8_t nmea_ins_rate;
extern uint8_t nmea_gga_rate;
extern uint8_t nmea_rmc_rate;
extern uint8_t nmea_pashr_rate;
extern uint8_t nmea_gsa_rate;
extern uint8_t nmea_zda_rate;
extern uint8_t nmea_vtg_rate;


CCMRAM UserConfigurationStruct gUserConfiguration;
const uint8_t *pUserConfigInFlash = (uint8_t *)APP_USER_CONFIG_ADDR;
BOOL configValid = FALSE;

char boot_version[BOOT_VERSION_MAX_LEN] = {0};
char st_sdk_version[30] = {0};

char* get_boot_version(void)
{
    if (strlen((char*)(BOOT_VERSION_ADDR)) < BOOT_VERSION_MAX_LEN) {
        if (strstr((char*)(BOOT_VERSION_ADDR), "BootLoader") != NULL || strstr((char*)(BOOT_VERSION_ADDR), "BOOT") != NULL) {
            strcpy(boot_version, (char*)(BOOT_VERSION_ADDR));
        }
    }
    
    return boot_version;
}

char* get_sdk_version(void)
{
    return st_sdk_version;
}

void set_sdk_version(char* ver)
{
    strcpy(st_sdk_version, ver);
}

int32_t config_packet_rate(uint8_t value, uint8_t *config_rate, uint8_t *rate)
{
    int32_t ret = 0;

    if (value == 0 || (100 % value == 0)) {
        *config_rate = value;
        if (value != 0) {
            *rate = 100 / value;
        } else {
            *rate = 0;
        }
    } else {
        ret = -1;
    }

    return ret;
}

/** ***************************************************************************
 * @name update_user_parameter - updating of user configuration parameter based of user preferences 
 * @brief
 *
 * @param [in]  number - parameter number in user configuration structure
 * @param [in]  data   - value of the parameter in little endian format
 * @retval error (0), no error (>=1)
 ******************************************************************************/
int32_t update_user_parameter(uint32_t number, uint8_t* data, BOOL offset)
{
    int32_t dataLen = 0;
    bool ret;
    uint16_t user_packet_rate;
    int rate;

    if (number <= USER_DATA_SIZE || number >= USER_MAX_PARAM ) {
        return -1;
    }

    if (!offset && !data) {
        return - 1;
    }
    
    switch (number)
    {

    case USER_RAWIMU_PACKET_RATE:
        if (offset) {
            return ((uint32_t)&gUserConfiguration.rawimu_packet_rate - (uint32_t)&gUserConfiguration);
        }
        config_packet_rate(*data, &gUserConfiguration.rawimu_packet_rate, &rawimu_packet_rate);
        dataLen = 1;
        break;
    case USER_BESTGNSS_PACKET_RATE:
        if (offset) {
            return ((uint32_t)&gUserConfiguration.bestgnss_packet_rate - (uint32_t)&gUserConfiguration);
        }
        config_packet_rate(*data, &gUserConfiguration.bestgnss_packet_rate, &bestgnss_packet_rate);
        dataLen = 1;
        break;
    case USER_INSPVA_PACKET_RATE:
        if (offset) {
            return ((uint32_t)&gUserConfiguration.inspvax_packet_rate - (uint32_t)&gUserConfiguration);
        }
        config_packet_rate(*data, &gUserConfiguration.inspvax_packet_rate, &inspvax_packet_rate);
        dataLen = 1;
        break;
    case USER_ODOSPEED_PACKET_RATE:
        if (offset) {
            return ((uint32_t)&gUserConfiguration.odospeed_packet_rate - (uint32_t)&gUserConfiguration);
        }
        config_packet_rate(*data, &gUserConfiguration.odospeed_packet_rate, &odospeed_packet_rate);
        dataLen = 1;
        break;
    case USER_SATELLITES_PACKET_RATE:
        if (offset) {
            return ((uint32_t)&gUserConfiguration.satellites_packet_rate - (uint32_t)&gUserConfiguration);
        }
        config_packet_rate(*data, &gUserConfiguration.satellites_packet_rate, &satellites_packet_rate);
        dataLen = 1;
        break;

    case USER_NMEA_INS_RATE:
        if (offset) {
            return ((uint32_t)&gUserConfiguration.nmea_ins_rate - (uint32_t)&gUserConfiguration);
        }
        config_packet_rate(*data, &gUserConfiguration.nmea_ins_rate, &nmea_ins_rate);
        dataLen = 1;
        break;
    case USER_NMEA_GGA_RATE:
        if (offset) {
            return ((uint32_t)&gUserConfiguration.nmea_gga_rate - (uint32_t)&gUserConfiguration);
        }
        config_packet_rate(*data, &gUserConfiguration.nmea_gga_rate, &nmea_gga_rate);
        dataLen = 1;
        break;
    case USER_NMEA_RMC_RATE:
        if (offset) {
            return ((uint32_t)&gUserConfiguration.nmea_rmc_rate - (uint32_t)&gUserConfiguration);
        }
        config_packet_rate(*data, &gUserConfiguration.nmea_rmc_rate, &nmea_rmc_rate);
        dataLen = 1;
        break;
    case USER_NMEA_PASHR_RATE:
        if (offset) {
            return ((uint32_t)&gUserConfiguration.nmea_pashr_rate - (uint32_t)&gUserConfiguration);
        }
        config_packet_rate(*data, &gUserConfiguration.nmea_pashr_rate, &nmea_pashr_rate);
        dataLen = 1;
        break;
    case USER_NMEA_GSA_RATE:
        if (offset) {
            return ((uint32_t)&gUserConfiguration.nmea_gsa_rate - (uint32_t)&gUserConfiguration);
        }
        config_packet_rate(*data, &gUserConfiguration.nmea_gsa_rate, &nmea_gsa_rate);
        dataLen = 1;
        break;
    case USER_NMEA_ZDA_RATE:
        if (offset) {
            return ((uint32_t)&gUserConfiguration.nmea_zda_rate - (uint32_t)&gUserConfiguration);
        }
        config_packet_rate(*data, &gUserConfiguration.nmea_zda_rate, &nmea_zda_rate);
        dataLen = 1;
        break;
    case USER_NMEA_VTG_RATE:
        if (offset) {
            return ((uint32_t)&gUserConfiguration.nmea_vtg_rate - (uint32_t)&gUserConfiguration);
        }
        config_packet_rate(*data, &gUserConfiguration.nmea_vtg_rate, &nmea_vtg_rate);
        dataLen = 1;
        break;

    case USER_PRI_LEVER_ARM_BX:
        if (offset) {
            return ((uint32_t)&gUserConfiguration.pri_lever_arm_bx - (uint32_t)&gUserConfiguration);
        }
        gUserConfiguration.pri_lever_arm_bx = *(float *)data;
        dataLen = sizeof(gUserConfiguration.pri_lever_arm_bx);
        break;
    case USER_PRI_LEVER_ARM_BY:
        if (offset) {
            return ((uint32_t)&gUserConfiguration.pri_lever_arm_by - (uint32_t)&gUserConfiguration);
        }
        gUserConfiguration.pri_lever_arm_by = *(float *)data;
        dataLen = sizeof(gUserConfiguration.pri_lever_arm_by);
        break;
    case USER_PRI_LEVER_ARM_BZ:
        if (offset) {
            return ((uint32_t)&gUserConfiguration.pri_lever_arm_bz - (uint32_t)&gUserConfiguration);
        }
        gUserConfiguration.pri_lever_arm_bz = *(float *)data;
        dataLen = sizeof(gUserConfiguration.pri_lever_arm_bz);
        break;
    case USER_VRP_LEVER_ARM_BX:
        if (offset) {
            return ((uint32_t)&gUserConfiguration.vrp_lever_arm_bx - (uint32_t)&gUserConfiguration);
        }
        gUserConfiguration.vrp_lever_arm_bx = *(float *)data;
        dataLen = sizeof(gUserConfiguration.vrp_lever_arm_bx);
        break;
    case USER_VRP_LEVER_ARM_BY:
        if (offset) {
            return ((uint32_t)&gUserConfiguration.vrp_lever_arm_by - (uint32_t)&gUserConfiguration);
        }
        gUserConfiguration.vrp_lever_arm_by = *(float *)data;
        dataLen = sizeof(gUserConfiguration.vrp_lever_arm_by);
        break;
    case USER_VRP_LEVER_ARM_BZ:
        if (offset) {
            return ((uint32_t)&gUserConfiguration.vrp_lever_arm_bz - (uint32_t)&gUserConfiguration);
        }
        gUserConfiguration.vrp_lever_arm_bz = *(float *)data;
        dataLen = sizeof(gUserConfiguration.vrp_lever_arm_bz);
        break;
    case USER_USER_LEVER_ARM_BX:
        if (offset) {
            return ((uint32_t)&gUserConfiguration.user_lever_arm_bx - (uint32_t)&gUserConfiguration);
        }
        gUserConfiguration.user_lever_arm_bx = *(float *)data;
        dataLen = sizeof(gUserConfiguration.user_lever_arm_bx);
        break;
    case USER_USER_LEVER_ARM_BY:
        if (offset) {
            return ((uint32_t)&gUserConfiguration.user_lever_arm_by - (uint32_t)&gUserConfiguration);
        }
        gUserConfiguration.user_lever_arm_by = *(float *)data;
        dataLen = sizeof(gUserConfiguration.user_lever_arm_by);
        break;
    case USER_USER_LEVER_ARM_BZ:
        if (offset) {
            return ((uint32_t)&gUserConfiguration.user_lever_arm_bz - (uint32_t)&gUserConfiguration);
        }
        gUserConfiguration.user_lever_arm_bz = *(float *)data;
        dataLen = sizeof(gUserConfiguration.user_lever_arm_bz);
        break;
    case USER_ROTATION_RBVX:
        if (offset) {
            return ((uint32_t)&gUserConfiguration.rotation_rbvx - (uint32_t)&gUserConfiguration);
        }
        gUserConfiguration.rotation_rbvx = *(float *)data;
        dataLen = sizeof(gUserConfiguration.rotation_rbvx);
        break;
    case USER_ROTATION_RBVY:
        if (offset) {
            return ((uint32_t)&gUserConfiguration.rotation_rbvy - (uint32_t)&gUserConfiguration);
        }
        gUserConfiguration.rotation_rbvy = *(float *)data;
        dataLen = sizeof(gUserConfiguration.rotation_rbvy);
        break;
    case USER_ROTATION_RBVZ:
        if (offset) {
            return ((uint32_t)&gUserConfiguration.rotation_rbvz - (uint32_t)&gUserConfiguration);
        }
        gUserConfiguration.rotation_rbvz = *(float *)data;
        dataLen = sizeof(gUserConfiguration.rotation_rbvz);
        break;
    
    case USER_ETHERNET_ETHMODE:
        if (offset) {
            return ((uint32_t)&gUserConfiguration.eth_mode - (uint32_t)&gUserConfiguration);
        }
        gUserConfiguration.eth_mode = *(uint16_t*)data;
        dataLen = sizeof(gUserConfiguration.eth_mode);
        break;
    case USER_ETHERNET_STATIC_IP:
        if (offset) {
            return ((uint32_t)gUserConfiguration.static_ip - (uint32_t)&gUserConfiguration);
        }
        dataLen = sizeof(gUserConfiguration.static_ip);
        memcpy(gUserConfiguration.static_ip, data, dataLen);
        break;
    case USER_ETHERNET_STATIC_NETMASK:
        if (offset) {
            return ((uint32_t)gUserConfiguration.static_netmask - (uint32_t)&gUserConfiguration);
        }
        dataLen = sizeof(gUserConfiguration.static_netmask);
        memcpy(gUserConfiguration.static_netmask, data, dataLen);
        break;
    case USER_ETHERNET_STATIC_GATEWAY:
        if (offset) {
            return ((uint32_t)gUserConfiguration.static_gateway - (uint32_t)&gUserConfiguration);
        }
        dataLen = sizeof(gUserConfiguration.static_gateway);
        memcpy(gUserConfiguration.static_gateway, data, dataLen);
        break;
    case USER_ETHERNET_MAC:
        if (offset) {
            return ((uint32_t)gUserConfiguration.mac - (uint32_t)&gUserConfiguration);
        }
        dataLen = sizeof(gUserConfiguration.mac);
        memcpy(gUserConfiguration.mac, data, dataLen);
        break;

    case USER_STATION_MODE:
        if (offset) {
            return ((uint32_t)&gUserConfiguration.station_mode - (uint32_t)&gUserConfiguration);
        }
        gUserConfiguration.station_mode = *(uint32_t*)data;
        dataLen = sizeof(gUserConfiguration.station_mode);
        break;
    case USER_NTRIP_CLIENT_IP:
        if (offset) {
            return ((uint32_t)gUserConfiguration.ntrip_client_ip - (uint32_t)&gUserConfiguration);
        }
        dataLen = sizeof(gUserConfiguration.ntrip_client_ip);
        memcpy(gUserConfiguration.ntrip_client_ip, data, dataLen);
        break;
    case USER_NTRIP_CLIENT_PORT:
        if (offset) {
            return ((uint32_t)&gUserConfiguration.ntrip_client_port - (uint32_t)&gUserConfiguration);
        }
        gUserConfiguration.ntrip_client_port = *(uint16_t*)data;
        dataLen = sizeof(gUserConfiguration.ntrip_client_port);
        break;
    case USER_NTRIP_CLIENT_MOUNT_POINT:
        if (offset) {
            return ((uint32_t)gUserConfiguration.ntrip_client_mount_point - (uint32_t)&gUserConfiguration);
        }
        dataLen = sizeof(gUserConfiguration.ntrip_client_mount_point);
        memcpy(gUserConfiguration.ntrip_client_mount_point, data, dataLen);
        break;
    case USER_NTRIP_CLIENT_USERNAME:
        if (offset) {
            return ((uint32_t)gUserConfiguration.ntrip_client_username - (uint32_t)&gUserConfiguration);
        }
        dataLen = sizeof(gUserConfiguration.ntrip_client_username);
        memcpy(gUserConfiguration.ntrip_client_username, data, dataLen);
        break;
    case USER_NTRIP_CLIENT_PASSWORD:
        if (offset) {
            return ((uint32_t)gUserConfiguration.ntrip_client_password - (uint32_t)&gUserConfiguration);
        }
        dataLen = sizeof(gUserConfiguration.ntrip_client_password);
        memcpy(gUserConfiguration.ntrip_client_password, data, dataLen);
        break;

    case USER_ACEINNA_CLIENT_IP:
        if (offset) {
            return ((uint32_t)gUserConfiguration.aceinna_client_ip - (uint32_t)&gUserConfiguration);
        }
        dataLen = sizeof(gUserConfiguration.aceinna_client_ip);
        memcpy(gUserConfiguration.aceinna_client_ip, data, dataLen);
        break;
    case USER_ACEINNA_CLIENT_PORT:
        if (offset) {
            return ((uint32_t)&gUserConfiguration.aceinna_client_port - (uint32_t)&gUserConfiguration);
        }
        gUserConfiguration.aceinna_client_port = *(uint16_t*)data;
        dataLen = sizeof(gUserConfiguration.aceinna_client_port);
        break;
    case USER_ACEINNA_CLIENT_MOUNT_POINT:
        if (offset) {
            return ((uint32_t)gUserConfiguration.aceinna_client_mount_point - (uint32_t)&gUserConfiguration);
        }
        dataLen = sizeof(gUserConfiguration.aceinna_client_mount_point);
        memcpy(gUserConfiguration.aceinna_client_mount_point, data, dataLen);
        break;
    case USER_ACEINNA_CLIENT_USERNAME:
        if (offset) {
            return ((uint32_t)gUserConfiguration.aceinna_client_username - (uint32_t)&gUserConfiguration);
        }
        dataLen = sizeof(gUserConfiguration.aceinna_client_username);
        memcpy(gUserConfiguration.aceinna_client_username, data, dataLen);
        break;
    case USER_ACEINNA_CLIENT_PASSWORD:
        if (offset) {
            return ((uint32_t)gUserConfiguration.aceinna_client_password - (uint32_t)&gUserConfiguration);
        }
        dataLen = sizeof(gUserConfiguration.aceinna_client_password);
        memcpy(gUserConfiguration.aceinna_client_password, data, dataLen);
        break;
    case USER_NTRIP_SERVER_IP:
        if (offset) {
            return ((uint32_t)gUserConfiguration.ntrip_server_ip - (uint32_t)&gUserConfiguration);
        }
        dataLen = sizeof(gUserConfiguration.ntrip_server_ip);
        memcpy(gUserConfiguration.ntrip_server_ip, data, dataLen);
        break;
    case USER_NTRIP_SERVER_PORT:
        if (offset) {
            return ((uint32_t)&gUserConfiguration.ntrip_server_port - (uint32_t)&gUserConfiguration);
        }
        gUserConfiguration.ntrip_server_port = *(uint16_t*)data;
        dataLen = sizeof(gUserConfiguration.ntrip_server_port);
        break;
    case USER_NTRIP_SERVER_MOUNT_POINT:
        if (offset) {
            return ((uint32_t)gUserConfiguration.ntrip_server_mount_point - (uint32_t)&gUserConfiguration);
        }
        dataLen = sizeof(gUserConfiguration.ntrip_server_mount_point);
        memcpy(gUserConfiguration.ntrip_server_mount_point, data, dataLen);
        break;
    case USER_NTRIP_SERVER_PASSWORD:
        if (offset) {
            return ((uint32_t)gUserConfiguration.ntrip_server_password - (uint32_t)&gUserConfiguration);
        }
        dataLen = sizeof(gUserConfiguration.ntrip_server_password);
        memcpy(gUserConfiguration.ntrip_server_password, data, dataLen);
        break;
    case USER_BASE_POSITION_TYPE:
        if (offset) {
            return ((uint32_t)&gUserConfiguration.base_position_type - (uint32_t)&gUserConfiguration);
        }
        gUserConfiguration.base_position_type = *(uint16_t*)data;
        dataLen = sizeof(gUserConfiguration.base_position_type);
        break;
    case USER_STATION_ID:
        if (offset) {
            return ((uint32_t)&gUserConfiguration.station_id - (uint32_t)&gUserConfiguration);
        }
        gUserConfiguration.station_id = *(uint16_t*)data;
        dataLen = sizeof(gUserConfiguration.station_id);
        break;
    case USER_ANTENNA_HEIGHT:
        if (offset) {
            return ((uint32_t)&gUserConfiguration.antenna_height - (uint32_t)&gUserConfiguration);
        }
        memcpy(&gUserConfiguration.antenna_height, data, 8);
        dataLen = sizeof(gUserConfiguration.antenna_height);
        break;
    case USER_REFERENCE_LATITUDE:
        if (offset) {
            return ((uint32_t)&gUserConfiguration.reference_latitude- (uint32_t)&gUserConfiguration);
        }
        memcpy(&gUserConfiguration.reference_latitude, data, 8);
        dataLen = sizeof(gUserConfiguration.reference_latitude);
        break;
    case USER_REFERENCE_LONGITUDE:
        if (offset) {
            return ((uint32_t)&gUserConfiguration.reference_longitude - (uint32_t)&gUserConfiguration);
        }
        memcpy(&gUserConfiguration.reference_longitude, data, 8);
        dataLen = sizeof(gUserConfiguration.reference_longitude);
        break;
    case USER_REFERENCE_HEIGHT:
        if (offset) {
            return ((uint32_t)&gUserConfiguration.reference_height - (uint32_t)&gUserConfiguration);
        }
        memcpy(&gUserConfiguration.reference_height, data, 8);
        dataLen = sizeof(gUserConfiguration.reference_height);
        break;

    case USER_CAN_ECU_ADRESS:
        if (offset) {
            return ((uint32_t)&gUserConfiguration.can_ecu_address - (uint32_t)&gUserConfiguration);
        }
        gUserConfiguration.can_ecu_address = *(uint16_t *)data;
        dataLen = sizeof(gUserConfiguration.can_ecu_address);
        break;
    case USER_CAN_BAUDRATE:
        if (offset) {
            return ((uint32_t)&gUserConfiguration.can_baudrate - (uint32_t)&gUserConfiguration);
        }
        gUserConfiguration.can_baudrate = *(uint16_t *)data;
        dataLen = sizeof(gUserConfiguration.can_baudrate);
        break;
    case USER_CAN_PACKET_TYPE:
        if (offset) {
            return ((uint32_t)&gUserConfiguration.can_packet_type - (uint32_t)&gUserConfiguration);
        }
        gUserConfiguration.can_packet_type = *(uint16_t *)data;
        dataLen = sizeof(gUserConfiguration.can_packet_type);
        break;
    case USER_CAN_PACKET_RATE:
        if (offset) {
            return ((uint32_t)&gUserConfiguration.can_packet_rate - (uint32_t)&gUserConfiguration);
        }
        gUserConfiguration.can_packet_rate = *(uint16_t *)data;
        dataLen = sizeof(gUserConfiguration.can_packet_rate);
        break;
    case USER_CAN_TERMRESISTOR:
        if (offset) {
            return ((uint32_t)&gUserConfiguration.can_termresistor - (uint32_t)&gUserConfiguration);
        }
        gUserConfiguration.can_termresistor = *(uint16_t *)data;
        dataLen = sizeof(gUserConfiguration.can_termresistor);
        break;
    case USER_CAN_BAUDRATE_DETECT:
        if (offset) {
            return ((uint32_t)&gUserConfiguration.can_baudrate_detect - (uint32_t)&gUserConfiguration);
        }
        gUserConfiguration.can_baudrate_detect = *(uint16_t *)data;
        dataLen = sizeof(gUserConfiguration.can_baudrate_detect);
        break;
    case USER_WHEELTICK_PIN_MODE:
        if (offset) {
            return ((uint32_t)&gUserConfiguration.wheeltick_pin_mode - (uint32_t)&gUserConfiguration);
        }
        gUserConfiguration.wheeltick_pin_mode = *(uint32_t *)data;
        dataLen = sizeof(gUserConfiguration.wheeltick_pin_mode);
        break;

    default:
        break;
    }

    return dataLen;
}


BOOL update_ethnet_config(uint32_t number)
{
    if (number >= USER_ETHERNET_ETHMODE && number <= USER_ETHERNET_MAC)
    {
        netif_ethernet_config_changed();
    }
    if (number >= USER_STATION_MODE && number <= USER_REFERENCE_HEIGHT)
    {
        netif_station_tcp_config_changed();
    }

    return true;
}

/** ***************************************************************************
 * @name EEPROM_ValidateUserConfig - validating of user configuration structure 
 *       in EEPROM
 *       changes should be made to gUserConfiguration before calling this function.
 * @brief
 *
 * @param [in] userConfigSize - pointer to variable, which initialized with the
 *                              size of user configuration structure
 * @retval error (0), no error (1)
 ******************************************************************************/
BOOL EEPROM_ValidateUserConfig(uint16_t *userConfigSize)
{
    uint16_t    crc, configCrc, size;
    uint16_t   *dataPtr =  (uint16_t*)pUserConfigInFlash;

    configCrc = dataPtr[0];         // CRC 
    size      = dataPtr[1];         // Total Number of bytes in user config structure in eeprom
    if(size != *userConfigSize){    // check if image fits into user storage in RAM
        return FALSE;
    }
    crc = CalculateCRC((uint8_t*)pUserConfigInFlash + 2, size - 2);
    if(crc == configCrc){
        return TRUE;
    }
    return FALSE;    // 
}

/** ***************************************************************************
 * @name loadUserConfigInEeprom - loading user configuration structure from 
 *       predefined flash sector
 *       changes should be made to gUserConfiguration before calling this function.
 * @brief
 *
 * @param [in] N/A
 * @retval error (0), no error (1)
 ******************************************************************************/
BOOL EEPROM_LoadUserConfig(uint8_t *ptrToUserConfigInRam, uint16_t *ptrUserConfigSize)
{
    memcpy(ptrToUserConfigInRam, pUserConfigInFlash, *ptrUserConfigSize);
    return TRUE;
}

/** ***************************************************************************
 * @name EEPROM_SaveUserConfig - saving of user configuration structure un the 
 *       predefined flash sector
 *       changes should be made to gUserConfiguration before calling this function.
 * @brief
 *
 * @param [in] N/A
 * @retval error (0), no error (1)
 ******************************************************************************/
BOOL EEPROM_SaveUserConfig(uint8_t *ptrToUserConfigStruct, uint16_t userConfigSize)
{
    HAL_StatusTypeDef status = HAL_OK;
    FLASH_EraseInitTypeDef pEraseInit;
    uint32_t    PageError;
    uint16_t    offset   =  0;
    uint16_t    num      =  userConfigSize;
    uint32_t    start    =  (uint32_t)pUserConfigInFlash;
    uint32_t   *dataPtr  =  (uint32_t*)ptrToUserConfigStruct;
    uint16_t   *paramPtr =  (uint16_t*)ptrToUserConfigStruct;

    paramPtr[1] = num;  //  Total size of user config structure, including Crc and data size 
    paramPtr[0] = CalculateCRC((uint8_t*)ptrToUserConfigStruct + 2, num - 2);
    // calculate CRC over user configuration structure

    ENTER_CRITICAL();

    status = HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(0xffff);

	pEraseInit.Banks = FLASH_BANK_1;
	pEraseInit.Sector = FLASH_SECTOR_10;
	pEraseInit.NbSectors = 1;
	pEraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	pEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;

	status = HAL_FLASHEx_Erase(&pEraseInit, &PageError);
	if (status != HAL_OK)
	{
		EXIT_CRITICAL();
		return FALSE;
	}

	while (num > 0)
    {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, start + offset, *dataPtr++);
        if (status != HAL_OK)
        {
            EXIT_CRITICAL();
            return FALSE;
        }
        offset += 4;
        num -= 4;
    }

    HAL_FLASH_Lock();

    EXIT_CRITICAL();

    return TRUE;
}

/** ***************************************************************************
 * @name SaveUserConfig - saving of user configuration structure on the 
 *       predefined flash sector
 * @brief
 *
 * @param [in] N/A
 * @retval error (0), no error (1)
 ******************************************************************************/
BOOL  SaveUserConfig(void)
{
    uint16_t size;
    BOOL status;

    size   = sizeof(UserConfigurationStruct);
    status = EEPROM_SaveUserConfig((uint8_t *)&gUserConfiguration, size);

    if(status){
        return TRUE; 
    }

    return FALSE;
}

/** ***************************************************************************
 * @name RestoreDefaultUserConfig - restore user configuration structure from 
 *       the default one
 * @brief
 *
 * @param [in] N/A
 * @retval error (0), no error (1)
 ******************************************************************************/
BOOL RestoreDefaultUserConfig(void)
{
    BOOL valid = TRUE;
    uint32_t sn0 = *(uint32_t *)(0x1FFF7A10);

    memcpy((void*)&gUserConfiguration, (void*)&gDefaultUserConfig, sizeof(UserConfigurationStruct));
    gUserConfiguration.mac[3] = (sn0 >> 16) & 0xff;
	gUserConfiguration.mac[4] = (sn0 >> 8) & 0xff;
	gUserConfiguration.mac[5] = sn0 & 0xff;

    if(!SaveUserConfig()){
        valid = FALSE;
    }
    return valid;
}

/** ***************************************************************************
 * @name userInitConfigureUnit
 * @brief init user configuration structure 
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void userInitConfigureUnit(void)
{
    uint16_t size = sizeof(gUserConfiguration);

    // Validate checksum of user configuration structure
    configValid = EEPROM_ValidateUserConfig(&size);
#ifdef DEVICE_DEBUG
    printf("configValid = %d\r\n", configValid);
#endif

    if (configValid == TRUE) {
        // Here we have validated User configuration image.
        // Load it from eeprom into ram on top of the default configuration
        EEPROM_LoadUserConfig((uint8_t*)&gUserConfiguration, &size);
    } else {
        RestoreDefaultUserConfig();
    }

    update_sys_params();

    base_station_run_update();

    apply_can_ecu_settings(&gEcuConfig);
}

void update_sys_params(void)
{
    uint8_t *ptrUser;
    uint32_t offset, i;

    ptrUser = (uint8_t*)&gUserConfiguration.rawimu_packet_rate;
    for (i = USER_RAWIMU_PACKET_RATE; i <= USER_NMEA_VTG_RATE; i++) {
        offset = update_user_parameter(i, ptrUser, false);
        ptrUser = ptrUser + offset;
    }

}

void apply_can_ecu_settings(void *pConfig)
{
    EcuConfigurationStruct *pEcuConfig = (EcuConfigurationStruct *)pConfig;
    
    pEcuConfig->address           = gUserConfiguration.can_ecu_address;
    pEcuConfig->baudRate          = gUserConfiguration.can_baudrate;
    pEcuConfig->packet_rate       = gUserConfiguration.can_packet_rate;
    pEcuConfig->packet_type       = gUserConfiguration.can_packet_type;
    pEcuConfig->baud_rate_detect_enable = gUserConfiguration.can_baudrate_detect;
}

/** ***************************************************************************
 * @name is_valid_config_command() check incoming SET packets are valid or invalid 
 * @brief a general API of checking supported SET commands
 *        
 * @param [in] ident, identifier message
 *             
 * @retval ACEINNA_J1939_INVALID_IDENTIFIER, ACEINNA_J1939_CONFIG or ACEINNA_J1939_IGNORE
 ******************************************************************************/
ACEINNA_J1939_PACKET_TYPE is_valid_config_command(SAE_J1939_IDENTIFIER_FIELD *ident)
{
    uint8_t pf_val, ps_val;

    if (ident == NULL)
        return ACEINNA_J1939_INVALID_IDENTIFIER;

    pf_val = ident->pdu_format;
    ps_val = ident->pdu_specific;

    if (
        (pf_val == SAE_J1939_PDU_FORMAT_GLOBAL) &&
        ((ps_val == gEcuConfig.save_cfg_ps) ||
         (ps_val == gEcuConfig.status_ps) ||
         (ps_val == gEcuConfig.packet_rate_ps) ||
         (ps_val == gEcuConfig.packet_type_ps)))
    {
        return ACEINNA_J1939_CONFIG;
    }

    return ACEINNA_J1939_IGNORE;
}



/** ****************************************************************************
 * @name update_user_param
 * @brief writes user data into user configuration structure, validates data if
 *        required, updates system parameters  
 * @param [in] pointer to userData payload in the packet
 * @retval N/A
 ******************************************************************************/
BOOL update_user_param(userParamPayload *pld, uint8_t *payloadLen)
{
    uint8_t ret = 0;
    int32_t result = 0;

    if (pld->paramNum < USER_MAX_PARAM) {
        ret = update_user_parameter(pld->paramNum, pld->parameter, false);
        if (ret > 0) {
            update_ethnet_config(pld->paramNum);
        } else {
            result = INVALID_VALUE;
        }
    }
    else {
        result = INVALID_PARAM;
    }

    pld->paramNum = result;
    *payloadLen = 4;

    return TRUE;
}

/** ****************************************************************************
 * @name  get_all_user_params
 * @brief Retrieves specified number of user configuration parameters data for 
 *        sending to the external host starting from specified offset in user 
 *        configuration structure (refer to UserConfigParamOffset structure for
 *        specific value of offsets)
 * @param [in] pointer to userData payload in the packet
 * @retval N/A
 ******************************************************************************/
BOOL get_all_user_params(uint8_t *payload, uint8_t *payloadLen)
{
    uint16_t size = sizeof(UserConfigurationStruct);

    memcpy(payload, &gUserConfiguration, size);

    *payloadLen = size;

    return TRUE;
}

BOOL get_block_user_params(uint8_t *payload, uint8_t *payloadLen)
{
    BOOL ret = FALSE;
    uint8_t sParaId, eParaId;
    int32_t sAddr, eAddr;
    uint8_t *p = (uint8_t *)&gUserConfiguration;

    sParaId = payload[0];
    eParaId = payload[1];
    if (*payloadLen == 2 && eParaId >= sParaId) {
        if (sParaId > USER_DATA_SIZE && eParaId < USER_MAX_PARAM) {
            sAddr = update_user_parameter(sParaId, NULL, TRUE);
            if (eParaId < USER_MAX_PARAM - 1) {
                eAddr = update_user_parameter(eParaId + 1, NULL, TRUE);
            } else {
                eAddr = sizeof(UserConfigurationStruct);
            }
            if (eAddr - sAddr >= 0 && eAddr - sAddr <= 254) {
                memcpy(&payload[2], &p[sAddr], eAddr - sAddr);
                *payloadLen = eAddr - sAddr + 2;
            } else {
                *payloadLen = 2;
            }
            ret = TRUE;
        }
    }
    return ret;
}

static int8_t get_category(uint8_t paramID)
{
    int8_t category = 0;

    if (USER_RAWIMU_PACKET_RATE <= paramID && paramID <= USER_NMEA_VTG_RATE) {
        category = 1;
    } else if (USER_PRI_LEVER_ARM_BX <= paramID && paramID <= USER_ROTATION_RBVZ) {
        category = 2;
    } else if (USER_ETHERNET_ETHMODE <= paramID && paramID <= USER_ETHERNET_MAC) {
        category = 3;
    } else if (USER_STATION_MODE == paramID) {
        category = 4;
    } else if (USER_NTRIP_CLIENT_IP <= paramID && paramID <= USER_NTRIP_CLIENT_PASSWORD) {
        category = 5;
    } else if (USER_ACEINNA_CLIENT_IP <= paramID && paramID <= USER_ACEINNA_CLIENT_PASSWORD) {
        category = 6;
    } else if (USER_NTRIP_SERVER_IP <= paramID && paramID <= USER_NTRIP_SERVER_PASSWORD) {
        category = 7;
    } else if (USER_BASE_POSITION_TYPE <= paramID && paramID <= USER_REFERENCE_HEIGHT) {
        category = 7;
    } else if (USER_CAN_ECU_ADRESS <= paramID && paramID <= USER_CAN_BAUDRATE_DETECT) {
        category = 8;
    } else if (USER_WHEELTICK_PIN_MODE <= paramID && paramID <= USER_ODO_MESGS) {
        category = 9;
    }

    return category;
}

static void refresh_category(uint8_t category)
{
    if (category == 8) {
        apply_can_ecu_settings(&gEcuConfig);

    } else if (category == 3) {
        netif_ethernet_config_changed();

    } else if (category == 4) {
        netif_station_tcp_config_changed();
        base_station_run_update();

    } else if (category == 2) {
        ins_init();
    }
}

BOOL update_block_user_params(uint8_t *payload, uint8_t *payloadLen)
{
    int32_t result = 0;
    uint8_t *p = payload;
    uint8_t t_len = *payloadLen;
    uint8_t len = 0;
    uint8_t category = 0, temp_category;
    int32_t param_len; 

    while (len < t_len)
    {
        if (*p <= USER_DATA_SIZE || *p >= USER_MAX_PARAM) {
            result = INVALID_PARAM;
            break;
        } else {
            temp_category = get_category(*p);
            if (category == 0) {
                category = temp_category;
            } else if (category != temp_category) {
                result = INVALID_CATEGORY;
                break;
            }
            param_len = update_user_parameter(*p, p+1, FALSE);
            if (param_len > 0) {
                p = p + 1 + param_len;
                len = len + 1 + param_len;
            } else {
                result = INVALID_PARAM;
                break;
            }
        }
    }

    if (!result) {
        refresh_category(category);
    }
    
    *(int32_t *)payload = result;
    *payloadLen = 4;

    return TRUE;
}


// api for get/set parameter of configuration
uint8_t get_rawimu_packet_rate(void)
{
    return gUserConfiguration.rawimu_packet_rate;
}
void set_rawimu_packet_rate(uint8_t rate)
{
    config_packet_rate(rate, &gUserConfiguration.rawimu_packet_rate, &rawimu_packet_rate);
}
uint8_t get_bestgnss_packet_rate(void)
{
    return gUserConfiguration.bestgnss_packet_rate;
}
void set_bestgnss_packet_rate(uint8_t rate)
{
    config_packet_rate(rate, &gUserConfiguration.bestgnss_packet_rate, &bestgnss_packet_rate);
}
uint8_t get_inspvax_packet_rate(void)
{
    return gUserConfiguration.inspvax_packet_rate;
}
void set_inspvax_packet_rate(uint8_t rate)
{
    config_packet_rate(rate, &gUserConfiguration.inspvax_packet_rate, &inspvax_packet_rate);
}
uint8_t get_odospeed_packet_rate(void)
{
    return gUserConfiguration.odospeed_packet_rate;
}
void set_odospeed_packet_rate(uint8_t rate)
{
    config_packet_rate(rate, &gUserConfiguration.odospeed_packet_rate, &odospeed_packet_rate);
}
uint8_t get_satellites_packet_rate(void)
{
    return gUserConfiguration.satellites_packet_rate;
}
void set_satellites_packet_rate(uint8_t rate)
{
    config_packet_rate(rate, &gUserConfiguration.satellites_packet_rate, &satellites_packet_rate);
}
uint8_t get_gnins_packet_rate(void)
{
    return gUserConfiguration.nmea_ins_rate;
}
void set_gnins_packet_rate(uint8_t rate)
{
    config_packet_rate(rate, &gUserConfiguration.nmea_ins_rate, &nmea_ins_rate);
}
uint8_t get_gpgga_packet_rate(void)
{
    return gUserConfiguration.nmea_gga_rate;
}
void set_gpgga_packet_rate(uint8_t rate)
{
    config_packet_rate(rate, &gUserConfiguration.nmea_gga_rate, &nmea_gga_rate);
}
uint8_t get_gprmc_packet_rate(void)
{
    return gUserConfiguration.nmea_rmc_rate;
}
void set_gprmc_packet_rate(uint8_t rate)
{
    config_packet_rate(rate, &gUserConfiguration.nmea_rmc_rate, &nmea_rmc_rate);
}
uint8_t get_pashr_packet_rate(void)
{
    return gUserConfiguration.nmea_pashr_rate;
}
void set_pashr_packet_rate(uint8_t rate)
{
    config_packet_rate(rate, &gUserConfiguration.nmea_pashr_rate, &nmea_pashr_rate);
}
uint8_t get_gsa_packet_rate(void)
{
    return gUserConfiguration.nmea_gsa_rate;
}
void set_gsa_packet_rate(uint8_t rate)
{
    config_packet_rate(rate, &gUserConfiguration.nmea_gsa_rate, &nmea_gsa_rate);
}
uint8_t get_zda_packet_rate(void)
{
    return gUserConfiguration.nmea_zda_rate;
}
void set_zda_packet_rate(uint8_t rate)
{
    config_packet_rate(rate, &gUserConfiguration.nmea_zda_rate, &nmea_zda_rate);
}
uint8_t get_vtg_packet_rate(void)
{
    return gUserConfiguration.nmea_vtg_rate;
}
void set_vtg_packet_rate(uint8_t rate)
{
    config_packet_rate(rate, &gUserConfiguration.nmea_vtg_rate, &nmea_vtg_rate);
}

uint16_t get_can_ecu_address(void)
{
    return gUserConfiguration.can_ecu_address;
}

void set_can_ecu_address(uint16_t can_ecu_address)
{
    gUserConfiguration.can_ecu_address = can_ecu_address;
}

void save_ecu_address(uint16_t address)
{
    gUserConfiguration.can_ecu_address  = address;
    SaveUserConfig();
}

uint16_t get_can_baudrate(void)
{
    return gUserConfiguration.can_baudrate;
}

void set_can_baudrate(uint16_t can_baudrate)
{
    gUserConfiguration.can_baudrate = can_baudrate;
}

uint16_t get_can_packet_type(void)
{
    return gUserConfiguration.can_packet_type;
}

void set_can_packet_type(uint16_t can_packet_type)
{
    gUserConfiguration.can_packet_type = can_packet_type;
}

uint16_t get_can_packet_rate(void)
{
    return gUserConfiguration.can_packet_rate;
}

void set_can_packet_rate(uint16_t can_packet_rate)
{
    gUserConfiguration.can_packet_rate = can_packet_rate;
}

uint16_t get_can_termresistor(void)
{
    return gUserConfiguration.can_termresistor;
}

void set_can_termresistor(uint16_t can_termresistor)
{
    gUserConfiguration.can_termresistor = can_termresistor;
}

uint16_t get_can_baudrate_detect(void)
{
    return gUserConfiguration.can_baudrate_detect;
}

void set_can_baudrate_detect(uint16_t can_baudrate_detect)
{
    gUserConfiguration.can_baudrate_detect = can_baudrate_detect;
}

uint16_t get_eth_mode(void)
{
    return gUserConfiguration.eth_mode;
}

void set_eth_mode(uint16_t eth_mode)
{
    gUserConfiguration.eth_mode = eth_mode;
}

uint8_t* get_static_ip(void)
{
    return gUserConfiguration.static_ip;
}

void set_static_ip(uint8_t* ip)
{
    memcpy(gUserConfiguration.static_ip, ip, 4);
}

uint8_t* get_static_netmask(void)
{
    return gUserConfiguration.static_netmask;
}

void set_static_netmask(uint8_t* netmask)
{
    memcpy(gUserConfiguration.static_netmask, netmask, 4);
}

uint8_t* get_static_gateway(void)
{
    return gUserConfiguration.static_gateway;
}

void set_static_gateway(uint8_t* gateway)
{
    memcpy(gUserConfiguration.static_gateway, gateway, 4);
}

uint8_t* get_static_mac(void)
{
    return gUserConfiguration.mac;
}

void set_static_mac(uint8_t* mac)
{
    memcpy(gUserConfiguration.mac, mac, 6);
}

uint32_t get_station_mode(void)
{
    return gUserConfiguration.station_mode;
}

uint8_t set_station_mode(uint32_t station_mode)
{
    if (station_mode <= MODE_NTRIP_SERVER) {
        gUserConfiguration.station_mode = station_mode;
        return 1;
    }
    return 0;
}

const char* get_ntrip_client_ip(void)
{
    return (const char*)gUserConfiguration.ntrip_client_ip;
}

void set_ntrip_client_ip(const char* ip)
{
    memset(gUserConfiguration.ntrip_client_ip, 0 , sizeof(gUserConfiguration.ntrip_client_ip));
    strcpy(gUserConfiguration.ntrip_client_ip, ip);
}

uint16_t get_ntrip_client_port(void)
{
    return gUserConfiguration.ntrip_client_port;
}

void set_ntrip_client_port(uint16_t port)
{
    gUserConfiguration.ntrip_client_port = port;
}

const char* get_ntrip_client_mount_point(void)
{
    return (const char*)gUserConfiguration.ntrip_client_mount_point;
}

void set_ntrip_client_mount_point(const char* mount_point)
{
    memset(gUserConfiguration.ntrip_client_mount_point, 0 , sizeof(gUserConfiguration.ntrip_client_mount_point));
    strcpy(gUserConfiguration.ntrip_client_mount_point, mount_point);
}

const char* get_ntrip_client_username(void)
{
    return (const char*)gUserConfiguration.ntrip_client_username;
}

void set_ntrip_client_username(const char* username)
{
    memset(gUserConfiguration.ntrip_client_username, 0 , sizeof(gUserConfiguration.ntrip_client_username));
    strcpy(gUserConfiguration.ntrip_client_username, username);
}

const char* get_ntrip_client_password(void)
{
    return (const char*)gUserConfiguration.ntrip_client_password;
}

void set_ntrip_client_password(const char* password)
{
    memset(gUserConfiguration.ntrip_client_password, 0 , sizeof(gUserConfiguration.ntrip_client_password));
    strcpy(gUserConfiguration.ntrip_client_password, password);
}

const char* get_aceinna_client_ip(void)
{
    return (const char*)gUserConfiguration.aceinna_client_ip;
}

void set_aceinna_client_ip(const char* ip)
{
    memset(gUserConfiguration.aceinna_client_ip, 0 , sizeof(gUserConfiguration.aceinna_client_ip));
    strcpy(gUserConfiguration.aceinna_client_ip, ip);
}

uint16_t get_aceinna_client_port(void)
{
    return gUserConfiguration.aceinna_client_port;
}

void set_aceinna_client_port(uint16_t port)
{
    gUserConfiguration.aceinna_client_port = port;
}

const char* get_aceinna_client_mount_point(void)
{
    return (const char*)gUserConfiguration.aceinna_client_mount_point;
}

void set_aceinna_client_mount_point(const char* mount_point)
{
    memset(gUserConfiguration.aceinna_client_mount_point, 0 , sizeof(gUserConfiguration.aceinna_client_mount_point));
    strcpy(gUserConfiguration.aceinna_client_mount_point, mount_point);
}

const char* get_aceinna_client_username(void)
{
    return (const char*)gUserConfiguration.aceinna_client_username;
}

void set_aceinna_client_username(const char* username)
{
    memset(gUserConfiguration.aceinna_client_username, 0 , sizeof(gUserConfiguration.aceinna_client_username));
    strcpy(gUserConfiguration.aceinna_client_username, username);
}

const char* get_aceinna_client_password(void)
{
    return (const char*)gUserConfiguration.aceinna_client_password;
}

void set_aceinna_client_password(const char* password)
{
    memset(gUserConfiguration.aceinna_client_password, 0 , sizeof(gUserConfiguration.aceinna_client_password));
    strcpy(gUserConfiguration.aceinna_client_password, password);
}

const char* get_ntrip_server_ip(void)
{
    return (const char*)gUserConfiguration.ntrip_server_ip;
}

void set_ntrip_server_ip(const char* ip)
{
    memset(gUserConfiguration.ntrip_server_ip, 0 , sizeof(gUserConfiguration.ntrip_server_ip));
    strcpy(gUserConfiguration.ntrip_server_ip, ip);
}

uint16_t get_ntrip_server_port(void)
{
    return gUserConfiguration.ntrip_server_port;
}

void set_ntrip_server_port(uint16_t port)
{
    gUserConfiguration.ntrip_server_port = port;
}

const char* get_ntrip_server_mount_point(void)
{
    return (const char*)gUserConfiguration.ntrip_server_mount_point;
}

void set_ntrip_server_mount_point(const char* mount_point)
{
    memset(gUserConfiguration.ntrip_server_mount_point, 0 , sizeof(gUserConfiguration.ntrip_server_mount_point));
    strcpy(gUserConfiguration.ntrip_server_mount_point, mount_point);
}

const char* get_ntrip_server_password(void)
{
    return (const char*)gUserConfiguration.ntrip_server_password;
}

void set_ntrip_server_password(const char* password)
{
    memset(gUserConfiguration.ntrip_server_password, 0 , sizeof(gUserConfiguration.ntrip_server_password));
    strcpy(gUserConfiguration.ntrip_server_password, password);
}

uint16_t get_base_position_type(void)
{
    return gUserConfiguration.base_position_type;
}

void set_base_position_type(uint16_t base_position_type)
{
    gUserConfiguration.base_position_type = base_position_type;
}

uint16_t get_station_id(void)
{
    return gUserConfiguration.station_id;
}

void set_station_id(uint16_t station_id)
{
    gUserConfiguration.station_id = station_id;
}

double get_antenna_height(void)
{
    return gUserConfiguration.antenna_height;
}

void set_antenna_height(double antenna_height)
{
    gUserConfiguration.antenna_height = antenna_height;
}

double get_reference_latitude(void)
{
    return gUserConfiguration.reference_latitude;
}

void set_reference_latitude(double reference_latitude)
{
    gUserConfiguration.reference_latitude = reference_latitude;
}

double get_reference_longitude(void)
{
    return gUserConfiguration.reference_longitude;
}

void set_reference_longitude(double reference_longitude)
{
    gUserConfiguration.reference_longitude = reference_longitude;
}

double get_reference_height(void)
{
    return gUserConfiguration.reference_height;
}

void set_reference_height(double reference_height)
{
    gUserConfiguration.reference_height = reference_height;
}

float get_pri_lever_arm_bx(void)
{
    return gUserConfiguration.pri_lever_arm_bx;
}

void set_pri_lever_arm_bx(float pri_lever_arm_bx)
{
    gUserConfiguration.pri_lever_arm_bx = pri_lever_arm_bx;
}

float get_pri_lever_arm_by(void)
{
    return gUserConfiguration.pri_lever_arm_by;
}

void set_pri_lever_arm_by(float pri_lever_arm_by)
{
    gUserConfiguration.pri_lever_arm_by = pri_lever_arm_by;
}

float get_pri_lever_arm_bz(void)
{
    return gUserConfiguration.pri_lever_arm_bz;
}

void set_pri_lever_arm_bz(float pri_lever_arm_bz)
{
    gUserConfiguration.pri_lever_arm_bz = pri_lever_arm_bz;
}

float get_vrp_lever_arm_bx(void)
{
    return gUserConfiguration.vrp_lever_arm_bx;
}
void set_vrp_lever_arm_bx(float vrp_lever_arm_bx)
{
    gUserConfiguration.vrp_lever_arm_bx = vrp_lever_arm_bx;
}

float get_vrp_lever_arm_by(void)
{
    return gUserConfiguration.vrp_lever_arm_by;
}
void set_vrp_lever_arm_by(float vrp_lever_arm_by)
{
    gUserConfiguration.vrp_lever_arm_by = vrp_lever_arm_by;
}

float get_vrp_lever_arm_bz(void)
{
    return gUserConfiguration.vrp_lever_arm_bz;
}
void set_vrp_lever_arm_bz(float vrp_lever_arm_bz)
{
    gUserConfiguration.vrp_lever_arm_bz = vrp_lever_arm_bz;
}

float get_user_lever_arm_bx(void)
{
    return gUserConfiguration.user_lever_arm_bx;
}

void set_user_lever_arm_bx(float user_lever_arm_bx)
{
    gUserConfiguration.user_lever_arm_bx = user_lever_arm_bx;
}

float get_user_lever_arm_by(void)
{
    return gUserConfiguration.user_lever_arm_by;
}

void set_user_lever_arm_by(float user_lever_arm_by)
{
    gUserConfiguration.user_lever_arm_by = user_lever_arm_by;
}

float get_user_lever_arm_bz(void)
{
    return gUserConfiguration.user_lever_arm_bz;
}

void set_user_lever_arm_bz(float user_lever_arm_bz)
{
    gUserConfiguration.user_lever_arm_bz = user_lever_arm_bz;
}

float get_rotation_rbvx(void)
{
    return gUserConfiguration.rotation_rbvx;
}

void set_rotation_rbvx(float rotation_rbvx)
{
    gUserConfiguration.rotation_rbvx = rotation_rbvx;
}

float get_rotation_rbvy(void)
{
    return gUserConfiguration.rotation_rbvy;
}

void set_rotation_rbvy(float rotation_rbvy)
{
    gUserConfiguration.rotation_rbvy = rotation_rbvy;
}

float get_rotation_rbvz(void)
{
    return gUserConfiguration.rotation_rbvz;
}

void set_rotation_rbvz(float rotation_rbvz)
{
    gUserConfiguration.rotation_rbvz = rotation_rbvz;
}


float* get_user_ins_para(void)
{
    return &(gUserConfiguration.pri_lever_arm_bx);
}


#define STDOUT_FILENO   1
#define STDERR_FILENO   2

__weak int _write(int fd, char* ptr, int len)
{
    if (fd == STDOUT_FILENO || fd == STDERR_FILENO)
    {
        //uart_write_bytes(UART_DEBUG,ptr,len,1);
        HAL_UART_Transmit(&huart_debug, (uint8_t *)ptr, len, 0xFFFF); // FOR TESTING ONLY?
        return len;
    }
    return 0;
}

int get_wheeltick_pin_mode()
{
    return gUserConfiguration.wheeltick_pin_mode;
}
