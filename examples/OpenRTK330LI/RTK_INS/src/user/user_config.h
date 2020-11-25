/** ***************************************************************************
 * @file user_config.h
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
*******************************************************************************/

#ifndef _USER_CONFIG_H
#define _USER_CONFIG_H

#include <stdint.h>

#include "constants.h"
#include "user_message.h"

#pragma pack(4)

typedef struct {
    uint32_t mesgID;
    uint8_t	startbit;	
    uint8_t length;
    uint8_t	endian;
    uint8_t sign;
    double factor;
    double offset;
    uint8_t unit;
    uint8_t source;
    uint16_t usage;
} CanMesgOdoStruct;

#pragma pack()

/// User defined configuration strucrture
#pragma pack(4)
typedef struct {
    uint16_t dataCRC;   /// CRC of user configuration structure CRC-16
    uint16_t dataSize;  /// Size of the user configuration structure 

    // user uart packets 
    // aceinna packets
    uint8_t rawimu_packet_rate;
    uint8_t bestgnss_packet_rate;
    uint8_t inspvax_packet_rate;
    uint8_t odospeed_packet_rate;
    uint8_t satellites_packet_rate;
    // nmea
    uint8_t nmea_ins_rate;
    uint8_t nmea_gga_rate;
    uint8_t nmea_rmc_rate;
    uint8_t nmea_pashr_rate;
    uint8_t nmea_gsa_rate;
    uint8_t nmea_zda_rate;
    uint8_t nmea_vtg_rate;
    // uint8_t nmea_pashr_rate;

    // leverarm
    float pri_lever_arm_bx;
    float pri_lever_arm_by;
    float pri_lever_arm_bz;
    float vrp_lever_arm_bx; // vehicle reference point
    float vrp_lever_arm_by;
    float vrp_lever_arm_bz;
    float user_lever_arm_bx;
    float user_lever_arm_by;
    float user_lever_arm_bz;
    float rotation_rbvx;
    float rotation_rbvy;
    float rotation_rbvz;

    // ethnet
    uint16_t eth_mode;
    uint8_t static_ip[4];
	uint8_t static_netmask[4];
	uint8_t static_gateway[4];
	uint8_t mac[6];

    // work mode for ethnet
    uint32_t station_mode;

    // ntrip
    char ntrip_client_ip[26];
	uint16_t ntrip_client_port;
	char ntrip_client_mount_point[20];
    char ntrip_client_username[20];
	char ntrip_client_password[20];

    // aceinna client
    char aceinna_client_ip[26];
	uint16_t aceinna_client_port;
	char aceinna_client_mount_point[20];
    char aceinna_client_username[20];
	char aceinna_client_password[20];

    // ntrip server
	char ntrip_server_ip[26];
	uint16_t ntrip_server_port;
	char ntrip_server_mount_point[20];
	char ntrip_server_password[20];
    // for base station
    uint16_t base_position_type;
    uint16_t station_id;
    double   antenna_height;
    double   reference_latitude;
    double   reference_longitude;
    double   reference_height;
    
    // can
    uint16_t can_ecu_address;
    uint16_t can_baudrate;
    uint16_t can_packet_type;
    uint16_t can_packet_rate;
    uint16_t can_termresistor;
    uint16_t can_baudrate_detect;

    uint32_t wheeltick_pin_mode;
    uint32_t can_mode;
    uint32_t gears[4];
    CanMesgOdoStruct odo_mesg[3];

} UserConfigurationStruct;
extern UserConfigurationStruct gUserConfiguration;
#pragma pack()

typedef enum {
    USER_DATA_CRC                   = 0,
    USER_DATA_SIZE                    ,

    USER_RAWIMU_PACKET_RATE           ,
    USER_BESTGNSS_PACKET_RATE         ,
    USER_INSPVA_PACKET_RATE           ,
    USER_ODOSPEED_PACKET_RATE         ,
    USER_SATELLITES_PACKET_RATE       ,
    USER_NMEA_INS_RATE                ,
    USER_NMEA_GGA_RATE                ,
    USER_NMEA_RMC_RATE                ,
    USER_NMEA_PASHR_RATE              ,
    USER_NMEA_GSA_RATE                ,
    USER_NMEA_ZDA_RATE                ,
    USER_NMEA_VTG_RATE                ,

    USER_PRI_LEVER_ARM_BX                 ,
    USER_PRI_LEVER_ARM_BY                 ,
    USER_PRI_LEVER_ARM_BZ                 ,
    USER_VRP_LEVER_ARM_BX                 ,
    USER_VRP_LEVER_ARM_BY                 ,
    USER_VRP_LEVER_ARM_BZ                 ,
    USER_USER_LEVER_ARM_BX         ,
    USER_USER_LEVER_ARM_BY         ,
    USER_USER_LEVER_ARM_BZ         ,
    USER_ROTATION_RBVX                ,
    USER_ROTATION_RBVY                ,
    USER_ROTATION_RBVZ                ,

    USER_ETHERNET_ETHMODE             ,
    USER_ETHERNET_STATIC_IP           ,
    USER_ETHERNET_STATIC_NETMASK      ,
    USER_ETHERNET_STATIC_GATEWAY      ,
    USER_ETHERNET_MAC                 ,

    USER_STATION_MODE                 ,

    USER_NTRIP_CLIENT_IP              ,
    USER_NTRIP_CLIENT_PORT            ,
    USER_NTRIP_CLIENT_MOUNT_POINT     ,
    USER_NTRIP_CLIENT_USERNAME        ,
    USER_NTRIP_CLIENT_PASSWORD        ,

    USER_ACEINNA_CLIENT_IP            ,
    USER_ACEINNA_CLIENT_PORT          ,
    USER_ACEINNA_CLIENT_MOUNT_POINT   ,
    USER_ACEINNA_CLIENT_USERNAME      ,
    USER_ACEINNA_CLIENT_PASSWORD      ,

    USER_NTRIP_SERVER_IP              ,
    USER_NTRIP_SERVER_PORT            ,
    USER_NTRIP_SERVER_MOUNT_POINT     ,
    USER_NTRIP_SERVER_PASSWORD        ,

    USER_BASE_POSITION_TYPE           ,
    USER_STATION_ID                   ,
    USER_ANTENNA_HEIGHT               ,
    USER_REFERENCE_LATITUDE           ,
    USER_REFERENCE_LONGITUDE          ,
    USER_REFERENCE_HEIGHT             ,

    USER_CAN_ECU_ADRESS               ,
    USER_CAN_BAUDRATE                 ,
    USER_CAN_PACKET_TYPE              ,
    USER_CAN_PACKET_RATE              ,
    USER_CAN_TERMRESISTOR             ,
    USER_CAN_BAUDRATE_DETECT          ,

    USER_WHEELTICK_PIN_MODE           ,
    USER_CAN_MODE                     ,
    USER_ODO_GEARS                    ,
    USER_ODO_MESGS                    ,
    
    USER_MAX_PARAM
} UserConfigParamNumber;

typedef enum {
    ETHMODE_DHCP        = 0,
    ETHMODE_STATIC,
} EthernetMode;

typedef enum {
    MODE_NTRIP_CLIENT        = 0,
    MODE_OPENARC_CLIENT,
    MODE_NTRIP_SERVER,
} EthernetUserMode;

typedef enum {
    BASE_POSITION_REFERENCE     = 0,
    BASE_POSITION_SPP,
    BASE_POSITION_RTK,
    BASE_POSITION_AUTO
} BasePositionType;


#define INVALID_PARAM           -1
#define INVALID_VALUE           -2
#define INVALID_CATEGORY        -3

// app configuartion address
#define APP_USER_CONFIG_ADDR    0x080C0000
#define BOOT_VERSION_ADDR       0x2004ff20
#define BOOT_VERSION_MAX_LEN    40

char* get_boot_version(void);

extern char st_sdk_version[30];

char* get_sdk_version(void);
void set_sdk_version(char* ver);

BOOL EEPROM_ValidateUserConfig(uint16_t *userConfigSize);
BOOL EEPROM_LoadUserConfig(uint8_t *ptrToUserConfigInRam, uint16_t *ptrUserConfigSize);
BOOL EEPROM_SaveUserConfig(uint8_t *ptrToUserConfigStruct, uint16_t userConfigSize);
BOOL SaveUserConfig(void);
BOOL RestoreDefaultUserConfig(void);

void userInitConfigureUnit(void);
void update_sys_params(void);
void apply_can_ecu_settings(void *pConfig);
int32_t update_user_parameter(uint32_t number, uint8_t* data, BOOL offset);
BOOL update_ethnet_config(uint32_t number);
BOOL update_user_param(userParamPayload*  pld, uint8_t *payloadLen);
BOOL get_all_user_params(uint8_t *payload, uint8_t *payloadLen);
BOOL get_block_user_params(uint8_t *payload, uint8_t *payloadLen);
BOOL update_block_user_params(uint8_t *payload, uint8_t *payloadLen);

// api: get/set user parameter
uint8_t get_rawimu_packet_rate(void);
void set_rawimu_packet_rate(uint8_t rate);
uint8_t get_bestgnss_packet_rate(void);
void set_bestgnss_packet_rate(uint8_t rate);
uint8_t get_inspvax_packet_rate(void);
void set_inspvax_packet_rate(uint8_t rate);
uint8_t get_odospeed_packet_rate(void);
void set_odospeed_packet_rate(uint8_t rate);
uint8_t get_satellites_packet_rate(void);
void set_satellites_packet_rate(uint8_t rate);
uint8_t get_gnins_packet_rate(void);
void set_gnins_packet_rate(uint8_t rate);
uint8_t get_gpgga_packet_rate(void);
void set_gpgga_packet_rate(uint8_t rate);
uint8_t get_gprmc_packet_rate(void);
void set_gprmc_packet_rate(uint8_t rate);
uint8_t get_pashr_packet_rate(void);
void set_pashr_packet_rate(uint8_t rate);
uint8_t get_gsa_packet_rate(void);
void set_gsa_packet_rate(uint8_t rate);
uint8_t get_zda_packet_rate(void);
void set_zda_packet_rate(uint8_t rate);
uint8_t get_vtg_packet_rate(void);
void set_vtg_packet_rate(uint8_t rate);

float get_pri_lever_arm_bx(void);
void set_pri_lever_arm_bx(float pri_lever_arm_bx);
float get_pri_lever_arm_by(void);
void set_pri_lever_arm_by(float pri_lever_arm_by);
float get_pri_lever_arm_bz(void);
void set_pri_lever_arm_bz(float pri_lever_arm_bz);
float get_vrp_lever_arm_bx(void);
void set_vrp_lever_arm_bx(float vrp_lever_arm_bx);
float get_vrp_lever_arm_by(void);
void set_vrp_lever_arm_by(float vrp_lever_arm_by);
float get_vrp_lever_arm_bz(void);
void set_vrp_lever_arm_bz(float vrp_lever_arm_bz);
float get_user_lever_arm_bx(void);
void set_user_lever_arm_bx(float user_lever_arm_bx);
float get_user_lever_arm_by(void);
void set_user_lever_arm_by(float user_lever_arm_by);
float get_user_lever_arm_bz(void);
void set_user_lever_arm_bz(float user_lever_arm_bz);
float get_rotation_rbvx(void);
void set_rotation_rbvx(float rotation_rbvx);
float get_rotation_rbvy(void);
void set_rotation_rbvy(float rotation_rbvy);
float get_rotation_rbvz(void);
void set_rotation_rbvz(float rotation_rbvz);

uint16_t get_eth_mode(void);
void set_eth_mode(uint16_t eth_mode);
uint8_t* get_static_ip(void);
void set_static_ip(uint8_t* ip);
uint8_t* get_static_netmask(void);
void set_static_netmask(uint8_t* netmask);
uint8_t* get_static_gateway(void);
void set_static_gateway(uint8_t* gateway);
uint8_t* get_static_mac(void);
void set_static_mac(uint8_t* mac);

uint32_t get_station_mode(void);
uint8_t set_station_mode(uint32_t station_mode);

const char* get_ntrip_client_ip(void);
void set_ntrip_client_ip(const char* ip);
uint16_t get_ntrip_client_port(void);
void set_ntrip_client_port(uint16_t port);
const char* get_ntrip_client_mount_point(void);
void set_ntrip_client_mount_point(const char* mount_point);
const char* get_ntrip_client_username(void);
void set_ntrip_client_username(const char* username);
const char* get_ntrip_client_password(void);
void set_ntrip_client_password(const char* password);

const char* get_aceinna_client_ip(void);
void set_aceinna_client_ip(const char* ip);
uint16_t get_aceinna_client_port(void);
void set_aceinna_client_port(uint16_t port);
const char* get_aceinna_client_mount_point(void);
void set_aceinna_client_mount_point(const char* mount_point);
const char* get_aceinna_client_username(void);
void set_aceinna_client_username(const char* username);
const char* get_aceinna_client_password(void);
void set_aceinna_client_password(const char* password);

const char* get_ntrip_server_ip(void);
void set_ntrip_server_ip(const char* ip);
uint16_t get_ntrip_server_port(void);
void set_ntrip_server_port(uint16_t port);
const char* get_ntrip_server_mount_point(void);
void set_ntrip_server_mount_point(const char* mount_point);
const char* get_ntrip_server_password(void);
void set_ntrip_server_password(const char* password);

uint16_t get_base_position_type(void);
void set_base_position_type(uint16_t base_position_type);
uint16_t get_station_id(void);
void set_station_id(uint16_t station_id);
double get_antenna_height(void);
void set_antenna_height(double antenna_height);

double get_reference_latitude(void);
void set_reference_latitude(double reference_latitude);
double get_reference_longitude(void);
void set_reference_longitude(double reference_longitude);
double get_reference_height(void);
void set_reference_height(double reference_height);

uint16_t get_can_ecu_address(void);
void set_can_ecu_address(uint16_t can_ecu_address);
void save_ecu_address(uint16_t address);
uint16_t get_can_baudrate(void);
void set_can_baudrate(uint16_t can_baudrate);
uint16_t get_can_packet_type(void);
void set_can_packet_type(uint16_t can_packet_type);
uint16_t get_can_packet_rate(void);
void set_can_packet_rate(uint16_t can_packet_rate);
uint16_t get_can_termresistor(void);
void set_can_termresistor(uint16_t can_termresistor);
uint16_t get_can_baudrate_detect(void);
void set_can_baudrate_detect(uint16_t can_baudrate_detect);


float* get_user_ins_para(void);
void ins_init(void);
int get_wheeltick_pin_mode();

#endif /* _USER_CONFIG_H */
