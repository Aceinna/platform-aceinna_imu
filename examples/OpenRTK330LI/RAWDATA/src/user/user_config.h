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

/// User defined configuration strucrture
///Please notice, that parameters are 64 bit to accomodate double types as well as longer string types

#pragma pack(4)

typedef struct {
    uint64_t            dataCRC;             /// CRC of user configuration structure CRC-16
    uint64_t            dataSize;            /// Size of the user configuration structure 

    // place new arbitrary configuration parameters here
    // parameter size should even to 4 bytes
    // Add parameter offset in UserConfigParamOffset structure if validation or
    // special processing required 
    uint32_t profile;
    float leverArmBx;
    float leverArmBy;
    float leverArmBz;
    float pointOfInterestBx;
    float pointOfInterestBy;
    float pointOfInterestBz;
    float rotationRbvx;
    float rotationRbvy;
    float rotationRbvz;

    // ethnet
    uint8_t ethMode;
    uint8_t staticIp[4];
	uint8_t netmask[4];
	uint8_t gateway[4];
	uint8_t mac[6];
    
    // ntrip
	uint8_t ip[23];
	uint16_t port;
	uint8_t mountPoint[20];
    uint8_t username[16];
	uint8_t password[24];

} UserConfigurationStruct;

#pragma pack()



typedef enum {
//*****************************************************************************************
// add system parameters here and reassign USER_LAST_SYSTEM_PARAM (DO NOT CHANGE THIS!!!)
    USER_CRC                       = 0,
    USER_DATA_SIZE                    ,   // 1

    USER_LAST_SYSTEM_PARAM = USER_DATA_SIZE, 
//*****************************************************************************************
// add parameter enumerator here while adding new parameter in user UserConfigurationStruct
    USER_PROFILE                      ,
    USER_LEVER_ARM_BX                 ,
    USER_LEVER_ARM_BY                 ,
    USER_LEVER_ARM_BZ                 ,
    USER_POINT_OF_INTEREST_BX         ,
    USER_POINT_OF_INTEREST_BY         ,
    USER_POINT_OF_INTEREST_BZ         ,
    USER_ROTATION_RBVX                ,
    USER_ROTATION_RBVY                ,
    USER_ROTATION_RBVZ                ,

    USER_ETHERNET_ETHMODE             ,
    USER_ETHERNET_IP                  ,
    USER_ETHERNET_NETMASK             ,
    USER_ETHERNET_GATEWAY             ,
    USER_ETHERNET_MAC                 ,

    USER_NTRIP_IP                     ,
    USER_NTRIP_PORT                   ,
    USER_NTRIP_MOUNTPOINT             ,
    USER_NTRIP_USERNAME               ,
    USER_NTRIP_PASSWORD               ,

    USER_MAX_PARAM
} UserConfigParamNumber;


#define INVALID_PARAM           -1
#define INVALID_VALUE           -2
#define INVALID_PAYLOAD_SIZE    -3

// app configuartion address
#define APP_USER_CONFIG_ADDR    0x080C0000
#define BOOT_FLAG_ADDR    0x080A0000

// ethnet
#define ETHMODE_DHCP 0
#define ETHMODE_STATIC 1

extern UserConfigurationStruct gUserConfiguration;

extern void userInitConfigureUnit(void);
extern BOOL UpdateUserParam(userParamPayload*  pld, uint8_t *payloadLen);
extern BOOL SaveUserConfig(void);
extern BOOL RestoreDefaultUserConfig(void);
extern BOOL GetAllUserParams(allUserParamsPayload*  pld, uint8_t *payloadLen);
extern uint8_t  UpdateUserParameter(uint32_t number, uint8_t* data);


BOOL EEPROM_SaveUserConfig(uint8_t *ptrToUserConfigStruct, int userConfigStructLen);
BOOL EEPROM_LoadUserConfig(uint8_t *ptrToUserConfigInRam, int *userConfigSize);

void initinsconfigfm(UserConfigurationStruct* p_gUserConfiguration);
float* get_user_ins_para();

#endif /* _USER_CONFIG_H */
