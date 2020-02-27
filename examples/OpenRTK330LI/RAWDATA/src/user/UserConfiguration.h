/*******************************************************************************
 * File:   UserConfiguration.h
 * Created on JAn 25, 2017
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

#ifndef USER_CONFIGURATION_H
#define USER_CONFIGURATION_H

#include <stdint.h>

#include "GlobalConstants.h"
#include "UserMessaging.h"

/// User defined configuration strucrture
///Please notice, that parameters are 64 bit to accomodate double types as well as longer string types

#pragma pack(4)

typedef struct {
    uint64_t            dataCRC;             /// CRC of user configuration structure CRC-16
    uint64_t            dataSize;            /// Size of the user configuration structure 

    uint64_t            userUartBaudRate;    /// baudrate of user UART, bps. 
                                            /// valid options are:
                                            /// 4800
                                            /// 9600
                                            /// 19200
                                            /// 38400
                                            /// 57600
                                            /// 115200
                                            /// 230400
                                            /// 460800
    uint8_t             userPacketType[8];   /// User packet to be continiously sent by unit
                                            /// Packet types defined in structure UserOutPacketType
                                            /// in file UserMessaging.h
                                            
    uint64_t            userPacketRate;      /// Packet rate for continiously output packet, Hz.
                                            /// Valid settings are: 0 ,2, 5, 10, 20, 25, 50, 100, 200 

    uint64_t            lpfAccelFilterFreq;  /// built-in lpf filter cutoff frequency selection for accelerometers   
    uint64_t            lpfRateFilterFreq;   /// built-in lpf filter cutoff frequency selection for rate sensors   
                                            /// Options are:
                                            /// 0  -  Filter turned off
                                            /// 50 -  Butterworth LPF 50HZ
                                            /// 20 -  Butterworth LPF 20HZ
                                            /// 10 -  Butterworth LPF 10HZ
                                            /// 05 -  Butterworth LPF 5HZ
                                            /// 02 -  Butterworth LPF 2HZ
                                            /// 25 -  Butterworth LPF 25HZ
                                            /// 40 -  Butterworth LPF 40HZ
    
    uint8_t             orientation[8];     /// unit orientation in format 0x0000000000ddrrff
                                            /// where   dd - down axis, rr - right axis, ff - forward axis
                                            /// next axis values a valid :  
                                            /// 'X' (0x58) -> plus  X, 'Y' (0x59) -> plus Y,  'Z' (0x5a) -> plus Z
                                            /// 'x' (0x78) -> minus X, 'y' (0x79) -> minus Y, 'z' (0x7a) ->minusZ

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
    uint8_t rtkType;
	uint8_t ip[22];
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
    USER_UART_BAUD_RATE               ,   // 2  order of next 4 parameters
    USER_UART_PACKET_TYPE             ,   // 3  of required unit output bandwidth
    USER_UART_PACKET_RATE             ,   // 4 
    USER_LPF_ACCEL_TYPE               ,   // 5  prefered LPF filter type for accelerometer
    USER_LPF_RATE_TYPE                ,   // 6  prefered LPF filter type for rate sensor
    USER_ORIENTATION                  ,   // 7  unit orientation
    USER_LAST_SYSTEM_PARAM = USER_ORIENTATION, 
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

    USER_NTRIP_RTKTYPE                ,
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

// ntrip config define
#define LocalRTK 0
#define CloudRTK 1

//
extern UserConfigurationStruct gUserConfiguration;


extern void      InitializeUserAlgorithmParams(void);
extern BOOL      validateUserConfigInEeprom(int *numParams);
extern uint32_t  getUserParamFromEeprom(uint32_t offset);
extern BOOL      saveUserConfigInEeprom(uint8_t *ptrToUserConfigStruct, int userConfigStructLen);
extern int       getUserPayloadLength(void);
extern BOOL      checkIfUserEEPROMErased(void);
extern BOOL      SaveUserData(void);
extern BOOL      loadUserConfigFromEeprom(uint8_t *ptrToUserConfigInRam, int *userConfigSize);
//extern UcbPacketType checkPacketType(UcbPacketCodeType receivedCode);
extern void      userPacketTypeToBytes(uint8_t bytes[]);
extern BOOL      UpdateUserParam(userParamPayload*  pld, uint8_t *payloadLen);
extern BOOL      SaveUserConfig(void);
extern BOOL      RestoreDefaultUserConfig(void);
extern BOOL      GetAllUserParams(allUserParamsPayload*  pld, uint8_t *payloadLen);
extern BOOL      UpdateSystemParameter(uint32_t number, uint64_t data, BOOL fApply);
extern uint8_t   UpdateUserParameter(uint32_t number, uint8_t* data, BOOL fApply);


BOOL EEPROM_SaveUserConfig(uint8_t *ptrToUserConfigStruct, int userConfigStructLen);
BOOL EEPROM_LoadUserConfig(uint8_t *ptrToUserConfigInRam, int *userConfigSize);


#ifndef RTK_INS
#define RTK_INS
#endif

#endif /* USER_CONFIGURATION_H */
