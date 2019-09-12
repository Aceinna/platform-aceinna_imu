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
#include "UserMessagingUART.h"

/// User defined configuration strucrture
///Please notice, that parameters are 64 bit to accomodate double types as well as longer string types

typedef struct {
    uint64_t           dataCRC;             /// CRC of user configuration structure CRC-16
    uint64_t           dataSize;            /// Size of the user configuration structure 
    
    int64_t            userUartBaudRate;    /// baudrate of user UART, bps. 
                                            /// valid options are:
                                            /// 4800
                                            /// 9600
                                            /// 19200
                                            /// 38400
                                            /// 57600
                                            /// 115200
                                            /// 230400
                                            /// 460800
    uint8_t            userPacketType[8];   /// User packet to be continiously sent by unit
                                            /// Packet types defined in structure UserOutPacketType
                                            /// in file UserMessaging.h
                                            
    int64_t            userPacketRate;      /// Packet rate for continiously output packet, Hz.
                                            /// Valid settings are: 0 ,2, 5, 10, 20, 25, 50, 100, 200 

    int64_t            lpfAccelFilterFreq;  /// built-in lpf filter cutoff frequency selection for accelerometers   
    int64_t            lpfRateFilterFreq;   /// built-in lpf filter cutoff frequency selection for rate sensors   
                                            /// Options are:
                                            /// 0  -  Filter turned off
                                            /// 50 -  Butterworth LPF 50HZ
                                            /// 20 -  Butterworth LPF 20HZ
                                            /// 10 -  Butterworth LPF 10HZ
                                            /// 05 -  Butterworth LPF 5HZ
                                            /// 02 -  Butterworth LPF 2HZ
                                            /// 25 -  Butterworth LPF 25HZ
                                            /// 40 -  Butterworth LPF 40HZ
    
    uint8_t           orientation[8];         /// unit orientation in format 0x0000000000ddrrff
                                            /// where   dd - down axis, rr - right axis, ff - forward axis
                                            /// next axis values a valid :  
                                            /// 'X' (0x58) -> plus  X, 'Y' (0x59) -> plus Y,  'Z' (0x5a) -> plus Z
                                            /// 'x' (0x78) -> minus X, 'y' (0x79) -> minus Y, 'z' (0x7a) ->minusZ
    
    //***************************************************************************************
    // here is the border between arbitrary parameters and platform configuration parameters
    //***************************************************************************************

    int64_t          gpsBaudRate;           /// baudrate of GPS UART, bps. 
                                            /// valid options are:
                                            /// 4800
                                            /// 9600
                                            /// 19200
                                            /// 38400
                                            /// 57600
                                            /// 115200
                                            /// 230400
    int64_t          gpsProtocol;           /// protocol of GPS receicer. 
                                            /// so far valid options are:
                                            /// NMEA_TEXT
                                            /// NOVATEL_BINARY
    // place new arbitrary configuration parameters here
    // parameter size should even to 4 bytes
    // Add parameter offset in UserConfigParamOffset structure if validation or
    // special processing required 

    double hardIron_X;
    double hardIron_Y;
    double softIron_Ratio;
    double softIron_Angle;
} UserConfigurationStruct;

typedef enum {
//*****************************************************************************************
// add system parameters here and reassign USER_LAST_SYSTEM_PARAM (DO NOT CHANGE THIS!!!)
    USER_CRC                       = 0,
    USER_DATA_SIZE                    ,   // 1
    USER_USER_BAUD_RATE               ,   // 2  order of next 4 parameters
    USER_USER_PACKET_TYPE             ,   // 3  of required unit output bandwidth
    USER_USER_PACKET_RATE             ,   // 4 
    USER_LPF_ACCEL_TYPE               ,   // 5  prefered LPF filter type for accelerometer
    USER_LPF_RATE_TYPE                ,   // 6  prefered LPF filter type for rate sensor
    USER_ORIENTATION                  ,   // 7  unit orientation
    USER_LAST_SYSTEM_PARAM = USER_ORIENTATION,
//*****************************************************************************************
// add parameter enumerator here while adding new parameter in user UserConfigurationStruct
    USER_GPS_BAUD_RATE                ,
    USER_GPS_PROTOCOL                 ,
    USER_HARD_IRON_X                  ,
    USER_HARD_IRON_Y                  ,
    USER_SOFT_IRON_RATIO              ,
    USER_SOFT_IRON_ANGLE              ,
    USER_MAX_PARAM
} UserConfigParamNumber;

#define MAX_SYSTEM_PARAM USER_ORIENTATION

extern int userPacketOut;

#define INVALID_PARAM           -1
#define INVALID_VALUE           -2
#define INVALID_PAYLOAD_SIZE    -3



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
extern BOOL      UpdateUserConfig(userConfigPayload*  pld, uint8_t *payloadLen);
extern BOOL      UpdateUserParam(userParamPayload*  pld, uint8_t *payloadLen);
extern BOOL      UpdateAllUserParams(allUserParamsPayload*  pld, uint8_t *payloadLen);
extern BOOL      SaveUserConfig(void);
extern BOOL      RestoreDefaultUserConfig(void);
extern BOOL      GetUserConfig(userConfigPayload*  pld, uint8_t *payloadLen);
extern BOOL      GetUserParam(userParamPayload*  pld, uint8_t *payloadLen);
extern BOOL      GetAllUserParams(allUserParamsPayload*  pld, uint8_t *payloadLen);
extern BOOL      UpdateUserParameter(uint32_t number, uint64_t data, BOOL fApply);
extern BOOL      UpdateSystemParameter(uint32_t offset, uint64_t data, BOOL fApply);
BOOL             setUserPacketType(uint8_t* type, BOOL fApply);

#endif /* USER_CONFIGURATION_H */


