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
#include "filter.h"

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

    int64_t          uartGpsBaudRate;           /// baudrate of GPS UART, bps. 
                                            /// valid options are:
                                            /// 4800
                                            /// 9600
                                            /// 19200
                                            /// 38400
                                            /// 57600
                                            /// 115200
                                            /// 230400
    int64_t          uartGpsProtocol;           /// protocol of GPS receicer. 
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
    uint64_t appBehavior;

    //***************************************************************************************
    // SPI-Related specific parameters
    //***************************************************************************************
    int64_t           spiSyncRate;          /// SPI data ready rate
                                            /// 0 - 0 Hz   
                                            /// 1 - 200 Hz   
                                            /// 2 - 100 Hz   
                                            /// 3 - 50 Hz   
                                            /// 4 - 25 Hz   
                                            /// 5 - 20 Hz   
                                            /// 6 - 10 Hz   
                                            /// 7 - 5 Hz   
                                            /// 8 - 4 Hz   
                                            /// 9 - 2 Hz   

    int64_t           spiOrientation;       /// orientation for SPI mode
                                               //0x0000	+Ux	+Uy	+Uz
	                                           //0x0009	-Ux	-Uy	+Uz
	                                           //0x0023	-Uy	+Ux	+Uz
	                                           //0x002A	+Uy	-Ux	+Uz
	                                           //0x0041	-Ux	+Uy	-Uz
	                                           //0x0048	+Ux	-Uy	-Uz
	                                           //0x0062	+Uy	+Ux	-Uz
	                                           //0x006B	-Uy	-Ux	-Uz
	                                           //0x0085	-Uz	+Uy	+Ux
	                                           //0x008C	+Uz	-Uy	+Ux
	                                           //0x0092	+Uy	+Uz	+Ux
	                                           //0x009B	-Uy	-Uz	+Ux
	                                           //0x00C4	+Uz	+Uy	-Ux
	                                           //0x00CD	-Uz	-Uy	-Ux
	                                           //0x00D3	-Uy	+Uz	-Ux
	                                           //0x00DA	+Uy	-Uz	-Ux
	                                           //0x0111	-Ux	+Uz	+Uy
	                                           //0x0118	+Ux	-Uz	+Uy
	                                           //0x0124	+Uz	+Ux	+Uy
	                                           //0x012D	-Uz	-Ux	+Uy
	                                           //0x0150	+Ux	+Uz	-Uy
	                                           //0x0159	-Ux	-Uz	-Uy
	                                           //0x0165	-Uz	+Ux	-Uy
	                                           //0x016C	+Uz	-Ux	-Uy

    int64_t           spiAccelLpfType;        /// built-in lpf filter cutoff frequency selection for accelerometers   
    int64_t           spiGyroLpfType;         /// built-in lpf filter cutoff frequency selection for rate sensors   
                                              /// Options are:
                                              //  UNFILTERED          = 0x00,
                                              //  FIR_40HZ_LPF        = 0x03,  // Bartlett LPF 40HZ
                                              //  FIR_20HZ_LPF        = 0x04,  // Bartlett LPF 20HZ
                                              //  FIR_10HZ_LPF        = 0x05,  // Bartlett LPF 10HZ
                                              //  FIR_05HZ_LPF        = 0x06,  // Bartlett LPF 5HZ
                                              //  IIR_50HZ_LPF        = 0x30, // Butterworth LPF 50HZ
                                              //  IIR_20HZ_LPF        = 0x40, // Butterworth LPF 20HZ
                                              // IIR_10HZ_LPF         = 0x50, // Butterworth LPF 10HZ
                                              //  IIR_05HZ_LPF        = 0x60, // Butterworth LPF 5HZ
                                              //  IIR_02HZ_LPF        = 0x70, // Butterworth LPF 2HZ
                                              //  IIR_25HZ_LPF        = 0x80, // Butterworth LPF 25HZ
                                              //  IIR_40HZ_LPF        = 0x90, // Butterworth LPF 40HZ
    uint64_t          extSyncFreq;            /// external sync frequency

} UserConfigurationStruct;

typedef enum {
//*****************************************************************************************
// add system parameters here and reassign USER_LAST_SYSTEM_PARAM (DO NOT CHANGE THIS!!!)
    USER_CRC                       = 0,   // 0
    USER_DATA_SIZE                    ,   // 1
    USER_USER_BAUD_RATE               ,   // 2  order of next 4 parameters
    USER_USER_PACKET_TYPE             ,   // 3  of required unit output bandwidth
    USER_USER_PACKET_RATE             ,   // 4 
    USER_LPF_ACCEL_TYPE               ,   // 5  prefered LPF filter type for accelerometer
    USER_LPF_RATE_TYPE                ,   // 6  prefered LPF filter type for rate sensor
    USER_ORIENTATION                  ,   // 7  unit orientation
    USER_LAST_SYSTEM_PARAM = USER_ORIENTATION,  // 7
//*****************************************************************************************
// add parameter enumerator here while adding new parameter in user UserConfigurationStruct
    USER_GPS_BAUD_RATE                ,   // 8
    USER_GPS_PROTOCOL                 ,   // 9 
    USER_HARD_IRON_X                  ,   // 10
    USER_HARD_IRON_Y                  ,   // 11
    USER_SOFT_IRON_RATIO              ,   // 12
    USER_SOFT_IRON_ANGLE              ,   // 13
    USER_LAST_UART_PARAM              ,   // 14
    USER_APPLICATION_BEHAVIOR         ,
// spi-specific parameters
    USER_SPI_SYNC_RATE                ,   // SPI data ready rate
    USER_SPI_ORIENTATION              ,   // SPI mode orientation
    USER_SPI_ACCEl_LPF                ,   // SPI mode accel lpf
    USER_SPI_RATE_LPF                 ,   // SPI mode gyro  lpf
    USER_EXT_SYNC_FREQ                ,   // Extern sync frequency applied tp SYNC/1PPS input
    USER_MAX_PARAM
} UserConfigParamNumber;

#define MAX_SYSTEM_PARAM USER_ORIENTATION

extern int userPacketOut;

#define INVALID_PARAM           -1
#define INVALID_VALUE           -2
#define INVALID_PAYLOAD_SIZE    -3
#define APP_BEHAVIOR_USE_EXT_SYNC  0x0000000000000001LL



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
extern BOOL      ExtSyncEnabled();
extern int       ExtSyncFrequency();

uint16_t         SpiOrientation();
uint8_t          SpiSyncRate();
uint8_t          SpiAccelLpfType();
uint8_t          SpiGyroLpfType();

#endif /* USER_CONFIGURATION_H */


