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
#include "filter.h"


/// User defined configuration strucrture
///Please notice, that parameters are 64 bit to accomodate double types as well as longer string types

typedef struct {
    uint64_t           dataCRC;             /// CRC of user configuration structure CRC-16
    uint64_t           dataSize;            /// Size of the user configuration structure 
    
    int64_t            uartBaudRate;        /// baudrate of user UART, bps. 
                                            /// valid options are:
                                            /// 4800
                                            /// 9600
                                            /// 19200
                                            /// 38400
                                            /// 57600
                                            /// 115200
                                            /// 230400
                                            /// 460800
    uint8_t            uartPacketType[8];   /// User packet to be continiously sent by unit
                                            /// Packet types defined in structure UserOutPacketType
                                            /// in file UserMessaging.h
                                            
    int64_t            uartPacketRate;      /// Packet rate for continiously output packet, Hz.
                                            /// Valid settings are: 0 ,2, 5, 10, 20, 25, 50, 100, 200 

    int64_t            uartLpfAccelFilterFreq;  /// built-in lpf filter cutoff frequency selection for accelerometers   
    int64_t            uartLpfRateFilterFreq;   /// built-in lpf filter cutoff frequency selection for rate sensors   
                                            /// Options are:
                                            /// 0  -  Filter turned off
                                            /// 50 -  Butterworth LPF 50HZ
                                            /// 20 -  Butterworth LPF 20HZ
                                            /// 10 -  Butterworth LPF 10HZ
                                            /// 05 -  Butterworth LPF 5HZ
                                            /// 02 -  Butterworth LPF 2HZ
                                            /// 25 -  Butterworth LPF 25HZ
                                            /// 40 -  Butterworth LPF 40HZ
    
    uint8_t           uartOrientation[8];   /// unit orientation in format 0x0000000000ddrrff
                                            /// where   dd - down axis, rr - right axis, ff - forward axis
                                            /// next axis values a valid :  
                                            /// 'X' (0x58) -> plus  X, 'Y' (0x59) -> plus Y,  'Z' (0x5a) -> plus Z
                                            /// 'x' (0x78) -> minus X, 'y' (0x79) -> minus Y, 'z' (0x7a) ->minusZ

    //***************************************************************************************
    // here is the border between arbitrary parameters and platform configuration parameters
    //***************************************************************************************
    int64_t           uartGpsBaudRate;      /// baudrate of GPS UART, bps. 
                                            /// valid options are:
                                            /// 4800
                                            /// 9600
                                            /// 19200
                                            /// 38400
                                            /// 57600
                                            /// 115200
                                            /// 230400
    int64_t           uartGpsProtocol;      /// protocol of GPS receicer. 
                                            /// so far valid options are:
                                            /// NMEA_TEXT
                                            /// NOVATEL_BINARY
    double            uartHardIron_X;
    double            uartHardIron_Y;
    double            uartSoftIron_Ratio;
    double            uartSoftIron_Angle;

    // place new arbitrary configuration parameters here
    // parameter size should even to 8 bytes
    // Add parameter offset in UserConfigParamOffset structure if validation or
    // special processing required 

} UserConfigurationUartStruct;

/// User defined configuration strucrture
///Please notice, that parameters are 64 bit to accomodate double types as well as longer string types
typedef struct {
    uint64_t           dataCRC;             // CRC of user configuration structure CRC-16
    uint64_t           dataSize;            // Size of the user configuration structure 

    uint16_t           ecuAddress;          // ecu address
    uint16_t           ecuBaudRate;         // ecu baudrate - 250, 500, 1000
    uint16_t           ecuPacketRate;       // divider for 200Hz tick
    uint16_t           ecuFilterFreqAccel;  // accels cutoff frequency for digital filter
    uint16_t           ecuFilterFreqRate;   // rates  cutoff frequency for digital filter
    uint16_t           ecuPacketType;       // packet types being transmitted
    uint16_t           canTermResistorEnabled;    // enable/disable CAN termination resistor
    uint16_t           canBaudRateDetectEnabled;  // enable/disable CAN baud rate auto detection
    uint16_t           ecuOrientation;      // unit orientation
                                            //0  +Z +Y +X
                                            //9  +Z -Y -X 
                                            // 35  +Z +X +Y
                                            // 42  +Z -X +Y
                                            // 65  -Z +Y -X 
                                            // 72  -Z -Y +X
                                            // 98  -Z +X +Y 
                                            // 107 -Z -X -Y
                                            // 133 +X +Y -Z
                                            // 140 +X -Y +Z
                                            // 146 +X +Z +Y
                                            // 155 +X -Z -Y
                                            // 196 -X +Y +Z
                                            // 205 -X -Y -Z
                                            // 211 -X +Z -Y
                                            // 218 -X -Z +Y
                                            // 273 +Y +Z -X
                                            // 280 +Y -Z +X
                                            // 292 +Y +X +Z
                                            // 301 +Y -X -Z
                                            // 336 -Y +Z +X
                                            // 345 -Y -Z -X
                                            // 357 -Y +X -Z
                                            // 364 -Y -X +Z
    uint16_t           userBehavior;        // user algorithm behaviour  

    uint16_t           statusPs;            // ps value of status message
    uint16_t           algResetPs;          // ps value of alg reset
    uint16_t           saveCfgPs;           // ps value of config save
    uint16_t           packetRatePs;        // ps value of packet type
    uint16_t           packetTypePs;        // ps value of packet type
    uint16_t           filterPs;            // ps value of lpf
    uint16_t           orientationPs;       // ps value of orientation
    uint16_t           userBehvPs;          // ps value of user behavior
    uint16_t           magAlignPs;          // ps value of mag alignment command
    float              hardIron_X;
    float              hardIron_Y;
    float              softIron_Ratio;
    float              softIron_Angle;

    UserConfigurationUartStruct uartConfig;

} UserConfigurationStruct;

extern UserConfigurationUartStruct* pUserUartConfig;

enum{
//*****************************************************************************************
// These parateters are not saved into eeprom as of yet
    USER_UART_CRC                  = 0,   // 0
    USER_UART_CONFIG_SIZE             ,   // 1
    USER_UART_BAUD_RATE               ,   // 2 
    USER_UART_PACKET_TYPE             ,   // 3 
    USER_UART_PACKET_RATE             ,   // 4 
    USER_UART_LPF_ACCEL_TYPE          ,   // 5  prefered LPF filter type for accelerometer
    USER_UART_LPF_RATE_TYPE           ,   // 6  prefered LPF filter type for rate sensor
    USER_UART_ORIENTATION             ,   // 7  unit orientation
//*****************************************************************************************
    USER_GPS_BAUD_RATE                ,   // 8
    USER_GPS_PROTOCOL                 ,   // 9
    USER_HARD_IRON_X                  ,   // 10 
    USER_HARD_IRON_Y                  ,   // 11
    USER_SOFT_IRON_RATIO              ,   // 12
    USER_SOFT_IRON_ANGLE              ,   // 13
    USER_UART_MAX_PARAM                   // 14   
};

#define MAX_SYSTEM_PARAM USER_ORIENTATION

extern int userPacketOut;

#define USER_OK      0x00
#define USER_NAK     0x80
#define USER_INVALID 0x81

#define INVALID_PARAM           -1
#define INVALID_VALUE           -2
#define INVALID_PAYLOAD_SIZE    -3

#define USER_BEHAVIOR_RUN_ALGORITHM_MASK    0x8000
#define USER_BEHAVIOR_USE_MAGS_MASK         0x4000
#define USER_BEHAVIOR_SWAP_PITCH_ROLL       0x0008
#define USER_BEHAVIOR_USE_AUTOBAUD          0x0010
#define USER_BEHAVIOR_NWU_FRAME             0x0040
#define USER_BEHAVIOR_SWAP_REQUEST_PGN      0x0200
// define additional algorithm flavours here

extern UserConfigurationStruct      gUserConfiguration;

extern void      InitializeUserAlgorithmParams(void);
extern BOOL      validateUserConfigInEeprom(int *numParams);
extern uint32_t  getUserParamFromEeprom(uint32_t offset);
extern BOOL      saveUserConfigInEeprom(uint8_t *ptrToUserConfigStruct, int userConfigStructLen);
extern BOOL      checkIfUserEEPROMErased(void);
extern BOOL      loadUserConfigFromEeprom(uint8_t *ptrToUserConfigInRam, int *userConfigSize);
extern BOOL      SaveUserConfig(void);
extern BOOL      LoadDefaultUserConfig(BOOL fSave);
extern void      ApplyEcuSettings(void* pEcuConfig);
extern void      ApplyEcuControlSettings(void *pConfig);
extern void      ApplySystemParameters(void* pEcuConfig);
extern void      userInitConfigureUart();
extern void      CopyUserUartConfig(UserConfigurationUartStruct *ext, BOOL toLocal);
extern void      UpdateEcuAccelFilterSettings(uint16_t data);
extern void      UpdateEcuRateFilterSettings(uint16_t data);
extern void      UpdateEcuOrientationSettings(uint16_t data);
extern void      UpdateUARTAccelFilterSettings(uint16_t data);
extern void      UpdateUARTRateFilterSettings(uint16_t data);
extern void      UpdateUARTOrientationSettings(uint16_t data);
extern void      UpdateEcuUartBaudrate(uint64_t data);
extern void      UpdateEcuUartPacketType(uint64_t data);
extern void      UpdateEcuUartPacketRate(uint64_t data);
extern void      LoadEcuBankSettings(void *pConfig);
extern BOOL      ApplyLegacyConfigParameters();
extern BOOL      SwapPitchAndroll();
extern BOOL      UseNWUFrame();
extern BOOL      SwapRequestPGN();
extern BOOL      OrientationToAscii(uint8_t *asciiOrien, uint16_t hexOrien);
extern BOOL      UseAutoBaud();



#endif /* USER_CONFIGURATION_H */


