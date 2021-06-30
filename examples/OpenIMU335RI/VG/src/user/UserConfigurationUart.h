/*******************************************************************************
 * File:   UserConfigurationUart.h
 * Created on Jan 25, 2020
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

#ifndef USER_CONFIGURATION_UART_H
#define USER_CONFIGURATION_UART_H

#include <stdint.h>


/// User defined configuration strucrture
///Please notice, that parameters are 64 bit to accomodate double types as well as longer string types

typedef struct {
    int64_t            reserved[2];
    int64_t            userUartBaudRate;    /// baudrate of user UART, bps. 
                                            /// valid options are:
                                            /// 4800
                                            /// 9600
                                            /// 19200
                                            /// 38400
                                            /// 57600
                                            /// 115200
                                            /// 230400
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
    
    uint8_t           orientation[8];       


    //***************************************************************************************
    // here is the border between arbitrary parameters and platform configuration parameters
    //***************************************************************************************

    // place new arbitrary configuration parameters here
    // parameter size should even to 8 bytes
    // Add parameter offset in UserConfigParamOffset structure if validation or
    // special processing required 

} userUartConfig_t;

extern userUartConfig_t *pUserUartConfig;

enum{
//*****************************************************************************************
// These parateters are not saved into eeprom as of yet
    USER_UART_START                   ,   // 0 
    USER_UART_RSVD                    ,   // 1 
    USER_UART_BAUD_RATE               ,   // 2 
    USER_UART_PACKET_TYPE             ,   // 3 
    USER_UART_PACKET_RATE             ,   // 4 
    USER_UART_LPF_ACCEL_TYPE          ,   // 5  prefered LPF filter type for accelerometer
    USER_UART_LPF_RATE_TYPE           ,   // 6  prefered LPF filter type for rate sensor
    USER_UART_ORIENTATION             ,   // 7  unit orientation
    USER_UART_MAX_PARAM                   // 8   
};

extern int userPacketOut;

#define USER_OK      0x00
#define USER_NAK     0x80
#define USER_INVALID 0x81

#define FORWARD   0
#define RIGHT     1
#define DOWN      2


#define PLUS_X    0x582B
#define PLUS_Y    0x592B
#define PLUS_Z    0x5A2B
#define MINUS_X   0x582D
#define MINUS_Y   0x592D
#define MINUS_Z   0x5A2D


#define FWD_X_PLUS_MASK	   0x00000000
#define FWD_X_MINUS_MASK   0x00000001
#define FWD_Y_PLUS_MASK	   0x00000002
#define FWD_Y_MINUS_MASK   0x00000003
#define FWD_Z_PLUS_MASK	   0x00000004
#define FWD_Z_MINUS_MASK   0x00000005

#define RIGHT_X_PLUS_MASK  0x00000020
#define RIGHT_X_MINUS_MASK 0x00000028
#define RIGHT_Y_PLUS_MASK  0x00000000
#define RIGHT_Y_MINUS_MASK 0x00000008
#define RIGHT_Z_PLUS_MASK  0x00000010
#define RIGHT_Z_MINUS_MASK 0x00000018

#define DOWN_X_PLUS_MASK   0x00000080
#define DOWN_X_MINUS_MASK  0x000000C0
#define DOWN_Y_PLUS_MASK   0x00000100
#define DOWN_Y_MINUS_MASK  0x00000140
#define DOWN_Z_PLUS_MASK   0x00000000
#define DOWN_Z_MINUS_MASK  0x00000040

BOOL ApplyUserUartOrientation(uint16_t *input, BOOL fApply);
void GetCurrentUartSettings(userUartConfig_t *pSettings);
void UserInitConfigureUart();
void FillCurrentUartSettings(userUartConfig_t *pSettings);
void BackFillUartDataStructure();

#endif /* USER_CONFIGURATION_UART_H */


