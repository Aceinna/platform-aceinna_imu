/*******************************************************************************
 * File:   ecu_settings.h
 * Created on JAn 25, 2017
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

#ifndef ECU_SETTINGS_H
#define ECU_SETTINGS_H

#ifdef __cplusplus
 extern "C" {
#endif
 


#include <stdint.h>

#include "GlobalConstants.h"
#include "aceinna_sae_J1939.h"
#include "UserConfigurationUart.h"

/// User defined configuration strucrture
///Please notice, that parameters are 64 bit to accomodate float64_t types as well as longer string types
#pragma pack(1)
typedef struct {
    uint64_t           dataCRC;             // CRC of user configuration structure CRC-16
    uint64_t           dataSize;            // Size of the user configuration structure 
    userUartConfig_t   uartCfg;             // user UART configuration structure
    uint16_t           ecuAddress;          // ecu address
    uint16_t           ecuBaudRate;         // ecu baudrate - 250, 500, 1000
    uint16_t           ecuPacketRate;       // CAN packet rate (100Hz tick)
    uint16_t           ecuFilterFreqAccel;  // accels cutoff frequency for digital filter
    uint16_t           ecuFilterFreqRate;   // rates  cutoff frequency for digital filter
    uint16_t           ecuPacketType;       // packet types being transmitted
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
    uint16_t          userBehavior;        // user algorithm behaviour  
    uint8_t           statusPs;            // ps value of status message
    uint8_t           algResetPs;          // ps value of alg reset

    uint8_t           saveCfgPs;           // ps value of config save
    uint8_t           packetRatePs;        // ps value of packet type
    uint8_t           packetTypePs;        // ps value of packet type
    uint8_t           filterPs;            // ps value of lpf
    uint8_t           orientationPs;       // ps value of orientation
    uint8_t           userBehvPs;          // ps value of user behavior
    uint8_t           masterStatusPs;      // ps value of status
    uint8_t           hwStatusPs;          // ps value of status
    uint8_t           swStatusPs;          // ps value of status
    uint8_t           algoCtrlPs;          // ps value of algorithm control
    uint8_t           hrRatePs;            // ps value of hr rate message
    uint8_t           hrAccelPs;           // ps value of hr accelrate message
    uint16_t          algoCoefOfReduceQ;
    uint16_t          algoLinAccelSwitchDelay;
    uint16_t          algoRateIntegrationTime;
    uint8_t           addressChanged;      // ps value of hr accelrate message
// Odometer lever arm
    int16_t           odoLeverArmX;
    int16_t           odoLeverArmY;
    int16_t           odoLeverArmZ;
// Aiding signal configuration settings
    uint8_t           signalSource;  
    uint8_t           aidingPF;  
    uint8_t           aidingPS;  
    uint8_t           aidingMsgRate;  
    uint8_t           drivingDirPF;  
    uint8_t           drivingDirPS;  
    uint8_t           odoCfgSwitch;  
// DM1 message configurable parameters
    uint8_t           lamp_status;          // status of warning indicators
    uint8_t           flash_status;         // flash control of warning indicators
    uint32_t          SPN;                  // SPN of message DM1
    uint8_t           FMI1;                 // Failure Mode Identifier for DTC1
    uint8_t           FMI2;                 // Failure Mode Identifier for DTC2
// new configurable ps
    uint8_t           aidLvarmPs;
    uint8_t           aidConfigPs;
    uint8_t           dm1ConfigPs;
//
    uint8_t           ariMsgPriority;         // priority of ARI  message
    uint8_t           accsMsgPriority;        // priority of ACCS message
    uint8_t           ssi2MsgPriority;        // priority of SSI2 message

} ecu_settings_struct;
#pragma pack()

enum {
  INVALID_PARAM         =  -1,
  INVALID_VALUE         =  -2,
  INVALID_PAYLOAD_SIZE  =  -3,
};

enum {
  USER_BEHAVIOR_RESTART_ON_OVERRANGE     =      0x0001,
  USER_BEHAVIOR_ENABLE_DYN_MOTION        =      0x0002,
  USER_BEHAVIOR_SEND_RAW_RATES           =      0x0004,
  USER_BEHAVIOR_SWAP_PITCH_AND_ROLL      =      0x0008,
  USER_BEHAVIOR_ENABLE_AUTO_BAUD         =      0x0010,
  USER_BEHAVIOR_ENABLE_TERM_RESISTOR     =      0x0020,
  USER_BEHAVIOR_SWAP_ACCEL_FRAME         =      0x0040,
  USER_BEHAVIOR_USE_RAW_ACCEL_FOR_ALG    =      0x0080,
  USER_BEHAVIOR_USE_RAW_RATE_TO_PREDICT_ACCEL = 0x0100,
  USER_BEHAVIOR_SWAP_BYTES_IN_REQUEST    =      0x0200,
  USER_BEHAVIOR_RUN_ALGORITHM_MASK       =      0x8000,
};

/**
 * @brief 
 * 
 * @param fDefault 
 * @return BOOL 
 */
extern BOOL      SaveEcuSettings(BOOL fDefault, BOOL fromUart);

/**
 * @brief 
 * 
 */
extern void      ApplyEcuSettings();

/**
 * @brief 
 * 
 */
extern void      LoadEcuSettings(void);

/**
 * @brief 
 * 
 * @return int32_t 
 */
extern int32_t   GetCANPacketRateDivider();

/**
 * @brief Get the Ecu Address object
 * 
 * @return uint8_t 
 */
extern uint8_t   GetEcuAddress();

/**
 * @brief Get the Ecu Baud Rate object
 * 
 * @return int32_t 
 */
extern int32_t   GetEcuBaudRate();

/**
 * @brief 
 * 
 * @return BOOL 
 */
extern BOOL      CanBaudRateDetectionEnabled();

/**
 * @brief 
 * 
 * @return BOOL 
 */
extern BOOL      UseAlgorithm();

/**
 * @brief 
 * 
 * @return BOOL 
 */
extern BOOL      SendRawRates();

/**
 * @brief 
 * 
 * @return BOOL 
 */
extern BOOL      SwapPitchAndRoll();

/**
 * @brief 
 * 
 * @return BOOL 
 */
extern BOOL      SwapAccelFrame();      // NED to NWU

/**
 * @brief 
 * 
 * @return BOOL 
 */
extern BOOL      SwapBytesInRequest();  // LSB with MSB

/**
 * @brief Set the Ecu Baudrate object
 * 
 * @param baudrate 
 */
extern void      SetEcuBaudrate(uint16_t baudrate);

/**
 * @brief Set the Ecu Address object
 * 
 * @param address 
 */
extern void      SetEcuAddress(uint16_t address);


/*******************************************
 * @brief Set the Ecu Behavior object
 * 
 * @param behavior ==
********************************************/
void SetEcuBehavior(uint16_t const behavior);


/*******************************************
 * @brief Set the Ecu Packet Type object
 * 
 * @param type ==
********************************************/
void SetEcuPacketType(uint16_t const type);

/*******************************************
 * @brief Set the Ecu Packet Rate Divider object
 * 
 * @param rate ==
 * @return BOOL 
********************************************/
BOOL SetEcuPacketRate(uint16_t const rate);


/*******************************************
 * @brief Set the Ecu Orientation object
 * 
 * @param orient ==
********************************************/
void SetEcuOrientation(uint16_t const orient);


/*******************************************
 * @brief Set the Ecu Accel Filter object
 * 
 * @param filter ==
********************************************/
void SetEcuAccelFilter(uint16_t const filter);


/*******************************************
 * @brief Set the Ecu Rate Filter object
 * 
 * @param filter ==
********************************************/
void SetEcuRateFilter(uint16_t const filter);


/*******************************************
 * @brief Set the User Uart Baud Rate object
 * 
 * @param baudRate ==
********************************************/
void SetUserUartBaudRate(uint16_t const baudRate);


/*******************************************
 * @brief 
 * 
********************************************/
extern  void UpdateEcuSettings();

/*******************************************
 * @brief 
 * 
 * @param baudrate ==
********************************************/
extern  void UpdateEcuBaudrate(uint16_t baudrate);


/*******************************************
 * @brief 
 * 
 * @param ecuAddress ==
 * @return BOOL 
********************************************/
BOOL  SaveEcuAddress(uint8_t ecuAddress);


extern int32_t   ConfigSaveError;

#ifdef __cplusplus
}
#endif

#endif /* USER_CONFIGURATION_H */


