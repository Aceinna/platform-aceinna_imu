/** ******************************************************************************
 * @file configurationAPI.h API functions for Interfacing with unit configurationb parameters
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
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


#ifndef _CONFIG_API_H
#define _GONFIG_API_H
#include <stdint.h>
#include "GlobalConstants.h"

// serial port related functions
int      configGetPacketRate(void);
int      configGetBaudRate(void);
int      configGetPacketRateDivider(int configParam);
BOOL     configSetPacketRate(int rate, BOOL fApply);
BOOL     configSetBaudRate(int baudRate, BOOL fApply);
BOOL     configSetOutputPacketCode(uint16_t code, BOOL fApply);

// IMU related functions
BOOL     configSetUserOrientation(uint16_t *input, BOOL fApply);
uint16_t configGetOrientation(void);
int      configGetAccelLfpFreq();
int      configGetRateLfpFreq();
uint16_t configGetPrefilterFreq();
BOOL     configSelectUserLPFilter(int sensor, int cutoffFreq, BOOL fApply);
int      configApplyOrientation(uint16_t orientation);
void     configSetUsedSensors(int idx, uint8_t mask);
uint16_t configGetUsedChips(void);
uint16_t configGetActiveChips(void);
uint16_t configGetUsedSensors(int chipIdx);
void     configSetUsedSensors(int idx, uint8_t mask);
uint16_t configGetParam(int idx);

// GPS related functions
void     configSetGpsBaudRate(int16_t rate); 
int      configGetGpsBaudRate(void);
int      configGetGpsProtocol(void);
void     configSetGpsProtocol(int protocol);
int      configGetNextGpsBaudRate(int baudRate);

// SPI bus related functions
uint16_t configGetSensorFilterTypeForSPI();
void     configSetSensorFilterTypeForSPI(uint16_t type);

// CAN bus related parameters
BOOL     configSaveEcuAddress(uint16_t* address);
uint16_t configGetCANBaudRate(); 
uint16_t configGetCANPacketRate();
uint16_t configGetCANPacketsToTransmit();
void     configSetCANPacketRate(int rate);
BOOL     configIsCanBaudRateDetectEnabled();
BOOL     configIsCanTermResistorEnabled();
void     configSetPacketRateDividorForSPI(uint16_t dvd);
uint16_t configGetPacketRateDividorForSPI();
void     ApplyFactoryConfiguration();
extern BOOL  SaveAppFlag(void);
#endif
