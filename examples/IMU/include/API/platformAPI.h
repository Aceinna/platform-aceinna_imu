/** ******************************************************************************
 * @file xbowAPI.h API functions for Interfacing with xbow library funcctions
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


#ifndef _XBOW_API_H
#define _XBOW_API_H
#include <stdint.h>
#include "GlobalConstants.h"

// NEEDS TO BE CHECKED
BOOL eepromLocked(void);
BOOL lockEeprom(void);
BOOL unlockEeprom(void);
void markConfigAsInvalid(void);
BOOL saveEcuAddress(uint16_t* address);
BOOL platformSetPacketRate(int rate, BOOL fApply);
BOOL platformSetBaudRate(int baudRate, BOOL fApply);
BOOL platformSetOrientation(uint16_t *input, BOOL fApply);
int  platformGetPacketRateDivider(int configParam);
int  platformGetOrientation(void);
int  platformGetPacketRate(void);
int  platformGetBaudRate(void);
int  platformGetAccelLfpFreq();
int  platformGetRateLfpFreq();
BOOL appStartedFirstTime(void);
BOOL platformSetOutputPacketCode(uint16_t code, BOOL fApply);
BOOL platformSelectLPFilter(int sensor, int cutoffFreq, BOOL fApply);
BOOL platformHasMag();
char *getBuildInfo();
int  platformGetSysRange();
BOOL platformUseGPS(void);
BOOL platformIsGpsPPSUsed(void);
void platformEnableGpsPps(BOOL enable);
int  getUnitCommunicationType(void);
void setUnitCommunicationType(int type);
void setGpsBaudRate(int16_t rate); 
void setGpsProtocol(int16_t p);   
void platformEnableMag(BOOL enable);
extern char *unitVersionString(void);
extern uint32_t unitSerialNumber(void);
extern char *platformBuildInfo(void);
extern uint32_t getDacqTime();
void   Reset();
void enableMagUsageInAlgorithm(BOOL enable);
void platformSetGpsBaudRate(int baudRate);
int  platformGetGpsBaudRate(void);
int platformGetGpsProtocol(void);
void platformSetGpsProtocol(int protocol);
int platformGetNextGpsBaudRate(int baudRate);
BOOL platformCanThermistorEnabled();
BOOL platformCanBaudRateDetectEnabled();
BOOL platformCanTermResistorEnabled();


/** ***************************************************************************
 * @name performSelfTest() Run self test
 * @brief verify existence of each sensor.
 * ">>swtest"  - data = 0x00
 *
 * @param [in] N/A
 * @retval N/A
 ******************************************************************************/
void performSelfTest();

// sets sensor error state for aalgorithm
extern void    SetSensorError(int sensor);

#endif
