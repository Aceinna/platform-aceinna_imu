/** ******************************************************************************
 * @file platformAPI.h API functions for Interfacing with xbow library funcctions
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


#ifndef _PLATFORM_API_H
#define _PLATFORM_API_H
#include <stdint.h>
#include "GlobalConstants.h"

// NEEDS TO BE CHECKED
BOOL eepromLocked(void);
BOOL lockEeprom(void);
BOOL unlockEeprom(void);
BOOL userApplicationActive(void);
BOOL platformHasMag();
int  platformGetSysRange();
int  platformGetUnitCommunicationType(void);
BOOL platformSetUnitCommunicationType(int type);
void platformEnableMag(BOOL enable);
extern char *unitVersionString(void);
extern uint32_t unitSerialNumber(void);
extern uint8_t  unitSpiSwVersion(void);
extern char *platformBuildInfo(void);
void            platormReset();
BOOL platformCanBaudRateDetectEnabled();
BOOL platformCanTermResistorEnabled();
void markEEPROMUnlocked();
void markEEPROMLocked();
BOOL readFlash(uint32_t addr, uint8_t *buf, uint16_t len);
void platformUnassignSerialChannels();
BOOL platformAssignPortTypeToSerialChannel(int portType, int channel);
int  platformGetSerialChannel(int portType);
void platformRegisterRxSerialSemaphoreID(int portType, void *semId);
void platformGetVersionBytes(uint8_t *bytes);
BOOL platformApplyOrientation(uint16_t input);
int  platformGetSensToPpsDelay();
int  platformGetPpsToDrdyDelay();
uint64_t platformGetCurrTimeStamp();
uint64_t platformGetDacqTimeStamp(); 
void     platformPerformSelfTest();
void     platformSetDacqTimeStamp(uint64_t tstamp); 
int      platformEnableExtSync(BOOL enable);
void     platformUpdateInterfaceTestStatus(BOOL fGood);
void     platformSetMode(BOOL isBoot);
BOOL     platformIsInBootMode();
BOOL platformIsGpsPPSUsed(void);
void platformEnableGpsPps(BOOL enable);

#define   kick_dog()

#define UART_CHANNEL_NONE -1  // undefined channel
#define UART_CHANNEL_0     0  // pins 3 and 4 on the 20-pin connector. NOTE: not available in SPI interface mode
#define UART_CHANNEL_1     1  // pins 5 and 6 on the 20-pin connector. NOTE: not available in SPI interface mode 
#define UART_CHANNEL_2     2  // pins 17 and 19 on the 20-pin connector. Always available

#endif
