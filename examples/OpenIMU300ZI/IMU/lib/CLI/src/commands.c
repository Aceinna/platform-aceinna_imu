/** ***************************************************************************
 * @file   commands.c callback functions from the commandTable
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *  Commands available to the commandLine.c shell
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


#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "commandLine.h"
#include "osapi.h"
#include "sensorsAPI.h"
#include "platformAPI.h"


#define LOGGING_LEVEL LEVEL_INFO
#include "debug.h"

char str[100];
// Display firmware version
void CmdVersion(uint32_t data)
{
    uint8_t *model = (uint8_t*)unitVersionString();
    uint8_t *rev   = (uint8_t*)platformBuildInfo();
    unsigned int serialNum  = unitSerialNumber();
    snprintf(str, sizeof(str), "%s %s SN:%u", model, rev, serialNum );

    DEBUG_STRING(str);
    DEBUG_ENDLINE();
} // End of display firmware version

/** ***************************************************************************
 * @name CmdReadAccelerometer() Read accelerometer
 * @brief  
 *
 * @retval N/A
 ******************************************************************************/
void CmdReadAccelerometer(uint32_t readInGs)
{
	double   readings[3];
 
    OSDisableHook();
    GetAccelData_mPerSecSq(readings);
    OSEnableHook();
    
    DEBUG_STRING("Reading accelerometer data\r\n");
    sprintf(str, " X = %3.5lf\r\n", readings[0]);
    DEBUG_STRING(str);
    sprintf(str, " Y = %3.5lf\r\n", readings[1]);
    DEBUG_STRING(str);
    sprintf(str, " Z = %3.5lf\r\n", readings[2]);
    DEBUG_STRING(str);

}

/** ***************************************************************************
 * @name CmdReadGyro() Read gyro data
 * @brief  
 *
 * @retval N/A
 ******************************************************************************/
void CmdReadGyro(uint32_t readInGs)
{
    double   readings[3];

    OSDisableHook();
    GetRateData_degPerSec(readings);
    OSEnableHook();
    
    DEBUG_STRING("Reading gyro data\r\n");
    sprintf(str, " X = %3.5lf\r\n", readings[0]);
    DEBUG_STRING(str);
    sprintf(str, " Y = %3.5lf\r\n", readings[1]);
    DEBUG_STRING(str);
    sprintf(str, " Z = %3.5lf\r\n", readings[2]);
    DEBUG_STRING(str);

}

/** ***************************************************************************
 * @name CmdReadMagnetometer. Read magnetometer data
 * @brief  
 *
 * @retval N/A
 ******************************************************************************/
void CmdReadMagnetometer(uint32_t readInGs)
{
    double   readings[3];

    OSDisableHook();
    GetMagData_G(readings);
    OSEnableHook();
    
    DEBUG_STRING("Reading magnetometer data\r\n");
    sprintf(str, " X = %3.5lf\r\n", readings[0]);
    DEBUG_STRING(str);
    sprintf(str, " Y = %3.5lf\r\n", readings[1]);
    DEBUG_STRING(str);
    sprintf(str, " Z = %3.5lf\r\n", readings[2]);
    DEBUG_STRING(str);

}


#include "commandTable.h"
