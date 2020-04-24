/** ***************************************************************************
 * @file driverGPSAllentrance.c GPS Driver for Internal/GPS NAV.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 * @details
 *  This module provides high level GPS management,
 *  based on product type and GPS receiver. This module
 *  is interfaced with NAV processing and other specific GPS
 *  process files. The functions are provided in this files are common
 *  for all GPS protocol. Protocol-specific process is provided in other GPS files
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
#ifdef GPS


#include <stdint.h>

#include "GpsData.h"
#include "gpsAPI.h"

float getGpsHdop() {return gGpsDataPtr->HDOP;}


BOOL  SetGpsBaudRate(int rate, int fApply)
{
    switch (rate)
    {
        case 4800:
        case 9600:
        case 19200:
        case 38400:
        case 57600:
        case 115200:
        case 230400:
            break;
        default:
            return FALSE;
    }
    if(fApply)
    {
        gGpsDataPtr->GPSbaudRate = rate;
    }

    return TRUE;
}

BOOL  SetGpsProtocol(int protocol, int fApply)
{
    switch(protocol)
    {
        case NMEA_TEXT:
        case NOVATEL_BINARY:
        case UBLOX_BINARY:
            break;
        default:
            return FALSE;
    }
    if(fApply)
    {
        gGpsDataPtr->GPSProtocol = protocol;
    }

    return TRUE;
}

#endif // GPS
