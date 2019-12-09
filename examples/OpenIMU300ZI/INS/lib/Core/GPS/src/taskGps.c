/** ***************************************************************************
 * @file taskGps.c handle GPS data, make sure the GPS handling function gets
 *  called on a regular basis
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
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
#include <stddef.h>

#include "driverGPS.h"
#include "uart.h"
#include "gps.h"
#include "osapi.h"
#include "platformAPI.h"
#include "gpsAPI.h"

int gpsIdleCnt = 0;

/** ****************************************************************************
 * @name TaskGps
 * @brief task callback with the main loop for handle GPS data, make sure the
 *        GPS handling function gets called on a regular basis;.
 * gCalibration.productConfiguration.bit.hasGps = 1; by setting:
 * <hasGps>true</hasGps> and <useGps>true</useGps> in name_IMU380.xml file
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void TaskGps(void const *argument)
{
    int    bytesAvailable;
    static uint32_t updateHDOP, pollSirfCnt;

    while(gpsSerialChan == UART_CHANNEL_NONE) {
        // nothing to do untill port decided
            OS_Delay( 1000);
        }

    // start out with the DOP high
    gGpsDataPtr->HDOP = 50.0;

    initGPSHandler();
    GPSHandler();

    while (1) 
    {

        // if (gpsUsedInAlgorithm())   // useGPS   so far only external GPS
        if (1)   // useGPS   so far only external GPS
        {
           	bytesAvailable = gpsBytesAvailable();
            if(!bytesAvailable)
            {
                gpsIdleCnt++;
                OS_Delay(20);
                continue;
            }
            GPSHandler();
            uart_BIT(GPS_SERIAL_PORT);  // 
            // or signal other tasks based on these params?

            if(gGpsDataPtr->GPSProtocol == SIRF_BINARY)
            {
                if (((getSystemTime() / 1000) - updateHDOP) > 600)
                {
                gGpsDataPtr->HDOP = 50.0;
                updateHDOP = (getSystemTime() / 1000);
                pollSiRFVersionMsg();
                pollSirfCnt++;
                }
            }
        }
    }
}

#endif // GPS