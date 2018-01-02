/** ***************************************************************************
 * @file taskGps.c handle GPS data, make sure the GPS handling function gets
 *  called on a regular basis
 * @author Dong An
 * @date   2009-04-10 23:20:59Z
 * @copyright (c) 2013, 2014 All Rights Reserved.
 * @section LICENSE
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *****************************************************************************/
#include <stdint.h>
#include <stddef.h>

#include "salvodefs.h"
#include "dmu.h"
#include "driverGPS.h"
#include "taskUserCommunication.h" // getUserCommunicationType
#include "uart.h"
#include "watchdog.h"
#include "debug.h"
#include "debug_usart.h"
#include "configureGPIO.h" // IO3 (B11) debug timing pin
#include "extern_port_config.h"
#include "timer.h"
#include "xbowsp_init.h"

#include "timer.h"    // for TimeNow()

// these don't make the loop run at the listed rate, they are set up for
// messages at the listed rate
#define GPS_LOOP_PERIOD_ONE_HZ (OS_TICKS_PER_SECOND / 2)
#define GPS_LOOP_PERIOD_FIVE_HZ (OS_TICKS_PER_SECOND / 10)

/** ****************************************************************************
 * @name TaskGps
 * @brief task callback with the main loop for handle GPS data, make sure the
 *        GPS handling function gets called on a regular basis;.
 * gCalibration.productConfiguration.bit.hasGps = 1; by setting:
 * <hasGps>true</hasGps> and <useGps>true</useGps> in name_IMU380.xml file
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void TaskGps(void)
{
    static uint32_t updateHDOP, pollSirfCnt;

    // UART_COMM is 0
    if (getUserCommunicationType() != UART_COMM) { // UART / SPI
        // you can't exit the task using a return so this continuously releases
        // cpu to the other tasks.
        while(1) {
            OS_Delay( GPS_LOOP_PERIOD_ONE_HZ );
        }
    }

    // start out with the DOP high
    gGpsDataPtr->HDOP = 50.0;

    /// initial internal GPS settings, uses settings to configure then switch
    //  to BINARY and faster baud
    if ( IsInternalGPS() == true)  {
        SetConfigurationProtocolGPS(NMEA_TEXT);
        SetConfigurationBaudRateGps(BAUD_4800);
    }

    initGPSHandler();
    GPSHandler();

    while (1) {
        // could wait on a message from the data acquistion
        OS_Delay( GPS_LOOP_PERIOD_FIVE_HZ );

        if ( (IsInternalGPS()  == true) ||  // hasGPS
             (UseExtUSARTGps() == true) )   // useGPS
        {
            GPSHandler();
            uart_BIT(GPS_UART);  // kUserB_UART
            // or signal other tasks based on these params?

            // Set the GPS validity, based on the Horizontal Dilution of Precision
            if (gGpsDataPtr->gpsValid && gGpsDataPtr->HDOP > 15.0) {
                gGpsDataPtr->gpsValid = false;
            } else if (gGpsDataPtr->HDOP < 10.0) {
                gGpsDataPtr->gpsValid = true;
            }

            if (((TimeNow() / 1000) - updateHDOP) > 600) {
                gGpsDataPtr->HDOP = 50.0;
                updateHDOP = (TimeNow() / 1000);
                pollSiRFVersionMsg();
                pollSirfCnt++;
            }
        }
    }
}
