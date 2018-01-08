/** ***************************************************************************
 * @file gps.h GPS Driver for Inertial/GPS NAV.
 * @author Dong An
 * @date   2009-04-10 23:20:59Z
 * @ver 8719
 * @copyright (c) 2013, 2014 All Rights Reserved.
 * @section LICENSE
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * @brief This is a generalized GPS interface, taken loosely from the DMU440
 * project possibly implemented using the Origin ORG4475 GPS module (or NovAtel
 * or uBlox) the GPS may communicated via SPI or UART, that is passed in on init
 *  03.2007 DA  Cleaned up, Doxygenized, and finalized for NAV440 release.
 *****************************************************************************/
#ifndef GPS_H
#define GPS_H

#include <stdint.h>

void initGPSHandler(void); /// note: should pass in SPI or UART
void GPSHandler(void);
extern void TaskGps(void);

// FIXME ECW: implement GpsWhoAmI and GpsSelfTest
uint8_t  GpsWhoAmI(uint32_t *whoami); /// returns true if value is as expected
uint8_t  GpsSelfTest();

// debugging stream out the debug port
#define GPS_NO_STREAM         0
#define GPS_NMEA_DEBUG_STREAM 1

#ifndef STREAM_GPS
//#define STREAM_GPS GPS_NO_STREAM
#define STREAM_GPS GPS_NMEA_DEBUG_STREAM
#endif


#endif /* GPS_H */
