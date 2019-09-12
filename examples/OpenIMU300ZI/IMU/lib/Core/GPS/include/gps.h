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


// debugging stream out the debug port
#define GPS_NO_STREAM         0
#define GPS_NMEA_DEBUG_STREAM 1

#ifndef STREAM_GPS
//#define STREAM_GPS GPS_NO_STREAM
#define STREAM_GPS GPS_NMEA_DEBUG_STREAM
#endif

#define NMEA_SYNC_1  0x00244750 // $GP
#define NMEA_SYNC_2  0x0024474E // $GN


#endif /* GPS_H */