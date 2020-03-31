/** ***************************************************************************
 * @file gps.h GPS Driver for Inertial/GPS NAV.
 *
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

#ifndef GPS_API_H
#define GPS_API_H

#include <stdint.h>
#include "GlobalConstants.h"

#ifdef __cplusplus
extern "C" {
#endif

void TaskGps(void const *argument);

#ifdef __cplusplus
}
#endif

// Fix type
#define INVALID     0   // GNSS not fixed yet
#define SPP         1   // Single Point Positioning
#define DGPS        2   // Pseudorange Differential
#define PPP         3   // Precise Point Positioning
#define RTK_FIX     4   // RTK Fixed
#define RTK_FLOAT   5   // RTK Float
#define DEAD_REC    6   // Dead Reckoning (will be considered as INVALID)
#define MANUAL      7   // Manual Input Mode (will be considered as INVALID)
#define SIMULATION  8   // Simulation Mode (will be considered as INVALID)

typedef struct  {
    uint8_t     gpsFixType;     // 1 if data is valid
    uint8_t     gpsUpdate;      // 1 if contains new data
    uint8_t     numSatellites;  // num of satellites in the solution    
    uint32_t    itow;           // gps Time Of Week, miliseconds

    double      latitude;       // latitude ,  degrees 
    double      longitude;      // longitude,  degrees 
    double      vNed[3];        // velocities,  m/s  NED (North East Down) x, y, z
    double      trueCourse;     // [deg]
    double      rawGroundSpeed; // [m/s]
    double      altitude;       // above WGS84 ellipsoid [m]
    double      GPSSecondFraction; 

 
    uint8_t     GPSmonth;     // mm
    uint8_t     GPSday;       // dd
    uint8_t     GPSyear;      // yy last two digits of year
    char        GPSHour;      // hh
    char        GPSMinute;    // mm
    char        GPSSecond;    // ss

    float       GPSHorizAcc, GPSVertAcc;
    float       HDOP;
    float       geoidAboveEllipsoid;    // [m] Height of geoid (mean sea level) above WGS84 ellipsoid
} gpsDataStruct_t;

extern gpsDataStruct_t gGPS, gCanGps;

/** ****************************************************************************
 * @name GetGPSData
 * @brief Get GPS data 
 * @param [in] data - pointer to external GPS structure
 * @retval N/A
 ******************************************************************************/
void  GetGPSData(gpsDataStruct_t *data);
BOOL  SetGpsBaudRate(int rate, int fApply);
BOOL  SetGpsProtocol(int protocol, int fApply);

#endif /* GPS_API_H */
