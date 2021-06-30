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

#ifdef __cplusplus
extern "C" {
#endif

// Fix type
enum
{

    INVALID = 0,    // GNSS not fixed yet
    SPP = 1,        // Single Point Positioning
    DGPS = 2,       // Pseudorange Differential
    PPP = 3,        // Precise Point Positioning
    RTK_FIX = 4,    // RTK Fixed
    RTK_FLOAT = 5,  // RTK Float
    DEAD_REC = 6,   // Dead Reckoning (will be considered as INVALID)
    MANUAL = 7,     // Manual Input Mode (will be considered as INVALID)
    SIMULATION = 8, // Simulation Mode (will be considered as INVALID)
};

typedef struct  {
    uint8_t     gpsFixType;     // 1 if data is valid
    uint8_t     gpsUpdate;      // 1 if contains new data
    uint8_t     numSatellites;  // num of satellites in the solution    
    uint32_t    itow;           // gps Time Of Week, miliseconds

    float64_t      latitude;       // latitude ,  degrees 
    float64_t      longitude;      // longitude,  degrees 
    float64_t      vNed[3];        // velocities,  m/s  NED (North East Down) x, y, z
    float64_t      trueCourse;     // [deg]
    float64_t      rawGroundSpeed; // [m/s]
    float64_t      altitude;       // above WGS84 ellipsoid [m]
    float64_t      GPSSecondFraction; 

 
    uint8_t     GPSmonth;     // mm
    uint8_t     GPSday;       // dd
    uint8_t     GPSyear;      // yy last two digits of year
    char        GPSHour;      // hh
    char        GPSMinute;    // mm
    char        GPSSecond;    // ss

    float32_t       GPSHorizAcc;
    float32_t       GPSVertAcc;
    float32_t       HDOP;
    float32_t       geoidAboveEllipsoid;    // [m] Height of geoid (mean sea level) above WGS84 ellipsoid
} gpsDataStruct_t;

extern gpsDataStruct_t gGPS;

#ifdef __cplusplus
}
#endif

#endif /* GPS_API_H */
