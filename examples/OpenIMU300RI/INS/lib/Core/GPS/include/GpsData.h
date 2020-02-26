/*******************************************************************************
* File:   GPS_Data.h
********************************************************************************/
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


#ifndef GPS_DATA_H
#define GPS_DATA_H

#include <stdint.h>
#include "GlobalConstants.h"

/** \struct universalMSGSpec
\brief specify an universal message spec.
*/
typedef struct {
    unsigned long  GPSheader; ///< could be 1, 2, 3, or 4 bytes - see message headers ^
    unsigned char  GPSheaderLength;
    unsigned short lengthOfHeaderIDLength;
    unsigned char  crcLength;
    unsigned char  binaryOrAscii; // 1 = ascii, 0 binary
    unsigned char  startByte; // pulled out of header
} universalMSGSpec;

/** @struct GPSDataSTRUCT
@brief global data structure for all GPS interface and process.

This Data structure is not only used for all GPS interface and process
but also used to be accessed by other modules rather than GPS files.
*/
typedef struct  {
    uint8_t         gpsFixType;    // GPS solution type: invalid, spp, dgps, spp, rtk_float, rtk_fix
    uint8_t         numSatellites;

    uint8_t         GPSmonth;      // mm
    uint8_t         GPSday;        // dd
    uint8_t         GPSyear;       // yy last two digits of year
    char            GPSHour;       // hh
    char            GPSMinute;     // mm
    char            GPSSecond;     // ss
    double          GPSSecondFraction; // FIXME used?

    uint32_t        itow;           ///< gps milisecond Interval Time Of Week
    int             updateFlagForEachCall; /// changed to 16 bits

    int             totalGGA;
    int             totalVTG;
    
    double          lat;           // [deg], latitude
    double          lon;           // [lon], longitude
    double          alt;          // [m] above WGS84 ellipsoid
    double          vNed[3];       // NED North East Down [m/s] x, y, z
    double          trueCourse; // [deg]
    double          rawGroundSpeed; // [m/s]

    float                geoidAboveEllipsoid;    // [m] Height of geoid (mean sea level) above WGS84 ellipsoid

    /// compatible with Ublox driver FIXME should these be seperate data structure?
    unsigned char        ubloxClassID;
    unsigned char        ubloxMsgID;
    signed long          LonLatH[3]; // SiRF Lat Lon[deg] * 10^7 Alt ellipse [m]*100 <-- UNUSED
    float                HDOP;       // Horizontal Dilution Of Precision x.x
    double               GPSVelAcc;
    unsigned short       GPSStatusWord;  /// will replace GPSfix
    unsigned char        isGPSFWVerKnown;
    unsigned char        isGPSBaudrateKnown;
    unsigned long        Timer100Hz10ms;
    unsigned char        ubloxOldVersion;
    float                UbloxSoftwareVer;
    unsigned int         navCFGword;
    unsigned int         nav2CFGword;
    char                 GPSConfigureOK; /// always needs to be initialized as -1

    unsigned char        reClassID;
    unsigned char        reMsgID;

    unsigned long        LLHCounter;
    unsigned long        VELCounter;
    unsigned long        STATUSCounter; // UBLOX - or first flag SiRF
    unsigned long        SBASCounter;
    unsigned long        firewallCounter;
    unsigned long        firewallRunCounter;
    unsigned long        reconfigGPSCounter;

    /// GPS Baudrate and protocal: -1, 0,1, 2, 3 corresponding to
    int                  GPSbaudRate;    /// 4800, 9600, 19200, 38400, 57600, 115200, etc
    /// AutoDect, Ublox Binary, NovAtel binary, NovAtel ASCII, NMEA
    enumGPSProtocol      GPSProtocol;

    universalMSGSpec     GPSMsgSignature;
    unsigned char        GPSAUTOSetting;
    unsigned char        GPSTopLevelConfig; // UBLOX
    unsigned char        resetAutoBaud;
    unsigned char        autoBaudCounter;

    //uint8_t              sirfInitialized;
    //float                latQ;
    //float                lonQ;
    //float                hgtQ;
    //uint8_t              useSigmas;

    float                GPSHorizAcc;
    float                GPSVertAcc;
} GpsData_t;

extern GpsData_t *gGpsDataPtr; // definition in driverGPSAllentrance.c


#endif /* GPS_DATA_H */
