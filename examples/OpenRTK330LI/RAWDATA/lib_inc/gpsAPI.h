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
#include "rtcm.h"
#include "timer.h"

/// bit position of updateFlagForEachCall;
#define GOT_GGA_MSG  			 0
#define GOT_VTG_MSG  			 1
#define W_CONTROL_POINT_EEPROM   2
#define W_GPS_DELAY_EEPROM       3
#define W_VTG_CONF_EEPROM        4
#define W_BYPASS_CONF_EEPROM     5
#define GOT_UBLOX_ACK            8  /// MSB byte for ublox specific
#define GOT_UBLOX_VNED           9
#define GOT_UBLOX_NAVSBAS       10

/// bit position of GPSStatusWord;
#define DGPS_ON                 0
#define WAAS_ON                 1
#define EGNOS_ON				2
#define MSAS_ON  				3
#define CLEAR_DGPS_SOURCES   0x01

void initGPSDataStruct(void); /// note: should pass in SPI or UART

// FIXME ECW: implement GpsWhoAmI and GpsSelfTest
uint8_t  GpsWhoAmI(uint32_t *whoami); /// returns true if value is as expected
uint8_t  GpsSelfTest();

// debugging stream out the debug port
#define GPS_NO_STREAM         0
#define GPS_NMEA_DEBUG_STREAM 1

// #define MAX_ITOW (604800000) // ms

#ifndef STREAM_GPS
//#define STREAM_GPS GPS_NO_STREAM
#define STREAM_GPS GPS_NMEA_DEBUG_STREAM
#endif

#ifdef __cplusplus
extern "C" {
#endif

void GnssDataAcqTask(void const *argument);

#ifdef __cplusplus
}
#endif

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

typedef struct  {
    uint8_t              gpsFixType;     // 1 if data is valid
    uint8_t              gpsUpdate;      // 1 if contains new data
    uint8_t              numSatellites;  // num of satellites in the solution    
    double               latitude;   // latitude ,  degrees 
    double               longitude;  // longitude,  degrees 
    double                vNed[3];   // velocities,  m/s  NED (North East Down) x, y, z
    double               trueCourse; // [deg]
    double               rawGroundSpeed;    // NMEA kph, SiRf m/s - change to m/s
    double               altitude;          // above mean sea level [m]
    float                altEllipsoid; // [km] altitude above ellipsoid for WMM

    uint16_t             week;
    uint32_t             itow;         // gps Time Of Week, miliseconds
    mcu_time_base_t      rovTime;          //gnss time rov->time

    float                GPSHorizAcc, GPSVertAcc;
    float                HDOP, sol_age;
    float                velHorAcc;
    float                geoidAboveEllipsoid;    // [m] Height of geoid (mean sea level) above WGS84 ellipsoid
	float latitude_standard_deviation;	//!< latitude standard deviation (m)
	float longitude_standard_deviation;	//!< longitude standard deviation (m)
	float height_standard_deviation;	//!< height standard deviation (m)

    uint8_t num_gnss_psr;
    uint8_t num_gnss_adr;

} gpsDataStruct_t;

extern gpsDataStruct_t *gPtrGnssSol;
extern gpsDataStruct_t gGnssSol;


/** @struct GPSDataSTRUCT
@brief global data structure for all GPS interface and process.

This Data structure is not only used for all GPS interface and process
but also used to be accessed by other modules rather than GPS files.
*/
typedef struct  {
    BOOL                 gpsFixType;
    int                  latSign;
    int                  lonSign;
    long double          lat; // concatinated from int components [deg.dec]
    long double          lon;
    double               vNed[3];    // NED North East Down [m/s] x, y, z
    uint32_t             itow;           ///< gps milisecond Interval Time Of Week

    int                  updateFlagForEachCall; /// changed to 16 bits
    int                  totalGGA;
    int                  totalVTG;

    double               trueCourse; // [deg]
    double               rawGroundSpeed; // NMEA kph, SiRf m/s

    double               alt;          // above mean sea level [m]
    double               filteredAlt; // FIXME should this be local?
    float                altEllipsoid; // [km] altitude above ellipsoid for WMM
    // uint8_t              GPSmonth;     // mm
    // uint8_t              GPSday;       // dd
    // uint8_t              GPSyear;      // yy last two digits of year
    // char                 GPSHour;      // hh
    // char                 GPSMinute;    // mm
    // char                 GPSSecond;    // ss
    // double               GPSSecondFraction; // FIXME used?

    /// compatible with Ublox driver FIXME should these be seperate data structure?
#ifdef USE_UBLOX
    unsigned char        ubloxClassID;
    unsigned char        ubloxMsgID;
    unsigned char        ubloxOldVersion;
    float                UbloxSoftwareVer;
#endif
    signed long          LonLatH[3]; // SiRF Lat Lon[deg] * 10^7 Alt ellipse [m]*100 <-- UNUSED
    char                 GPSFix;
    float                HDOP;       // Horizontal Dilution Of Precision x.x
    double               GPSVelAcc;
    unsigned short       GPSStatusWord;  /// will replace GPSfix
    unsigned char        isGPSFWVerKnown;
    unsigned char        isGPSBaudrateKnown;
    unsigned long        Timer100Hz10ms;
 
    unsigned int         navCFGword;
    unsigned int         nav2CFGword;
    char                 GPSConfigureOK; /// always needs to be initialized as -1

    unsigned char        reClassID;
    unsigned char        reMsgID;

    //unsigned long        LLHCounter;
    //unsigned long        VELCounter;
    //unsigned long        STATUSCounter; // UBLOX - or first flag SiRF
    //unsigned long        SBASCounter;
    //unsigned long        firewallCounter;
    //unsigned long        firewallRunCounter;
    //unsigned long        reconfigGPSCounter;

    /// GPS Baudrate and protocal: -1, 0,1, 2, 3 corresponding to
    int                  GPSbaudRate;    /// 4800, 9600, 19200, 38400, 57600, 115200, etc
    /// AutoDect, Ublox Binary, NovAtel binary, NovAtel ASCII, NMEA
    enumGPSProtocol      GPSProtocol;

    universalMSGSpec     GPSMsgSignature;
    unsigned char        GPSAUTOSetting;
    unsigned char        GPSTopLevelConfig; // UBLOX
    //unsigned char        resetAutoBaud;
    //unsigned char        autoBaudCounter;

    //uint8_t              sirfInitialized;
    //float                latQ;
    //float                lonQ;
    //float                hgtQ;
    //uint8_t              useSigmas;

    float                GPSHorizAcc;
    float                GPSVertAcc;

    int                  numSatelites;

    unsigned char gnss_data_flag;
    unsigned char new_eph_flag;
    gnss_rtcm_t rtcm; /* store RTCM data struct for RTK and PPP */

    /* for algorithm */
    obs_t rov;
    obs_t ref;
    nav_t nav;

} GpsData_t;


BOOL  SetGpsBaudRate(int rate, int fApply);
BOOL  SetGpsProtocol(int protocol, int fApply);

#endif /* GPS_API_H */
