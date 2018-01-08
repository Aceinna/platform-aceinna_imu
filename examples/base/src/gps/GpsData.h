/*
* File:   GPS_Data.h
* Author: joemotyka
*
* Created on May 8, 2016, 12:35 AM
*/

#ifndef GPS_DATA_H
#define GPS_DATA_H

#define  MAX_ITOW  604800000    // Second in a week (assuming exactly 24 hours in a day)

#include <stdint.h>


enum enumGPSProtocol {
    AUTODETECT              = -1,
	UBLOX_BINARY            =  0,
	NOVATEL_BINARY          =  1,
	NOVATEL_ASCII           =  2,
	NMEA_TEXT               =  3,
    DEFAULT_SEARCH_PROTOCOL =  NMEA_TEXT, // 3
	SIRF_BINARY             =  4,
	INIT_SEARCH_PROTOCOL    =  SIRF_BINARY, ///< 4 max value, goes through each until we hit AUTODETECT
	UNKNOWN                 = 0xFF
};

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
    int      gpsValid;
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
    uint8_t              GPSmonth;     // mm
    uint8_t              GPSday;       // dd
    uint8_t              GPSyear;      // yy last two digits of year
    char                 GPSHour;      // hh
    char                 GPSMinute;    // mm
    char                 GPSSecond;    // ss
    double               GPSSecondFraction; // FIXME used?

    /// compatible with Ublox driver FIXME should these be seperate data structure?
    unsigned char        ubloxClassID;
    unsigned char        ubloxMsgID;
    signed long          LonLatH[3]; // SiRF Lat Lon[deg] * 10^7 Alt ellipse [m]*100
    char                 GPSFix;
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
    int                  GPSbaudRate; /// AutoBaud, 9600, 19200, 38400, 57600
    /// AutoDect, Ublox Binary, NovAtel binary, NovAtel ASCII, NMEA
    enum enumGPSProtocol GPSProtocol;

    universalMSGSpec     GPSMsgSignature;
    unsigned char        GPSAUTOSetting;
    unsigned char        GPSTopLevelConfig; // UBLOX
    unsigned char        resetAutoBaud;
    unsigned char        autoBaudCounter;
} GpsData_t;

extern GpsData_t *gGpsDataPtr; // definition in driverGPSAllentrance.c


#endif /* GPS_DATA_H */
