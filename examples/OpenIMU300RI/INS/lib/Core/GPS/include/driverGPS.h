/** ***************************************************************************
 * @file driverGPS.h  header file for all GPS interface and process files.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 * @details
 *  This file defines constants, data structures, and function
 *	prototypes. To enable GPS the config file must have <hasGps>true</hasGps>
 *  and <useGps>true</useGps> in name_IMU380.xml file. This version runs the
 *  Origin ORG4475 SiRF based GPS module.
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

#ifndef DRIVERGPS_H
#define DRIVERGPS_H

#include "GlobalConstants.h"
#include "GpsData.h"

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

/// GPSAutoSetting
#define AUTOBAUD                1
#define AUTOPROTOCOL            2

/// bit position of GPSTopLevelConfig
#define HZ2                     0
#define GPSPORT_SUPPORT420      1
#define EXTERNAL_GPS            2

/// max message length, field length
#define MAX_MSG_LENGTH     400		/// for sharing with other binary messages
									/// must be less than SCIA&B buffer size
									/// NovAtel Ascii could be big. PosVelNavDopa is 350 bytes
									/// should be <= max SCIBR buffer size
#define MAX_UBLOX_BODY_LENGTH    100

#define NMEA_MSG_BUF_SIZE 		 100    ///< Max length of NMEA
#define NMEA_MSG_MAX_ID_LENGTH	   8
#define NMEA_MSG_MAX_DATA_LENGTH   NMEA_MSG_BUF_SIZE-3
#define NMEA_MSG_MAX_FIELD		  25	///< Max length of Field

/// maxmum GPS velocity accuracy allowed of ublox
#define MAX_GPS_VELOCITY_ACCURACY   3.0 ///< m/s

/// ublox message ID
#define UBLOX_NAV_PVT       0x0107
#define UBLOX_NAV_POSLLH    0x0102
#define UBLOX_NAV_STATUS    0x0103
#define UBLOX_NAV_VELNED    0x0112
#define UBLOX_NAV_SBAS      0x0132
#define UBLOX_ACK_ACK       0x0501
#define UBLOX_CFG_CFG       0x0609
#define UBLOX_CFG_PRT       0x0600
#define UBLOX_CFG_NAV       0x0603
#define UBLOX_CFG_NAV2      0x061A
#define UBLOX_CFG_RXM       0x0611
#define UBLOX_CFG_SBAS      0x0616
#define UBLOX_CFG_MSG       0x0601
#define UBLOX_CFG_RATE      0x0608
#define UBLOX_NMEA_GLL      0xF001
#define UBLOX_NMEA_GSA      0xF002
#define UBLOX_NMEA_GSV      0xF003
#define UBLOX_NMEA_ZDA      0xF008
#define UBLOX_NMEA_VTG      0xF005
#define UBLOX_NMEA_GGA      0xF000
#define UBLOX_NMEA_RMC      0xF004
#define UBLOX_MON_VER       0x0A04
#define CFG_DONE            0xFFFF

/// some definition for ublox plug-play
#define QUERY_TIMEOUT         1000  ///< ms: 1 seconds

#define TEST_ID_RECORD   	 10   //ublox

/// message headers
#define UBLOX_BINAERY_SYNC         0xB562
#define UBLOX_BINAERY_HEADER_LEN   6
#define NOVATEL_ASCII_HEADER       0x23504F   ///< "#PO"
#define NMEA_HEADER                0x244750   ///< "$GP"
#define NOVATEL_OME4_BINARY_HEADER 0xAA4412
#define SIRF_BINARY_HEADER         0xA0A2
#define SIRF_END_SEQUENCE          0xB0B3
#define MAX_HEADER_LEN               4
#define NMEA_SYNC_1  			   0x00244750 // $GP
#define NMEA_SYNC_2  			   0x0024474E // $GN

/// maximum waiting time for a GPS message to complete
//#define GPS_PACKET_RECEIVE_TIMEOUT 500  ///< 0.5 second
#define GPS_PACKET_RECEIVE_TIMEOUT 100   /// < 100ms for 5Hz data rate

/// UBLOX msg Rate
#define NAV_SBAS_RATE_RATIO           40     ///< 40*0.25=10 sec. 0.1Hz
///constants for monitoring ublox message rate.
#define TIME_INTERVAL_FOR_RATE_CALCU  20000  ///< ms: 20 seconds
#define TICKS_TO_SECONDS              0.001
#define MIN_POSLLH_RATE  			  0.2    ///< 0.2 Hz timeout
#define MIN_VELNED_RATE  			  0.2    ///< 0.2 Hz timeout
#define MIN_STATUS_RATE  			  0.2    ///< 0.2 Hz timeout
#define MIN_SBAS_RATE  			      0.0    ///< completely stop
#define MAX_LOW_RATE_COUNT            3

/// ublox FW version threshold
#define NAV2_ENABLED_VERSION 3.04

/// used for autoProtocol
#define MAX_PROTOCOL_SEARCHTIME 2000  ///< 2 seconds
#define INIT_SEARCH_BAUD           3  ///< 3 - BAUD_57600

///for NMEA vertical process
#define ALT_FILTER_GAIN         0.2    ///< 1/(4+1)
#define VD_FILTER_GAIN          0.0476 ///< 1/(20+1)
#define MAX_ALTI_JUMP          10.0    ///< meters
#define MIN_DELTA_T             1.e-5  ///< seconds 0.01 ms
#define MAX_DELTA_T             2      ///< seconds
#define MAX_VEL_JUMP            5

/** struct ubloxIDTypeSTRUCT
	@brief specify ublox message ID and associated message rate and target port.
 */
typedef struct 
{
    uint32_t iTOW;      // ms
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t valid;      // Validity flags
    uint32_t tAcc;      // time accuracy estimate (UTC), ns
    int32_t nano;       // fraction of second, range -1e9..1e9 (UTC), ns
    uint8_t fixType;    /* GNSS fix type
                         *  0: no fix
                         *  1: dead reckoning only
                         *  2: 2D-fix
                         *  3: 3D-fix
                         *  4: GNSS + dead reckoning combined
                         *  5: time only fix
                         */
    uint8_t flags;      /* Fix status flags
                         *  bit0:   gnssFixOK, 1 = valid fix (i.e within DOP & accuracy masks)
                         *  bit1:   diffSoln, 1 = differential corrections were applied
                         *  bit2-4: psmState, Power Save Mode state
                         *  bit5:   headVehValid, 1 = heading of vehicle is valid
                         *  bit6-7: carrSoln, Carrier phase range solution status:
                         *      0: no carrier phase range solution
                         *      1: float solution
                         *      2: fixed solution 
                         */
    uint8_t flags2;
    uint8_t numSV;      // number of satellites in Nav solution;
    int32_t lon;        // deg, scaling is 1e-7
    int32_t lat;        // deg, scaling is 1e-7
    int32_t height;     // mm, height above ellipsoid;
    int32_t hMSL;       // mm, hegiht above mean seal level
    uint32_t hAcc;      // mm, horizontal accuracy estimate
    uint32_t vAcc;      // mm, vertical accuracy estimate
    int32_t velN;       // mm/s, north velocity
    int32_t velE;       // mm/s, east velocity
    int32_t velD;       // mm/s, dow velocity
    int32_t gSpeed;     // mm/s, groud speed (2-D)
    int32_t headMot;    // deg, scaling is 1e-5, heading of motion (2-D)
    uint32_t sAcc;      // mm/s, speed accuracy estimate
    uint32_t headAcc;   // deg, scaling is 1e-5, heading accuracy estimate
    uint16_t pDOP;      // scaling is 0.01, position DOP
    uint8_t reserved1[6];
    int32_t headVeh;    // deg, scaling is 1e-5, heading of vehicle (2-D)
    int16_t magDec;     // deg, scaling is 1e-2, magnetic declination
    uint16_t magAcc;    // deg, scaling is 1e-2, magnetic declination accuracy
} ubloxNavPvtSTRUCT;

typedef struct {
	char classID;
	char msgID;
	char cfgClassID;
	char cfgMsgID;
	char rateRatio;
	char whatTagert;
}ubloxIDTypeSTRUCT;

typedef struct  {
	int    lat_deg;
	int    lat_min;
	double lat_min_fraction;
	int    lon_deg;
	int    lon_min;
	double lon_min_fraction;
    char latSign;
    char lonSign;
} NmeaLatLonSTRUCT;

// delta struct
typedef struct {
    double oldValues[3]; // moving window of delta values
    double sum;
    double last; // Last whole value
} gpsDeltaStruct;

double avgDeltaSmoother(double in, gpsDeltaStruct* data);
void   thresholdSmoother( double dataIn[3], float dataOut[3]);


/// function prototypes

/// interface with algorithms
// these are duplicated in gps.h
// driverGPSAllEntrance.cpp
void GPSHandler(void);

void initGPSHandler(void); /// note: should pass in SPI or UART

void initGPSDataStruct(void); /// note: should pass in SPI or UART

/// ublox binary processUbloxGPS.cpp
void configUbloxGPSReceiver(int *bytesFromBuffer, GpsData_t* GPSData);
void processUbloxBinaryMessage(char *msg,unsigned int *msgLength, GpsData_t	*GPSData);
unsigned char configurateUBloxGPSPerformance (GpsData_t* GPSData);
unsigned char configurateUBloxGPSIOMsgRate (GpsData_t* GPSData);
unsigned char getConnectedWithUnknownStatusUbloxGPS(GpsData_t* GPSData);
void pollUbloxMsg(ubloxIDTypeSTRUCT *IDInput, GpsData_t *GPSData);

/// NMEA processNMEAGPS.cpp
void processNMEAMessage(char *msg, unsigned int *msgLength, GpsData_t* GPSData);
char extractNMEAfield(char *msgBody, char *fieldOutput, int fieldIndex, int MAXFieldLength);
char computeNMEAChecksum(char *NMEAMess, unsigned int *lengthBeforeStar);
void convertNorhEastVelocity(GpsData_t* GPSData);
void convertItow(GpsData_t* GPSData);

/// SiRF  binary processsSiRFGPS.cpp
void processSiRFBinaryMessage(char *msg,unsigned int *msgLength, GpsData_t	*GPSData);
void pollSiRFVersionMsg(void);
void sirfBinarySwitchBaud(uint32_t baud);
void configSiRFGPSReceiver(GpsData_t* GPSData, int goalBaudRate);

/// NovAtel binary processNovAtelGPS.cpp
void processNovAtelBinaryMsg(char *msg,unsigned int *msgLength, GpsData_t* GPSData);
void processNovAtelBinaryMsg_Fast(char *msg, unsigned int *msgLength, GpsData_t *GPSData);

// driver interface driverGPS.cpp
extern uint32_t  baudEnumToBaudRate(int baudRate);

int      gpsBytesAvailable();
int      initGpsUart(int baudrate);
void     initOnePpsUart( void );
//void     setExternalPortPtr(port_struct* gGpsPortPtr);
//void     flushGPSRecBuf(void);
//BOOL     isGpsTxEmpty(void);
//uint16_t delBytesGpsBuf(uint16_t numToPop);
//uint16_t peekWordGpsBuf(uint16_t index);
//uint8_t  peekByteGpsBuf(uint16_t index);
//unsigned long peekGPSmsgHeader(uint16_t index, GpsData_t *GPSData);
int      writeGps(char     *send, uint16_t len);
//int16_t  findHeader(uint16_t numInBuff, GpsData_t *GPSData);
//int16_t  retrieveGpsMsg(uint16_t numBytes, GpsData_t *GPSData, uint8_t *outBuffer);
//unsigned char autobaud(GpsData_t* GPSData);
BOOL HandleGps(GpsData_t *GPSData);
int parseNMEAMessage(uint8_t inByte, uint8_t *gpsMsg, GpsData_t *GPSData);
int parseNovotelBinaryMessage(uint8_t inByte, uint8_t *gpsMsg, GpsData_t *GPSData);
int parseUbloBinaryMessage(uint8_t inByte, uint8_t *gpsMsg, GpsData_t *GPSData);



#define GPS_INTERFACE_RX_BUFFER_SIZE 512
#define GPS_INTERFACE_TX_BUFFER_SIZE  30

#endif /// END OF DRIVERGPS_H
