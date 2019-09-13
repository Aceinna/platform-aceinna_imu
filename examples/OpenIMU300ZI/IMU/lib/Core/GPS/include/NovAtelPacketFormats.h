/******************************************************************************
 *  @file NovatelPacketFormats.h
*******************************************************************************/
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


#include "driverGPS.h"
#include <stdint.h>

#pragma pack(1)
typedef struct { // 7 32 bit words
	uint8_t preamble0; // 0xAA
	uint8_t preamble1; // 0x44
	uint8_t preamble2; // 0x12
	uint8_t headerLength; // bytes 0x1c 28d

	uint16_t msgID; // 0x0100 0x2a 42d,  0x63 99d
	uint8_t  msgType; //
	uint8_t  portAddress; // 0x20 COM1

	uint16_t msgLength; // bytes payload only
	uint16_t sequence; // 0 or 1

    uint8_t  idleTime;
	uint8_t  timeStatus;
	uint16_t week; // GPS week number

    uint32_t milliseconds;

    uint32_t recStatus;

    uint16_t reserved;
	uint16_t swVersion;
} novatelBinaryHeader;
#pragma pack()

#pragma pack(1)
typedef struct {
    novatelBinaryHeader header; // #1 7 32 bit words
    uint32_t sol_status;    // #2
    uint32_t pos_type;      // #3
    double   Lat;           // #4
    double   Lon;           // #5
    double   hgt;           // #6 above msl
    float    undulation;    // #7
    uint32_t datum_id_num;  // #8
    float    lat_sigma;     // #9
    float    lon_sigma;     // #10
    float    hgt_sigma;     // #11
    uint32_t stn_id;        // #12
    float    diff_age;      // #13
    float    sol_age;       // #14
    uint8_t  num_obs;       // #15
    uint8_t  num_GPSL1;     // #16
    uint8_t  num_L1;        // #17
    uint8_t  num_L2;        // #18
    uint32_t  reserved;     // #19 - #22
    uint32_t  Crc;          // #23
    uint16_t  terminator;   // #24 /CR /LF
} logBestPosB;
#pragma pack()

#pragma pack(1)
typedef struct {
    novatelBinaryHeader header; // #1 7 32 bit words
    uint32_t sol_status;    // #2
    uint32_t vel_type;      // #3
    float    latency;       // #4
    float    age;           // #5
    double   hor_spd;       // #6 [m/s]
    double   trk_gnd;       // #7 True [deg]
    double   vert_spd;      // #8 +up [m/s]
    float    reserved;      // #9
    uint32_t Crc;           // #10
    uint16_t terminator;    // #11 /CR /LF
} logBestVelB;
#pragma pack()

unsigned long _CalculateBlockCRC32(unsigned long ulCount,
	                               unsigned char *ucBuffer);

unsigned long _CRC32Value(int i);
void processNovAtelBinaryMsg(char          *msg,
                             unsigned int  *msgLength,
                             GpsData_t     *GPSData);

void _parseBestPosB_Fast(logBestPosB    *completeMessage,
                         GpsData_t      *GPSData);

void _parseBestPosB(char           *completeMessage,
					unsigned char  *headerLength,
					unsigned short *bodyLength,
					GpsData_t      *GPSData);

void _parseBestVelB(char *completeMessage,
					unsigned char  *headerLength,
					unsigned short *bodyLength,
					GpsData_t      *GPSData);
void _parseBestVelB_Fast(logBestVelB *completeMessage,
                         GpsData_t   *GPSData);


void _decodeSelectedNovAtelASCIImsg(char          *msgID,
									char          *msgBody,
									unsigned int  *msgBodyLength,
									GpsData_t     *GPSData);

char _parseNoVatelPOSVELNAVDOPA(char          *msgBody,
								unsigned int  *msgBodyLength,
								GpsData_t     *GPSData);

long double _assembyToLongDouble(unsigned char *doubleBytes);

unsigned int _decodePositionTypeAscii(char *positionType);
