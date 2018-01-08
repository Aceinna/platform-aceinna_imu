/** ***************************************************************************
 * @file processNovAtelGPS.c interface NovAtel OEM4 GPS receiver.
 * @author Dong An
 * @date   2009-04-10 23:20:59Z
 * @copyright (c) 2013, 2014 All Rights Reserved.
 * @section LICENSE
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 * @details
 *  This file provides parsing functions for NovAtel OEM4 GPS receiver.
 * @version
 * 11/06   DA 	initial creation based on the original file in UCB-beta27
 *                  refined to comply with UCB spec.
 * 03.2007 DA  Cleaned up, Doxygenized, and finalized for NAV440 release.
 *****************************************************************************/
#include <math.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include "driverGPS.h"
#include "NovAtelPacketFormats.h"
#include "filter.h"
#include "xbowsp_algorithm.h"

#define CRC32_POLYNOMIAL 0xEDB88320

static gpsDeltaStruct downVelDelta;

/** ****************************************************************************
 * @name processNovAtelBinaryMsg parse a complete NovAtel binary message.
 *
 * @param [in] msg - input buffer
 * @param [in] msgLen - input buffer length
 * @param [in] GPSData structure to parse into
 * @retval N/A
 ******************************************************************************/
void processNovAtelBinaryMsg_Fast(char          *msg,
                                  unsigned int  *msgLength,
                                  GpsData_t     *GPSData)
{
    unsigned long  calcuCRC;
    uint32_t*      pMsgCRC;

    calcuCRC = _CalculateBlockCRC32((unsigned long)(*msgLength - 4),
                                    (unsigned char *)&msg[0]);
    pMsgCRC  = (uint32_t*)&msg[*msgLength - 4];

    if (calcuCRC == *pMsgCRC) {
        switch ( *(unsigned short*)&msg[4] ) { // msgID
            case  42:  /// 0x2a bestPosB
            case  47:  /// 0x2b PSRPosB - PSeudoRange
                _parseBestPosB_Fast((logBestPosB*)msg,
                                     GPSData);
                break;
            case  99:  /// 0x63 bestVelB
            case 100:  /// 0x64 PSRVelB - PSeudoRange
                _parseBestVelB_Fast((logBestVelB*)msg,
                                    GPSData);
                break;
            default:;
        }
    }
    // bad CRC just return
}

void processNovAtelBinaryMsg(char          *msg,
                             unsigned int  *msgLength,
                             GpsData_t *GPSData)
{
	unsigned char  headerLength = 0;
	unsigned short bodyLength   = 0;
	unsigned short msgID        = 0;
	unsigned long  calcuCRC;
    uint32_t*      pRecCRC;

	calcuCRC = _CalculateBlockCRC32( (unsigned long)(*msgLength - 4),
                                     (unsigned char *)&msg[0]);

    pRecCRC = (uint32_t*)&msg[*msgLength - 4];

    if (calcuCRC == *pRecCRC) {
        headerLength = msg[3];
        bodyLength = *(unsigned short*)&msg[8];

        msgID = *(unsigned short*)&msg[4];
        switch (msgID) {
            case 42:  /// 0x2a bestPosB
            case 47:  /// 0x2b PSRPosB - PSeudoRange
                    _parseBestPosB(msg,
                                   &headerLength,
                                   &bodyLength,
                                   GPSData);
            break;
            case 99:  /// 0x63 bestVelB
            case 100:  /// 0x64 PSRVelB - PSeudoRange
                    _parseBestVelB(msg,
                                   &headerLength,
                                   &bodyLength,
                                   GPSData);
            break;
            default:;
        }
    }
    // bad CRC just return
}

/** ****************************************************************************
 * @name _parseBestPosB LOCAL parse a BESTPOSB message.
 * @author Doug Hiranaka
 * @param [in] bestPosB - input buffer cast to best pos B
 * @param [in] GPSData structure to parse into
 * @retval N/A
    1 BESTPOS header Log header H 0
    2 sol stat Solution status, see Table 85 on page 408 Enum 4 H
    3 pos type Position type, see Table 84 on page 407 Enum 4 H+4
    4 lat Latitude 9 (degrees) Double 8 H+8
    5 lon Longitude (degrees) Double 8 H+16
    6 hgt Height above mean sea level (metres) Double 8 H+24
    7 undulation - relationship between the geoid and ellipsoid (m) of the datum a Float 4 H+32
    8 datum id# Datum ID number Enum 4 H+36
    9 lat ? Latitude standard deviation (m) Float 4 H+40
    10 lon ? Longitude standard deviation (m) Float 4 H+44
    11 hgt ? Height standard deviation (m) Float 4 H+48
    12 stn id Base station ID Char[4] 4 H+52
    13 diff_age Differential age in seconds Float 4 H+56
    14 sol_age Solution age in seconds Float 4 H+60
    15 #SVs Number of satellites tracked Uchar 1 H+64
    16 #solnSVs Number of satellites used in solution Uchar 1 H+65
    17 #solnL1SVs Number of satellites with L1/E1/B1 used in solution Uchar 1 H+66
    18 #solnMultiSVs # satellites with multi-frequency signals used in solution Uchar 1 H+67
    19 Reserved Hex 1 H+68
    20 ext sol stat Extended solution status Hex 1 H+69
    21 Galileo and BeiDou sig mask Hex 1 H+70
    22 GPS and GLONASS sig mask Hex 1 H+71
    23 xxxx 32-bit CRC (ASCII and Binary only) Hex 4 H+72
 ******************************************************************************/
void _parseBestPosB_Fast(logBestPosB   *bestPosB,
                         GpsData_t     *GPSData)
{
    GPSData->lat = bestPosB->Lat;
    GPSData->lon = bestPosB->Lon;
    GPSData->alt = bestPosB->hgt; ///alt

    if (GPSData->lat < 0) {
        GPSData->lat *= -1;
        GPSData->latSign = -1;
    } else {
        GPSData->latSign = 1;
    }

    if (GPSData->lon < 0) {
        GPSData->lon *= -1;
        GPSData->lonSign = -1;
    } else {
        GPSData->lonSign = 1;
    }

    GPSData->updateFlagForEachCall |= 1 << GOT_GGA_MSG;
    if (bestPosB->sol_status != 0) {  // zero is good fix anything else is enumeratio for bad fix
        GPSData->GPSFix = 0;// PosVelType
        gAlgorithm.bitStatus.hwStatus.bit.unlockedInternalGPS = 1; // locked
        gAlgorithm.bitStatus.swStatus.bit.noGPSTrackReference = 1; // no GPS track
        gGpsDataPtr->HDOP = 21.0f; // force to above threshold
    } else {
        GPSData->GPSFix = bestPosB->pos_type;
        gAlgorithm.bitStatus.hwStatus.bit.unlockedInternalGPS = 0; // locked
        gAlgorithm.bitStatus.swStatus.bit.noGPSTrackReference = 0; // GPS track
        gGpsDataPtr->HDOP = 9.0f; // force to below threshold
    }

    GPSData->itow = (uint32_t) bestPosB->header.milliseconds; //(bestPosB->sol_age * 1000.0); // ITOW
    GPSData->LLHCounter++;

    float tmp = (float)1.9230769944E-2 * (float)bestPosB->header.week;
    float tmpYear = (uint8_t)tmp;
    GPSData->GPSyear  = (uint8_t)( tmpYear - 20 );
    GPSData->GPSmonth = (uint8_t)( ( tmp - tmpYear ) * 12 );
    if( GPSData->GPSmonth < 1 ) {
        GPSData->GPSmonth = 1;
    } else if( GPSData->GPSmonth > 12 ) {
        GPSData->GPSmonth = 12;
    }
    GPSData->GPSday   = 1;

// Unused in FW
//    GPSData->LonLatH[0] = (signed long)(GPSData->lon * 1e7);
//    GPSData->LonLatH[1] = (signed long)(GPSData->lat * 1e7);
//    GPSData->LonLatH[2] = (signed long)(GPSData->alt * 1e3);
}

/** ****************************************************************************
* @name _parseBestPosB LOCAL parse a BESTPOSB message.
* @author Dong An
* @param [in] completeMessage - input buffer
* @param [in] headerLength - header length
* @param [in] bodyLength - input buffer length
* @param [in] GPSData structure to parse into
* @retval N/A
    1 BESTPOS header Log header H 0
    2 sol stat Solution status, see Table 85 on page 408 Enum 4 H
    3 pos type Position type, see Table 84 on page 407 Enum 4 H+4
    4 lat Latitude 9 (degrees) Double 8 H+8
    5 lon Longitude (degrees) Double 8 H+16
    6 hgt Height above mean sea level (metres) Double 8 H+24
    7 undulation - relationship between the geoid and ellipsoid (m) of the datum a Float 4 H+32
    8 datum id# Datum ID number Enum 4 H+36
    9 lat ? Latitude standard deviation (m) Float 4 H+40
    10 lon ? Longitude standard deviation (m) Float 4 H+44
    11 hgt ? Height standard deviation (m) Float 4 H+48
    12 stn id Base station ID Char[4] 4 H+52
    13 diff_age Differential age in seconds Float 4 H+56
    14 sol_age Solution age in seconds Float 4 H+60
    15 #SVs Number of satellites tracked Uchar 1 H+64
    16 #solnSVs Number of satellites used in solution Uchar 1 H+65
    17 #solnL1SVs Number of satellites with L1/E1/B1 used in solution Uchar 1 H+66
    18 #solnMultiSVs # satellites with multi-frequency signals used in solution Uchar 1 H+67
    19 Reserved Hex 1 H+68
    20 ext sol stat Extended solution status Hex 1 H+69
    21 Galileo and BeiDou sig mask Hex 1 H+70
    22 GPS and GLONASS sig mask Hex 1 H+71
    23 xxxx 32-bit CRC (ASCII and Binary only) Hex 4 H+72
******************************************************************************/
void _parseBestPosB(char           *completeMessage,
                    unsigned char  *headerLength,
                    unsigned short *bodyLength,
                    GpsData_t  *GPSData)
{
	char SolStatus[4];

	// these are uint16_t but only looking at first byte
    SolStatus[0]  = completeMessage[ *headerLength];

	GPSData->lat = _assembyToLongDouble((unsigned char*)&completeMessage[*headerLength +  8]); // lat
	GPSData->lon = _assembyToLongDouble((unsigned char*)&completeMessage[*headerLength + 16]); // lon
	GPSData->alt = _assembyToLongDouble((unsigned char*)&completeMessage[*headerLength + 24]); ///alt

	if (GPSData->lat < 0) {
		GPSData->lat *= -1;
		GPSData->latSign = -1;
	} else {
		GPSData->latSign = 1;
	}

	if (GPSData->lon < 0) {
		GPSData->lon *= -1;
		GPSData->lonSign = -1;
	} else {
		GPSData->lonSign = 1;
	}

	GPSData->updateFlagForEachCall |= 1 << GOT_GGA_MSG;

	if (SolStatus[0] != 0) { // zero is good fix anything else is enumeratio for bad fix
        gGpsDataPtr->HDOP = 21.0f; // force to above threshold
        gAlgorithm.bitStatus.hwStatus.bit.unlockedInternalGPS = 1; // not locked
        gAlgorithm.bitStatus.swStatus.bit.noGPSTrackReference = 1; // no GPS track
    } else  {
        gGpsDataPtr->HDOP = 9.0f; // force to below threshold
        gAlgorithm.bitStatus.hwStatus.bit.unlockedInternalGPS = 0; // locked
        gAlgorithm.bitStatus.swStatus.bit.noGPSTrackReference = 0; // GPS track
    }
	GPSData->GPSFix = SolStatus[0];

	memcpy(&GPSData->itow, &completeMessage[16], 4); ///Itow

	GPSData->LLHCounter++;

// Unused in FW
//	GPSData->LonLatH[0] = (signed long)(GPSData->lon * 1e7);
//	GPSData->LonLatH[1] = (signed long)(GPSData->lat * 1e7);
//	GPSData->LonLatH[2] = (signed long)(GPSData->alt * 1e3);
}

/** ****************************************************************************
* @name _parseBestVelB LOCAL parse a BESTVELB message.
* @author Dong An
* @param [in] bestVelB - input buffer cast to bestVelB
* @param [in] GPSData structure to parse into
* @retval N/A
    1 BESTVEL header Log header H 0
    2 sol status Solution status (enum) 4 H
    3 vel type Velocity type, see Table 84, Position or Velocity Type 4 H+4
    4 latency in the velocity time tag in seconds. subrtrac from timeFloat 4 H+8
    5 age Differential age in seconds Float 4 H+12
    6 hor spd Horizontal speed over ground [m/s] Double 8 H+16
    7 trk gnd (track over ground) True North, [deg] Double 8 H+24
    8 vert spd Vertical speed, [m/s] + (up) - (down) Double 8 H+32
    9 Reserved Float 4 H+40
    10 xxxx 32-bit CRC (ASCII and Binary only) Hex 4 H+44
******************************************************************************/
void _parseBestVelB_Fast(logBestVelB   *bestVelB,
                         GpsData_t     *GPSData)
{
    double radTrueCourse = 0.0;

    GPSData->trueCourse = bestVelB->trk_gnd; ///COG
    radTrueCourse       = GPSData->trueCourse * D2R; // [rad]

// Error: Variable never populated so vNed is always zero
//    GPSData->vNed[0] = GPSData->rawGroundSpeed * cos(radTrueCourse); // [m/s] N
//    GPSData->vNed[1] = GPSData->rawGroundSpeed * sin(radTrueCourse); // [m/s] E
    GPSData->vNed[0] = bestVelB->hor_spd * cos(radTrueCourse); // [m/s] N
    GPSData->vNed[1] = bestVelB->hor_spd * sin(radTrueCourse); // [m/s] E
    GPSData->vNed[2] = avgDeltaSmoother( -bestVelB->vert_spd, &downVelDelta);

    GPSData->updateFlagForEachCall |= 1 << GOT_VTG_MSG;
    if (bestVelB->sol_status != 0) { // zero is good fix anything else is enumeratio for bad fix
        GPSData->GPSFix = bestVelB->vel_type;
        gAlgorithm.bitStatus.hwStatus.bit.unlockedInternalGPS = 1; // not locked
        gAlgorithm.bitStatus.swStatus.bit.noGPSTrackReference = 1; // no GPS track
        gGpsDataPtr->HDOP = 21.0f; //
    } else {
        GPSData->GPSFix = 0;
        gAlgorithm.bitStatus.hwStatus.bit.unlockedInternalGPS = 0; // not locked
        gAlgorithm.bitStatus.swStatus.bit.noGPSTrackReference = 0; // GPS track
        gGpsDataPtr->HDOP = 9.0f; //
    }

    GPSData->VELCounter++;
}


/** ****************************************************************************
 * @name _parseBestVelB LOCAL parse a BESTVELB message.
 * @author Dong An
 * @param [in] completeMessage - input buffer
 * @param [in] headerLength - header length
 * @param [in] bodyLength - input buffer length
 * @param [in] GPSData structure to parse into
 * @retval N/A
 ******************************************************************************/
void _parseBestVelB(char           *completeMessage,
                    unsigned char  *headerLength,
                    unsigned short *bodyLength,
                    GpsData_t      *GPSData)
{
	char          SolStatus[4];
    double        radTrueCourse = 0.0;

    SolStatus[0]  = completeMessage[ *headerLength];

	GPSData->rawGroundSpeed =  _assembyToLongDouble((unsigned char*)&completeMessage[*headerLength + 16]); // ground speed [m/s]
	GPSData->trueCourse     =  _assembyToLongDouble((unsigned char*)&completeMessage[*headerLength + 24]); ///COG [deg]
    radTrueCourse           = GPSData->trueCourse * D2R; // [rad]
    GPSData->vNed[0]        = GPSData->rawGroundSpeed * cos(radTrueCourse); // [m/s] N
    GPSData->vNed[1]        = GPSData->rawGroundSpeed * sin(radTrueCourse); // [m/s] E
	GPSData->vNed[2]        = -_assembyToLongDouble((unsigned char*)&completeMessage[*headerLength + 32]); ///vertical Vel [m/s] + up

	GPSData->updateFlagForEachCall |= 1 << GOT_VTG_MSG;

    if (SolStatus[0] != 0) { // zero is good fix anything else is enumeratio for bad fix
        gGpsDataPtr->HDOP = 21.0f; //
        gAlgorithm.bitStatus.hwStatus.bit.unlockedInternalGPS = 1; // not locked
        gAlgorithm.bitStatus.swStatus.bit.noGPSTrackReference = 1; // GPS track
    } else {
        gGpsDataPtr->HDOP = 9.0f; //
        gAlgorithm.bitStatus.hwStatus.bit.unlockedInternalGPS = 0; // not locked
        gAlgorithm.bitStatus.swStatus.bit.noGPSTrackReference = 0; // GPS track
    }
	GPSData->GPSFix = SolStatus[0];

	memcpy(&GPSData->itow, &completeMessage[16], 4); ///Itow

	GPSData->VELCounter++;
}


/** ****************************************************************************
 * @name assembyToLongDouble LOCAL put 8 bytes IEEE double precision into
 *       "long double" of c2000.
 * @author Dong An
 * @param [in] doubleBytes - input
 * @retval long double version of the number
 ******************************************************************************/
long double _assembyToLongDouble(unsigned char *inByte)
{
	union {
		long long     dataLong;
		long double	  dataDouble;
	} long2Double;

	memcpy(&long2Double.dataLong, inByte, 8);
	return long2Double.dataDouble;
}

/** ****************************************************************************
 * @name _CRC32Value LOCAL subroutine of CRC computation for NovAtel binary
 *       message of c2000.
 * @author Dong An
 * @param [in] i - input value
 * @retval N/A
 ******************************************************************************/
unsigned long _CRC32Value(int i)
{
	int           j;
	unsigned long ulCRC;

	ulCRC = i;
	for(j = 8; j > 0; j--) {
		if (ulCRC & 1)
		    ulCRC = (ulCRC >> 1) ^ CRC32_POLYNOMIAL;
		else
            ulCRC >>= 1;
	}
	return ulCRC;
}

/** ****************************************************************************
 * @name _CalculateBlockCRC32 LOCAL compute CRC for a complete NovAtel binary
 *       message of c2000.
 * @author Dong An
 * @param [in] ulCount - input size 32bit words
 * @param [in] ucBuffer - input value
 * @retval the calulated CRC
 ******************************************************************************/
unsigned long _CalculateBlockCRC32(unsigned long ulCount,
                                   unsigned char *ucBuffer)
{
	unsigned long ulTemp1;
	unsigned long ulTemp2;
	unsigned long ulCRC = 0;

	while (ulCount-- != 0)	{
		ulTemp1 = (ulCRC >> 8) & 0x00FFFFFFL;
		ulTemp2 = _CRC32Value( ((int)ulCRC ^ *ucBuffer++) & 0xff);
		ulCRC = ulTemp1 ^ ulTemp2;
	}
	return ulCRC;
}

void sendNovAtelBinaryCmdMsg(void)
{
	uint8_t baudCmd[] = { "COM COM1,115200,N,8,1,N,OFF,ON\r" };
	uint8_t posBCmd[] = { "log com1 BESTPOSB ontime 1\r" };
	uint8_t velBCmd[] = { "log com1 BESTVELB ontime 1\r" };

	writeGps((char*)posBCmd, strlen((char*)posBCmd));
	writeGps((char*)velBCmd, strlen((char*)velBCmd));
	writeGps((char*)baudCmd, strlen((char*)baudCmd));
}
