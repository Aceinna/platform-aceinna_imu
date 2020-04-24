/** ***************************************************************************
 * @file processNMEAGPS.c
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 * @betails Processing for GPS NMEA protocol.
 *
 *  	This includes all specific processing for NMEA protocol.
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
#ifdef GPS

#include <math.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

#include "gpsAPI.h"
#include "driverGPS.h"
#include "BITStatus.h"

char _parseGPGGA(char *msgBody, GpsData_t* GPSData);
char _parseVTG(char *msgBody, GpsData_t* GPSData);
char _parseRMC(char *msgBody, GpsData_t* GPSData);
void _handleNMEAmsg(char *msgID, char *msgBody,GpsData_t* GPSData);
void _NMEA2UbloxAndLLA(NmeaLatLonSTRUCT* NmeaData, GpsData_t* GPSData);
void _smoothVerticalData(GpsData_t* GPSData);

int dayofweek(int day, int month, int year);

int crcError  = 0;
int starError = 0;
/** ****************************************************************************
 * @name: processNMEAMessage parse a complete NMEA message.
 * @author  Dong An
 * @param [in] msg - data to parse
 * @param [in] msgLength - length of data to parse
 * @param [in] GPSData - data structure to parse into
 * @retval N/A
 * @details Already found the header, checks for '*' delimeter and checksum
 *          before calling decode
 * 07/01/15 dkh removed redundant checks and simplified conversion from ascii
 *          to int
 ******************************************************************************/
void processNMEAMessage(char          *msg,
                        unsigned int  *msgLength,
                        GpsData_t *GPSData)
{
	volatile char  checksumRcvd;
    volatile char  checksumCalc;
    unsigned int lengthBeforeStar;
	uint8_t      tmp;

	if (msg[*msgLength - 5]!= '*'){
		starError++;
        return; /// no '*' delimiter before checksum

	}
    // convert hex "digits" from ascii to int
	tmp = msg[*msgLength - 4];
	checksumRcvd = tmp > '9'? tmp - '7' : tmp - '0';
	checksumRcvd <<= 4;
	tmp = msg[*msgLength - 3];
	checksumRcvd |= tmp > '9'? tmp - '7' : tmp - '0';

	lengthBeforeStar = *msgLength - 5;
	checksumCalc     = computeNMEAChecksum(msg, &lengthBeforeStar);
	if (checksumCalc == checksumRcvd) {
        _handleNMEAmsg(msg,     // header "$GP..."
                       &msg[7], // payload
                       GPSData);
    }else{
		crcError++;
		checksumCalc  = computeNMEAChecksum(msg, &lengthBeforeStar);
    }
}

/** ****************************************************************************
 * @name: parseNMEAMessage parse a complete NMEA message.
 * @author  Dong An
 * @param [in] inByte    - next byte from serial stream
 * @param [in] gpsMsg    - GPS message buffer
 * @param [in] GPSData   - data structure to parse into
 * @retval N/A
 * @details Extract message from serial byte stream and process
 ******************************************************************************/
int parseNMEAMessage(uint8_t inByte, uint8_t *gpsMsg, GpsData_t *GPSData)
{
    static int state = 0;
    static unsigned int len = 0;
    static uint8_t *ptr;
	static uint32_t sync = 0;
	unsigned static int  synced = 0;

    sync   = (sync << 8) | inByte;
    synced = 0;

    if ((sync & 0x00ffffff) == NMEA_SYNC_1)
    {
        synced = 1;
        gpsMsg[0] = '$';
        gpsMsg[1] = 'G';
        gpsMsg[2] = 'P';
    } else if ((sync & 0x00ffffff) == NMEA_SYNC_2) {
        synced = 1;
        gpsMsg[0] = '$';
        gpsMsg[1] = 'G';
        gpsMsg[2] = 'N';
    }

    // After getting synced, start to wait for the full message
    if (synced){
        state = 1;
        ptr   = &gpsMsg[3];
        len   = 3;
        return 0;
    }

    if(state == 0){
        return 0;
    }
    
    // After getting synced, put each input byte into gpsMsg
    *ptr++ = inByte;
    len++;

    if (len >= MAX_MSG_LENGTH){
        // overflow - reset packet engine
        state = 0;
        return 1;
    }

    if (inByte == 0x0a)
    { // LF - last character
        // got full message - process it
        processNMEAMessage((char *)gpsMsg, &len, GPSData);
        state = 0;
    }

    return 0;
} 



/** ****************************************************************************
 * @name: _handleNMEAmsg decode specific GPS NMEA messages.
 * @author  Dong An
 * @param [in] msgId - message ID
 * @param [in] msgBody - data to parse
 * @param [in] GPSData - data structure to parse into
 * @retval actual baud rate value
 * @details GGA is fix data
 * VTG Vector track ofer the Ground
 * 07/01/15 dkh changed to strncmp so don't need to add '\0'
 ******************************************************************************/
void _handleNMEAmsg(char          *msgID,
                    char          *msgBody,
                    GpsData_t     *GPSData)
{
	char *ptr = (char *)&msgID[3];

    if( strncmp(ptr, "GGA", 3) == 0 ) {
		GPSData->totalGGA++;
		_parseGPGGA(msgBody, GPSData);
    }
    if( strncmp(ptr, "VTG", 3) == 0 ) {
		GPSData->totalVTG++;
		_parseVTG(msgBody, GPSData);
    }
    if( strncmp(ptr, "RMC", 3) == 0 ) {
		_parseRMC(msgBody, GPSData);
    }
}

/** ****************************************************************************
 * @name: extractNMEAfield extract a field from NMEA body.
 * @author  Dong An
 * @param [in] msgBody - data to parse
 * @param [out] outField - the data field extracted
 * @param [in] fieldIndex - index of the field for the data
 * @param [in] resetFlag - sets the indeies to zero
 * @retval status
 * @details NMEA messages are delimited with ',' and '*' to end message before
 * the checksum byte
 * 07/01/15 dkh changed to continue search starting from last found field
 ******************************************************************************/
char extractNMEAfield(char *msgBody,
                      char *outField,
                      int  fieldIndex,
                      int  resetFlag)
{
	static int searchIndex  = 0; // input array index
	static int currentField = 0; // delimeter ',' or '*' number
    int        outIndex     = 0; // output array index

	if(msgBody == NULL || outField == NULL || NMEA_MSG_MAX_FIELD <= 0) {
		return 0; // bad pointer check
	}
    // Continues indecies from the last search to avoid going through the entire
    // sentence every time. After done parsing a sentance needs to be reset.
    if (resetFlag == 1) {
        searchIndex  = 0;
        currentField = 0;
    }

    /// Scan through until the local field number matches the requested one
	while(currentField != fieldIndex && msgBody[searchIndex]) {
		if(msgBody[searchIndex] == ',') {
			currentField++; // count delimters
		}
		searchIndex++;
	}

    /// Next element in the array is a delimter or empty: return a empty buffer
    if( (msgBody[searchIndex] == ',' ||
         msgBody[searchIndex] == '*' ||
         msgBody[searchIndex] == '\x0') ) {
        outField[0] = '\x0';
        return 0; // check for end of buffer
    }

    /// Scan and load output buffer until the next delimter is found. outIndex
    /// starts at 0 here
	while(msgBody[searchIndex] != ',' &&
          msgBody[searchIndex] != '*' &&
          msgBody[searchIndex])	{
		outField[outIndex] = msgBody[searchIndex];
		outIndex++;
        searchIndex++;

		if(outIndex >= NMEA_MSG_MAX_FIELD) {
			outIndex = NMEA_MSG_MAX_FIELD - 1;
			break;
		}
	}
    outField[outIndex] = '\x0';

    // Leave the currentField as the one returned so the next search passes
    // the currrent doesn't match requested test
    // Back up one so the next search picks up the end ','
    searchIndex--;
	return 1;
}

/** ****************************************************************************
 * @name: _parseGPGGA LOCAL parse GPGGA message. Time, position, fix data
 * @author  Dong An
 * @param [in] msgBody - data to parse
 * @param [in] GPSData - data structure to parst he data into
 * @retval status
 * @details
 *   $GPGGA       Global Positioning System Fix Data
 *   123519.00    Fix taken at 12:35:19.00 UTC
 *   4807.038,N   Latitude 48 deg 07.038' N
 *   01131.000,E  Longitude 11 deg 31.000' E
 *   1            Fix quality: 0 = invalid
 *                             1 = GPS fix (SPS)
 *                             2 = DGPS fix
 *                             3 = PPS fix
 *		                       4 = Real Time Kinematic
 *		                       5 = Float RTK
 *                             6 = estimated (dead reckoning) (2.3 feature)
 * 		                       7 = Manual input mode
 *  		                   8 = Simulation mode
 *   08           Number of satellites being tracked
 *   0.9          Horizontal dilution of position
 *   545.4,M      Altitude, Meters, above mean sea level
 *   46.9,M       Height of geoid (mean sea level) above WGS84
 *                    ellipsoid
 *   (empty field) time in seconds since last DGPS update
 *   (empty field) DGPS station ID number
 *   *47          the checksum data, always begins with *
 * $GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
 * $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
 * 01/07/15 dkh load data structure directly, convert directly from ascii to
 *          decimal
 * 02/16/15 dkh move convert to after check for flag
 ******************************************************************************/
#include "debug.h"
char _parseGPGGA(char          *msgBody,
                 GpsData_t *GPSData)
{
	char   field[NMEA_MSG_MAX_FIELD];
	char   status = 0;
    int    parseReset = true;
    NmeaLatLonSTRUCT nmeaLatLon;

    memset(&nmeaLatLon, 0, sizeof(nmeaLatLon) );
	/// Time - convert from ascii digits to decimal '0' = 48d
	if( extractNMEAfield(msgBody, field, 0, parseReset) )	{
		GPSData->GPSHour   = ( (field[0] - '0') * 10) + field[1] - '0'; /// Hour
		GPSData->GPSMinute = ( (field[2] - '0') * 10) + field[3] - '0'; /// minute
		GPSData->GPSSecond = ( (field[4] - '0') * 10) + field[5] - '0'; /// Second

        if (field[6] == '.') {
            // Some messages have second fraction .SS skip the '.'
		    GPSData->GPSSecondFraction =  ( (field[7] - '0') / 10) + (field[8] - '0') / 100;
        }
	} else
        status = 1;

    parseReset = false;
	/// Latitude
	if( extractNMEAfield(msgBody, field, 1, parseReset) ) 	{
		nmeaLatLon.lat_min_fraction = atof((char *)field + 4);
		nmeaLatLon.lat_min          = ( (field[2] - '0') * 10) + field[3] - '0';
		nmeaLatLon.lat_deg          = ( (field[0] - '0') * 10) + field[1] - '0';
	} else
        status = 1;

	if( extractNMEAfield(msgBody, field, 2, parseReset) )	{
		if(field[0] == 'S') // // sign
            nmeaLatLon.latSign = -1;
		else
            nmeaLatLon.latSign = 1;
	} else
        status = 1;

	/// Longitude
	if( extractNMEAfield(msgBody, field, 3, parseReset) )	{
		nmeaLatLon.lon_min_fraction = atof((char *)field + 5);
		nmeaLatLon.lon_min          = ( (field[3] - '0') * 10) + field[4] - '0';
		nmeaLatLon.lon_deg          = ( (field[0] - '0') * 100) + ((field[1] - '0') * 10) + (field[2] - '0') ;
	} else
        status = 1;

	if( extractNMEAfield(msgBody, field, 4, parseReset) )	{
		if(field[0] == 'W') // sign
            nmeaLatLon.lonSign = -1;
		else
            nmeaLatLon.lonSign = 1;
	} else
        status = 1;

	/// GPS quality
	if( extractNMEAfield(msgBody, field, 5, parseReset) )	{
        GPSData->gpsFixType = field[0] - '0'; // convert ascii digit to decimal
        if (GPSData->gpsFixType >= DEAD_REC)  // DR, manual and simulation is considered invalid
        {
            GPSData->gpsFixType = INVALID;
        }
	} else
        status = 1;

    // Number of satellites
    if( extractNMEAfield(msgBody, field, 6, parseReset) )	{
        GPSData->numSatellites = atoi((char *)field);
	} else
        status = 1;

	/// HDOP x.x
	if( extractNMEAfield(msgBody, field, 7, parseReset) )	{
		GPSData->HDOP = (field[0] - '0') + (field[2] - '0') * 0.1f ;// convert ascii digit to decimal
	} else
        status = 1;

	/// Altitude
	if( extractNMEAfield(msgBody, field, 8, parseReset) )	{
		GPSData->alt = atof((char *)field); // altitude above MSL
	} else{
        status = 1;
	}
    if( extractNMEAfield(msgBody, field, 10, parseReset) )	{
		GPSData->geoidAboveEllipsoid = atof((char *)field);
	} else{
        status = 1;
	}

    // if fixed, convert data
    if(GPSData->gpsFixType > INVALID) 
    {
        // Convert deg/min/sec to xxx.xxx deg
        _NMEA2UbloxAndLLA(&nmeaLatLon, GPSData);
        // convert geiod height to ellipsoid height
        GPSData->alt += GPSData->geoidAboveEllipsoid;
        // create pseudo ITOW
        convertItow(GPSData); 

        gBitStatus.hwStatus.bit.unlockedInternalGPS = 0; // locked
        gBitStatus.swStatus.bit.noGPSTrackReference = 0; // GPS track
    }
    else
    {
        gBitStatus.hwStatus.bit.unlockedInternalGPS = 1; // no signal lock
        gBitStatus.swStatus.bit.noGPSTrackReference = 1; // no GPS track
    }
    
	if( status == 0) {
		GPSData->updateFlagForEachCall |= 1 << GOT_GGA_MSG;
		GPSData->LLHCounter++;
        GPSData->GPSHorizAcc = GPSData->HDOP * 3.0;         // just an approximation
        GPSData->GPSVertAcc = GPSData->GPSHorizAcc * 1.5;   // just another approximation
	}

	return status;
}

/** ****************************************************************************
 * @name: _parseVTG LOCAL parse GPVTG message. track good speed over ground
 * @author  Dong An
 * @param [in] msgBody - data to parse
 * @param [in] GPSData - data structure to parst he data into
 * @retval status
 * @details $GPVTG   Track made good and ground speed
 *      054.7,T      Course (degrees)
 *      034.4,M      Magnetic track made good
 *      005.5,N      Ground speed, knots
 *      010.2,K      Ground speed, Kilometers per hour
 *      *48          Checksum
 * $GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48
 * 07/01/15 dkh load data structutre directly skipping local variables
 ******************************************************************************/
char _parseVTG(char          *msgBody,
               GpsData_t     *GPSData)
{
	char   field[NMEA_MSG_MAX_FIELD]; // [25]
	char   status = 0;
    int    parseReset = 1;

	///true course [deg]
	if( extractNMEAfield(msgBody, field, 0, parseReset) )	{
		GPSData->trueCourse = atof((char *)field); // double
	} else
        status = 1;

    parseReset = false;
	/// speed over ground [km/hr]
	if( extractNMEAfield(msgBody, field, 6, parseReset) )	{
		GPSData->rawGroundSpeed = atof((char *)field); // double [kph]
        GPSData->rawGroundSpeed *= 0.2777777777778;    // convert km/hr to m/s
        // the heading of the VTG of some receiver is empty when velocity is low.
        //  a default zero will be set to the heading value.
        if ( status == 1 ) {
            status = 0;
            GPSData->trueCourse = 0.0;
        }
	} else
        status = 1;

	if( status  == 0 ) {
		/// Fill into user buffer
		GPSData->updateFlagForEachCall |= 1 << GOT_VTG_MSG;
		GPSData->VELCounter++;
        convertNorhEastVelocity(GPSData); // split ground speed to NED
	}
	return status;
}

/** ****************************************************************************
 * @name: _parseRMC LOCAL parse GPRMC message. duplicates nav data but has
 *        calander data needed for WMM (Worl Magnetic Model) lookup
 * @author  Dong An
 * @param [in] msgBody - data to parse
 * @param [in] GPSData - data structure to parst he data into
 * @retval status
 * @details       $GPRMC (Recommended Minimum sentence C)
 *   123519       Fix taken at 12:35:19 UTC
 *   A            Status A=active or V=Void.
 *   4807.038,N   Latitude 48 deg 07.038' N
 *   01131.000,E  Longitude 11 deg 31.000' E
 *   022.4        Speed over the ground in knots
 *   084.4        Track angle in degrees True
 *   230394       Date - 23rd of March 1994 ddmmyy
 *   003.1,W      Magnetic Variation
 *   *6A          The checksum data, always begins with '*'
 * 07/01/15 dkh convert directly from ascii to decimal
 * $GPRMC,123519.000,A,4807.0380,N,01131.0000,E,022.4,084.4,230294,,,A*6A
 ******************************************************************************/
char _parseRMC(char          *msgBody,
               GpsData_t     *GPSData)
{
	char field[NMEA_MSG_MAX_FIELD];
	char status     = 0;
    int  parseReset = 1;

	/// Date field 8
	if( extractNMEAfield(msgBody, field, 8, parseReset) )	{
		GPSData->GPSday   = ( (field[0] - '0') * 10) + field[1] - '0'; /// day
		GPSData->GPSmonth = ( (field[2] - '0') * 10) + field[3] - '0'; /// month
		GPSData->GPSyear  = ( (field[4] - '0') * 10) + field[5] - '0'; /// year
	} else
        status = 1;
	return status;

}
/** ****************************************************************************
 * @name: computeNMEAChecksum compute the checksum of a NMEA  message.
 * @author  Dong An
 * @param [in] msgBody - data to parse
 * @param [in] GPSData - data structure to parst he data into
 * @retval status
 ******************************************************************************/
char computeNMEAChecksum(char         *NMEAMsg,
                         unsigned int *lengthBeforeStar)
{
	unsigned int  i;
	char msgChecksumComputed = 0;

	for (i = 1; i < (*lengthBeforeStar); i++) {
//		if(NMEAMsg[i] == ','){
//			continue;
//		}
		msgChecksumComputed ^= NMEAMsg[i]; ///skip '$'
	}
	return msgChecksumComputed;
}

/** ****************************************************************************
 * @name: _NMEA2UbloxAndLLA convert NMEA LLH into ublox LLH format.
 * @author  Dong An
 * @param [in] GPSData - GPS data strutcture to process
 * @retval N/A
 * @details called in driverGPSAllEntrance.cpp INS380 uses the decimal
 * GPSData->lat and GPSData->lon
 * NMEA format is dddmm.mmmmmm d = degrees, m = minutes
 * lat / lon = dd.dddddddd
 * LatLonH = lat or lon * 1000000 (decimal shift to unsigned integer)
 ******************************************************************************/
void _NMEA2UbloxAndLLA(NmeaLatLonSTRUCT *NmeaData,
                       GpsData_t        *GPSData)
{
    long double tmp1;
    long double tmp2;
    long double tmp3;

    // Longitude
    tmp1 = (long double) NmeaData->lon_deg;
    tmp2 = (long double) NmeaData->lon_min;
    tmp3 = (long double) NmeaData->lon_min_fraction;
    GPSData->lon = (tmp1 + (tmp2 + tmp3) / (long double)(60.0));
    if (NmeaData->lonSign == -1)
    {
        GPSData->lon = -GPSData->lon;
    }
//    GPSData->LonLatH[0] = (signed long)(GPSData->lon * 1.e7);

    // Latitude
    tmp1 = (long double) NmeaData->lat_deg;
    tmp2 = (long double) NmeaData->lat_min;
    tmp3 = (long double) NmeaData->lat_min_fraction;

    GPSData->lat = (tmp1 + (tmp2 + tmp3) / (long double)(60.0));
    if (NmeaData->latSign == -1)
    {
        GPSData->lat = -GPSData->lat;
    }
//    GPSData->LonLatH[1] = (signed long)(GPSData->lat * 1.e7);
    // Height (note: inconsistency in this equation based on other equations (should be 100 not 1000)
//    GPSData->LonLatH[2] = (signed long)(GPSData->alt * 1000.0);
}

/** ****************************************************************************
 * @name: convertNorhEastVelocity API convert ground speed into NED velocity.
 * @brief note: down velocity is synthesized from delta alt
 * @author  Dong An
 * @param [in] GPSData - GPS data strutcture to process
 * @retval status
 ******************************************************************************/
void convertNorhEastVelocity(GpsData_t* GPSData)
{
	GPSData->vNed[0] = GPSData->rawGroundSpeed * cos(D2R * GPSData->trueCourse);
	GPSData->vNed[1] = GPSData->rawGroundSpeed * sin(D2R * GPSData->trueCourse);

	_smoothVerticalData(GPSData); // synthesize and filter down velocity
}

/** ****************************************************************************
 * @name processGPSVerticalData SYNTHESIZE and filter vertical velocity from alt
 * @brief perform differential of altitude to obtain
 *        vertical velocity when using NME along with filtering of the data.
 * @author Dong An
 * @param [in] GPSData - GPS data
 * @retval status
 * @details EMA with a threshold to "filter" (omit) any value over the limit
 ******************************************************************************/
void _smoothVerticalData(GpsData_t* GPSData)
{
	static unsigned char firstFlag      = 0;
	static unsigned char firstFlagVd    = 0;
	static unsigned int  OutAltCounter  = 0;
	static unsigned int  OutVelCounter  = 0;
	static float         filteredAlt;
	static float         filteredVd;
    static double        LastTotalTime  = 0.0;

    double               TotalTime      = 0.0;
    float                temp1; // used for both alt and vel
	double               tmpDouble;
	float                lastAlt    = filteredAlt;
	float                CurrentAlt = GPSData->alt;
	float                CurrentVd;
	double               delta_T;

	if (firstFlag == 0) {
		firstFlag   = 1;
		filteredAlt = CurrentAlt;
		filteredVd  = 0;
		firstFlagVd = 0;
	} else {
		temp1 = CurrentAlt - filteredAlt;
		if( fabs( temp1 ) > MAX_ALTI_JUMP ) { // 10.0 [m]
			temp1 = 0.0 ;
			OutAltCounter++;
		} else {
            OutAltCounter = 0;
        }
		if ( OutAltCounter >= 5 ) {
			firstFlag            = 0;  // reset
			GPSData->vNed[2]     = filteredVd ;
			return;
		}

		filteredAlt = filteredAlt + temp1 * ALT_FILTER_GAIN; // gain 0.2 [m]

        TotalTime = GPSData->GPSHour * 3600.0 + GPSData->GPSMinute * 60.0 +
			        GPSData->GPSSecond + GPSData->GPSSecondFraction;
		delta_T = TotalTime - LastTotalTime;

		tmpDouble = filteredAlt - lastAlt;
		if ( delta_T < MIN_DELTA_T) { // MIN 1.e-5 [s]
            CurrentVd = filteredVd;
		} else {
			if (delta_T < MAX_DELTA_T) { // MAX 2.0 [s]
//				CurrentVd = tmpDouble - tmpDouble / delta_T; ///down is positive
				CurrentVd = -(tmpDouble / delta_T); ///down is positive
			} else { /// may be NMEA is lost
				firstFlag            = 0;  /// reset
				GPSData->vNed[2]     = filteredVd ;
				return;
			}
		} ///end of enough delta T

		if( firstFlagVd == 0) {
			firstFlagVd = 1;
			filteredVd  = CurrentVd;
		} else {
	 		temp1 = CurrentVd - filteredVd;

			if( temp1 > MAX_VEL_JUMP) { // 5.0 [m/s]
				temp1 = 0.0 ;
				OutVelCounter++;
			} else
                OutVelCounter = 0;

			if (OutVelCounter >= 5) {
				firstFlag            = 0;  /// reset
				GPSData->vNed[2]     = filteredVd ;
				return;
			}
            // do you need to filter the velcity? It is derived from altitude which is filtered
			filteredVd = filteredVd + temp1 * VD_FILTER_GAIN; // gain 0.0476 [m/s]
		}
	} /// end of not first

    // save the current time local to calc delta T next time
    LastTotalTime = TotalTime;

	/// output the vel
	GPSData->vNed[2]     = filteredVd;
}

/** ****************************************************************************
 * @name convertITOW convert GPS NMEA's UTC time tag into pseudo GPS ITOW number.
 * @author Dong An
 * @param [in] GPSData - GPS data
 * @retval status
 ******************************************************************************/
void convertItow(GpsData_t* GPSData)
{
	double tmp;
    // calculate day of week
    int tow  = dayofweek(GPSData->GPSday, GPSData->GPSmonth, GPSData->GPSyear+2000);
    // calculate second of week
    tmp = (double)tow * 86400.0;
	tmp += ((double)GPSData->GPSHour) * 3600.0 + ((double)GPSData->GPSMinute) * 60.0;
	tmp += (double)GPSData->GPSSecond + (double)GPSData->GPSSecondFraction;

    /// converting UTC to GPS time is impossible without GPS satellite
    /// Navigation message. using UTC time directly by scaling to ms.
	GPSData->itow = (unsigned long) (tmp * 1000);
}

int dayofweek(int day, int month, int year) 
{

	int adjustment, mm, yy;
 
	adjustment = (14 - month) / 12;
	mm = month + 12 * adjustment - 2;
	yy = year - adjustment;

	return (day + (13 * mm - 1) / 5 +
		yy + yy / 4 - yy / 100 + yy / 400) % 7;
}

#endif // GPS
