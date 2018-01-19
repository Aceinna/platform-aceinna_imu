/** ***************************************************************************
 * @file processNMEAGPS.c
 * @author Dong An
 * @date   2009-04-10 23:20:59Z
 * @copyright (c) 2013, 2014 All Rights Reserved.
 * @section LICENSE
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 * @betails Processing for GPS NMEA protocol.
 *
 *  	This includes all specific processing for NMEA protocol.
 * @version
 * 10/2006  DA  Initially created as UCB version
 *              based on the GPS process code for NMEA in VG325 "-06_B."
 *              The updates include naming conventions and removal
 *              of VG325 specific code, etc.
 * 03.2007 DA  Cleaned up, Doxygenized, and finalized for NAV440 release.
 * 10.27.14 DH _parseVTG() for WMM data, added alt above ellipsiod for WMM
 *****************************************************************************/
#include <math.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

#include "dmu.h"
#include "driverGPS.h"
#include "filter.h"
#include "timer.h"
#include "xbowsp_init.h"

char _parseGPGGA(char *msgBody, GpsData_t* GPSData);
char _parseVTG(char *msgBody, GpsData_t* GPSData);
char _parseRMC(char *msgBody, GpsData_t* GPSData);
void _handleNMEAmsg(char *msgID, char *msgBody,GpsData_t* GPSData);
void _NMEA2UbloxAndLLA(NmeaLatLonSTRUCT* NmeaData, GpsData_t* GPSData);
void _smoothVerticalData(GpsData_t* GPSData);

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
    char         checksumRcvd;
    char         checksumCalc;
    unsigned int lengthBeforeStar;

	if (msg[*msgLength - 5]!= '*')
        return; /// no '*' delimiter before checksum
    // convert hex "digits" from ascii to int
	checksumRcvd =( (msg[*msgLength - 4] - '0') << 4) | (msg[*msgLength - 3] - '0');

	lengthBeforeStar = *msgLength - 5;
	checksumCalc     = computeNMEAChecksum(msg, &lengthBeforeStar);
	if (checksumCalc == checksumRcvd) {
        _handleNMEAmsg(msg,     // header "$GP..."
                       &msg[7], // payload
                       GPSData);
    }
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
    if( strncmp((char *)msgID, "$GPGGA", 6) == 0 ) {
		_parseGPGGA(msgBody, GPSData);
    }
    if( strncmp((char *)msgID, "$GPVTG", 6) == 0 ) {
		_parseVTG(msgBody, GPSData);
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
    // Leave the currentField as the one returned so the next search passes
    // the currrent doesn't match requested test
    // Back up one so the next search picks up the end ','
    searchIndex--;
	return 1;
}

/** ****************************************************************************
 * @name: _parseGPGGA LOCAL parse GPGGA message. Tiem, position, fix data
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
char _parseGPGGA(char          *msgBody,
                 GpsData_t *GPSData)
{
	char   field[NMEA_MSG_MAX_FIELD];
	char   status = 0;
    int    parseReset = true;
    NmeaLatLonSTRUCT nmeaLatLon;
    // this is the local using the NMEA convention
    char   GPSFix = 0; // no fix

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
            GPSData->latSign = -1;
		else
            GPSData->latSign = 1;
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
            GPSData->lonSign = -1;
		else
            GPSData->lonSign = 1;
	} else
        status = 1;

	/// GPS quality
	if( extractNMEAfield(msgBody, field, 5, parseReset) )	{
        GPSFix = field[0] - '0'; // convert ascii digit to decimal
	} else
        status = 1;

	/// HDOP x.x
	if( extractNMEAfield(msgBody, field, 7, parseReset) )	{
		GPSData->HDOP = (field[0] - '0') + (field[2] - '0') * 0.1f ;// convert ascii digit to decimal
	} else
        status = 1;

    if(GPSFix >= 1) {
        convertItow(GPSData); // create pseudo ITOW
        gAlgorithm.bitStatus.hwStatus.bit.unlockedInternalGPS = 0; // locked
        gAlgorithm.bitStatus.swStatus.bit.noGPSTrackReference = 0; // GPS track
    } else {
        gAlgorithm.bitStatus.hwStatus.bit.unlockedInternalGPS = 1; // no signal lock
        gAlgorithm.bitStatus.swStatus.bit.noGPSTrackReference = 1; // no GPS track
    }

    /// NMEA - DGPS bit
    if (GPSFix == 2 || GPSFix == 4 || GPSFix == 5){ /// on
        SetAlgorithmUseDgps(0);
	}
    else{
        SetAlgorithmUseDgps(1);
	}

	/// Altitude
	if( extractNMEAfield(msgBody, field, 8, parseReset) )	{
		GPSData->alt = atof((char *)field);
	} else{
        status = 1;
	}

    if(GPSFix >= 1) {
        _NMEA2UbloxAndLLA(&nmeaLatLon, GPSData);
    }
    // flip the sense from the NMEA convention to match the propiatery binary messages
    if (GPSData->HDOP < 20) { // the threshold may get set lower in algorithm.c
        GPSData->GPSFix = 0; // fix
    } else {
        GPSData->GPSFix = 1; // no fix
    }
	if( status == 0) {
		GPSData->updateFlagForEachCall |= 1 << GOT_GGA_MSG;
		GPSData->LLHCounter++;
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

	for (i = 0; i < (*lengthBeforeStar - 1); i++) {
		msgChecksumComputed ^= NMEAMsg[1 + i]; ///skip '$'
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
//    GPSData->LonLatH[0] = (signed long)(GPSData->lon * 1.e7);

    // Latitude
    tmp1 = (long double) NmeaData->lat_deg;
    tmp2 = (long double) NmeaData->lat_min;
    tmp3 = (long double) NmeaData->lat_min_fraction;

    GPSData->lat = (tmp1 + (tmp2 + tmp3) / (long double)(60.0));
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
	GPSData->vNed[0] = (GPSData->rawGroundSpeed * 0.2777777777778) *
					    cos(D2R * GPSData->trueCourse); // 0.277 = 1000/3600 kph -> m/s
	GPSData->vNed[1] = (GPSData->rawGroundSpeed * 0.2777777777778) *
						sin(D2R * GPSData->trueCourse);

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
			GPSData->filteredAlt = filteredAlt ;
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
				GPSData->filteredAlt = filteredAlt ;
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
				GPSData->filteredAlt = filteredAlt ;
				GPSData->vNed[2]     = filteredVd ;
				return;
			}
            // do you need to filter the velcity? It is derived from altitude which is filtered
			filteredVd = filteredVd + temp1 * VD_FILTER_GAIN; // gain 0.0476 [m/s]
		}
	} /// end of not first

    // save the current time local to calc delta T next time
    LastTotalTime = TotalTime;

	/// output the filtered alt and vel values
	GPSData->filteredAlt = filteredAlt;
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
	long double tmp;

	tmp = ((long double)GPSData->GPSHour) * 3600.0 + ((long double)GPSData->GPSMinute) * 60.0;
	tmp += (long double)GPSData->GPSSecond + (long double)GPSData->GPSSecondFraction;

    /// converting UTC to GPS time is impossible without GPS satellite
    /// Navigation message. using UTC time directly by scaling to ms.
	GPSData->itow = (unsigned long) (tmp * 1000);
}