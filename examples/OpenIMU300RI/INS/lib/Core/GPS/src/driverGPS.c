/** ***************************************************************************
 * @file driverGPS.c serial interface and buffer functions
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
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

#include <stdint.h>
#include <string.h>

#include "osapi.h"
#include "driverGPS.h"
#include "uart.h"
#include "gpsAPI.h"
#include "math.h"
#include "platformAPI.h"

#ifdef DISPLAY_DIAGNOSTIC_MSG
#include "debug.h"
#endif

int gpsSerialChan = UART_CHANNEL_NONE;  // undefined, needs to be explicitly defined 


/// GPS data struct
// to change to NMEA then pass GPS out debug port: un-comment this and
// set LOGGING_LEVEL to LEVEL_STREAM. Nugget must have GPS config file loaded
// output stream is at 115200 baud even though the internal gps may be at 4800
#define STREAM_GPS GPS_NMEA_DEBUG_STREAM // [1]
GpsData_t gGpsData = {
    .GPSbaudRate     = 4800,
    .GPSProtocol     = NMEA_TEXT,
    //.sirfInitialized = FALSE,
};

GpsData_t *gGpsDataPtr = &gGpsData;

// local functions
void    _configGPSReceiver(GpsData_t* GPSData);
void    _parseGPSMsg(int *numInBuffPrt, GpsData_t *GPSData);
int16_t _getIndexLineFeed(uint16_t numInBuff, unsigned *msgLength);
void    _setGPSMessageSignature(GpsData_t* GPSData);
void    _userCmdBaudProtcol(GpsData_t* GPSData);

void loadGpsCommSettings(GpsData_t* GPSData)
{
    GPSData->GPSAUTOSetting = 0;
  	_setGPSMessageSignature(GPSData);
	GPSData->GPSConfigureOK = 0;
}

// extern_port.c
extern void GetGpsExternUartChannel(unsigned int* uartChannel);

/** ****************************************************************************
 * @name: initGpsUart set up baudrate and GPS UART
 * @param [in] gGpsDataPtr - GPS control structure
 * @retval N/A
 ******************************************************************************/
int initGpsUart(int baudrate) 
{
    gpsSerialChan  = platformGetSerialChannel(GPS_SERIAL_PORT);
    return uart_init(gpsSerialChan, baudrate);
}


/** ****************************************************************************
 * @name: gpsBytesAvailable bytes in Gps circ buffer API only used in
 *        driverGPSAllEntrance
 * @param N/A
 * @retval number of bytes in Gps circ buffer
 ******************************************************************************/
int gpsBytesAvailable()
{
    return uart_rxBytesAvailable(gpsSerialChan);
}

/** ****************************************************************************
 * @name: flushGPSRecBuf empty GPS interface's software buffer.
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void flushGPSRecBuf(void)
{
    uart_flushRecBuffer(gpsSerialChan);
}

/** ****************************************************************************
 * @name isGpsTxEmpty check if transmit buffer is empty...
 * @param N/A
 * @retval N/A
 ******************************************************************************/
BOOL isGpsTxEmpty(void)
{
    return uart_txBytesRemains(gpsSerialChan) == 0;
}

/** ****************************************************************************
 * @name delBytesGpsBuf will pop bytes off in the receive FIFO.
 * @param [in] numToPop is the number of bytes to pop off
 * @retval the number of bytes remaining in the FIFO.
 ******************************************************************************/
uint16_t delBytesGpsBuf(uint16_t numToPop)
{
	int16_t numInBuffer;

    numInBuffer = uart_rxBytesAvailable(gpsSerialChan);
    if (numInBuffer < numToPop) {
        numToPop = numInBuffer;
    }
    uart_removeRxBytes(gpsSerialChan, numToPop);
//    COM_buf_delete(&(gGpsUartPtr->rec_buf),
//                   numToPop);
    return ( numInBuffer - numToPop ); ///< unscanned bytes
}

/** ****************************************************************************
 * @name peekWordGpsBuf will return a word without popping the FIFO.
 * @brief No error checking on the range of index is performed.
 * @param [in] index represents the byte offset into the FIFO to start looking at.
 * @retval  the 16-bit word in the FIFO starting at index offset from end.
 ******************************************************************************/
/*
uint16_t peekWordGpsBuf(uint16_t index)
{
	uint16_t word;

    word = peekByteGpsBuf(index) << 8 | peekByteGpsBuf(index + 1);
	return word;
}
*/
/** ****************************************************************************
 * @name peekByteGpsBuf peekByteSCIA will return a byte without popping
 *       the FIFO.
 * @brief No error checking on the range of index is performed.
 * @param [in] index represents the byte offset into the FIFO to look at.
 * @retval the byte in the FIFO pointed to by index.
 ******************************************************************************/
uint8_t peekByteGpsBuf(uint16_t index)
{
    uint8_t output = 0;

    uart_copyBytes(gpsSerialChan, index, 1, &output);

	return output;
}

/** ****************************************************************************
 * @name peekGPSmsgHeader search and return GPS message header
 * @brief No error checking on the range of index or start of sentence word is
 *        performed.
 * @param [in] index represents the byte offset into the FIFO to look at.
 * @param [in] GPSData - data structure containng the message data
 * @retval the message header.
 ******************************************************************************/
unsigned long peekGPSmsgHeader(uint16_t      index,
                               GpsData_t     *GPSData)
{
    uint8_t       header[MAX_HEADER_LEN];
    int           i;
    int           j;
    int           headerLength;
	unsigned long GPSHeader;

    headerLength = GPSData->GPSMsgSignature.GPSheaderLength;
    uart_copyBytes(gpsSerialChan, index, headerLength, header);
    GPSHeader = 0;

    for (i = 0, j = headerLength - 1; i < headerLength; i++, j--) {
        GPSHeader |= header[i] << (j * 8);
    }

	return GPSHeader;
}

/** ****************************************************************************
 * @name retrieveGpsMsg search for start of GPS message header
 * @brief COM_buf_get does a check for underflow the returns the message or
 *        nothing
 * @param [in] numBytes - to retrieve.
 * @param [in] GPSData - data structure containing the message information
 * @param [out] outBuffer - pointer to the output buffer
 * @retval The number of bytes left in the buffer. Or Zero - finished or
 *         buffer had exactly the number of bytes in the complete messsage
 ******************************************************************************/
int16_t retrieveGpsMsg(uint16_t  numBytes, GpsData_t *GPSData, uint8_t  *outBuffer)
{
	uart_read(gpsSerialChan, outBuffer, numBytes);
    return uart_rxBytesAvailable(gpsSerialChan);
}

/** ****************************************************************************
 * @name findHeader search for start of GPS message header
 * @brief Does a simple check for the start byte then peeks at enough bytes
 *        to compare with signature header.
 * @param [in] numInBuff -  bytes available in input buffer.
 * @param [in] GPSData - data structure containing the message information
 * @retval The number of bytes left in the buffer including the header. Can
 *         be less than the complete messsage
 ******************************************************************************/
int16_t findHeader(uint16_t      numInBuff,
                   GpsData_t     *GPSData)
{
    uint8_t  buf[MAX_HEADER_LEN];
	uint8_t  byte;
	uint32_t header = 0;
	volatile uint8_t  exit   = 0;
    int      num;

	do {
		uart_copyBytes(gpsSerialChan,0,1,&byte);

		if (byte == GPSData->GPSMsgSignature.startByte) {
		    uart_copyBytes(gpsSerialChan,1,GPSData->GPSMsgSignature.GPSheaderLength - 1, buf);
			header = byte <<  (GPSData->GPSMsgSignature.GPSheaderLength - 1) * 8;
			switch (GPSData->GPSMsgSignature.GPSheaderLength) {
				case 2: // SiRF 0xa0a2
					header |= (buf[0]);
					break;
				case 3: // NMEA "$GP", NovAtel binary 0xAA4412
					header |=  (buf[0] << 8) | (buf[1]);
					break;
			}
            if ( header == GPSData->GPSMsgSignature.GPSheader ) {
				exit = 1;
			} else {
				num = uart_removeRxBytes(gpsSerialChan, 1);
				if(num){
				numInBuff--;
			}
			}
		} else {
			num = uart_removeRxBytes(gpsSerialChan, 1);
			if(num){
			numInBuff--;
		}
		}
    } while ( (exit == 0) && (numInBuff > 0) );
	return numInBuff;
}

/** ****************************************************************************
 * @name writeGps send data to the GPS
 * @param [in] data - to go out
 * @param [in] len - length in bytes
 * @retval N/A
 * @details gConfiguration.userBehavior.bit.useGPS (use USART) determines
 *          which pipeline to send command through
 ******************************************************************************/
int writeGps(char  *data, uint16_t len)
{
    return uart_write(gpsSerialChan, (uint8_t*)data, len);

}

void GetGPSData(gpsDataStruct_t *data)
{
    data->gpsUpdate        =  ( gGpsDataPtr->updateFlagForEachCall >> GOT_VTG_MSG ) & 0x00000001 &&
                               ( gGpsDataPtr->updateFlagForEachCall >> GOT_GGA_MSG ) & 0x00000001;
    // gGpsDataPtr->updateFlagForEachCall &= 0xFFFFFFFD;
    if(data->gpsUpdate)
    {
        // Reset GPS update flag only when pos and vel are both available
        gGpsDataPtr->updateFlagForEachCall &= 0xFFFFFFFC;

        data->gpsFixType        = gGpsDataPtr->gpsFixType;
        data->numSatellites     = gGpsDataPtr->numSatellites;

        data->latitude          = gGpsDataPtr->lat;
        data->longitude         = gGpsDataPtr->lon;
        data->altitude          = gGpsDataPtr->alt;

        data->vNed[0]           = gGpsDataPtr->vNed[0];
        data->vNed[1]           = gGpsDataPtr->vNed[1];
        data->vNed[2]           = gGpsDataPtr->vNed[2];

        data->trueCourse        = gGpsDataPtr->trueCourse;
        data->rawGroundSpeed    = gGpsDataPtr->rawGroundSpeed;

        data->GPSSecondFraction = gGpsDataPtr->GPSSecondFraction; 

        data->itow              = gGpsDataPtr->itow;       
        data->GPSmonth          = gGpsDataPtr->GPSmonth;
        data->GPSday            = gGpsDataPtr->GPSday;
        data->GPSyear           = gGpsDataPtr->GPSyear;  
        data->GPSHour           = gGpsDataPtr->GPSHour;
        data->GPSMinute         = gGpsDataPtr->GPSMinute; 
        data->GPSSecond         = gGpsDataPtr->GPSSecond; 

        data->HDOP              = gGpsDataPtr->HDOP;
        data->GPSHorizAcc       = gGpsDataPtr->GPSHorizAcc;
        data->GPSVertAcc        = gGpsDataPtr->GPSVertAcc;
        data->geoidAboveEllipsoid = gGpsDataPtr->geoidAboveEllipsoid;
    }
}


/** ****************************************************************************
 * @name: avgDeltaSmoother API routine using data structure to hold instance data
 *        for smoothing gps using a 3 item moving average of the change in values
  *       (deltas) that removes data "spikes" over 6 times the average
 * @author
 * @param [in]  in - new value to add to the average
 * @param [in]  data - filter data for lat, lon or alt
 * @retval last value if > than 6x avg or pass the value back out
 ******************************************************************************/
double avgDeltaSmoother( double  rawData,    gpsDeltaStruct *delta)
{
    double thisDelta;
    double returnVal;

    thisDelta = delta->last - rawData;

    delta->sum -= delta->oldValues[2]; // pop the old value off of the sum
    delta->sum += thisDelta;           // push the new value in the sum

    // push the data down the hold stack
    delta->oldValues[2] = delta->oldValues[1];
	delta->oldValues[1] = delta->oldValues[0];
	delta->oldValues[0] = thisDelta;
    if ( fabs(thisDelta) > 6.0 * fabs( delta->sum / 3.0 ) ) {
        returnVal = delta->last; // filter (omit) the value
    } else
        returnVal = rawData; // send the input value back out

    delta->last = rawData;             // hold input value for next entry
    return returnVal;
}

/******************************************************************************
 * @name: thresholdSmoother API threshold filter that uses total speed from Vned
 *        for smoothing gps "glitches" (in delta speed) "spikes" over 15m/s
 * @author
 * @param [in]  in - new value to compare to threshold
 * @param [in]  data - return last good data or current good data
 * @retval N/A
 ******************************************************************************/
void thresholdSmoother( double         vNedIn[3],
                        float          vNedOut[3])
{
    double        speedSqCurr;
    double        SPEED_SQ_LIMIT = 225;  // limit the change to 15 m/s
    static double speedSqPast;
    static double vNedPast[3];

    // "glitch" filter
    // Compute the speed
    speedSqCurr = vNedIn[0]*vNedIn[0] + vNedIn[1]*vNedIn[1] + vNedIn[2]*vNedIn[2];

    // "Filter" the velocity signal if the delta is greater than 15 [ m/s ]
    //   (This is an 8% margin over expected system dynamics)
    if( fabs( speedSqCurr - speedSqPast ) > SPEED_SQ_LIMIT ) {
        // If a glitch is encountered, set output to last "good" value.
        //  Do not update past with current.
        vNedOut[0] = (float)vNedPast[0];
        vNedOut[1] = (float)vNedPast[1];
        vNedOut[2] = (float)vNedPast[2];
    } else {
        // Do not change GPS velocity, update past values with current values
        speedSqPast = speedSqCurr;
        vNedPast[0] = vNedIn[0];
        vNedPast[1] = vNedIn[1];
        vNedPast[2] = vNedIn[2];

        vNedOut[0] = (float)vNedIn[0];
        vNedOut[1] = (float)vNedIn[1];
        vNedOut[2] = (float)vNedIn[2];
    }
}

/** ****************************************************************************
 * @name initGPSHandler initializes data GPS structure using all configuration
 *       structure that is read from EEPROM.
 * @athor Dong An
 * @param [in] baudRate - enumeration to translate
 * @retval actual baud rate value
 ******************************************************************************/
void initGPSHandler(void)
{
 #ifdef GPS
    gGpsDataPtr->HDOP = 50.0f;
	gGpsDataPtr->GPSTopLevelConfig |= (1 << HZ2); // update to change internal (ublox) GPS to 2Hz
    /// Configure GPS structure, from Flash (EEPROM)
	loadGpsCommSettings(gGpsDataPtr);
    initGpsUart(gGpsDataPtr->GPSbaudRate);
#endif
}



/** ****************************************************************************
 * @name initGPSDataStruct initializes data GPS structure using all configuration
 *       structure that is read from EEPROM.
 * @athor Dong An
 * @param [in] baudRate - enumeration to translate
 * @retval actual baud rate value
 ******************************************************************************/
void initGPSDataStruct(void)
{
	gGpsDataPtr = &gGpsData;
    memset((void*)&gGpsData, 0,  sizeof(GpsData_t));
    gGpsData.GPSbaudRate = 4800;
    gGpsData.GPSProtocol = NMEA_TEXT;
}


BOOL _handleGpsMessages(GpsData_t *GPSData)
{
	static uint8_t gpsUartBuf[100]; 
    static uint8_t gpsMsg[MAX_MSG_LENGTH];
    static int bytesInBuffer = 0;
    unsigned char tmp;
	unsigned static int  pos = 0;
	
    
	while(1){
        if(!bytesInBuffer){
            bytesInBuffer = uart_read(gpsSerialChan, gpsUartBuf, sizeof (gpsUartBuf));
            if(!bytesInBuffer){
                return 0; // nothing to do
            }
            pos = 0; 
        }
        tmp = gpsUartBuf[pos++];
        bytesInBuffer--;
        switch(GPSData->GPSProtocol){
            case NMEA_TEXT: 
                parseNMEAMessage(tmp, gpsMsg, GPSData);
                break; 
            case NOVATEL_BINARY:
                parseNovotelBinaryMessage(tmp, gpsMsg, GPSData);
                break;
            case UBLOX_BINARY:
                parseUbloBinaryMessage(tmp, gpsMsg, GPSData);
                break;
            default:
                break;
                }
        }
}
/* end _handleGpsMessages */

/** ****************************************************************************
 * @name GPSHandler GPS stream data and return GPS data for NAV algorithms
 * @athor Dong An
 * @retval N/A
 * @brief The _configGPSReceiver() has a 2 second delay in it that causes a
 * DAQ restart in the data acquisition task
 ******************************************************************************/
void GPSHandler(void)
{
	gGpsDataPtr->Timer100Hz10ms     = getSystemTime();      ///< get system clock ticks
    gGpsDataPtr->isGPSBaudrateKnown = 1;

    // parse moved to top since is always called after init
  	if (gGpsDataPtr->GPSConfigureOK > 0 ) {              // Configuration is completed
        _handleGpsMessages(gGpsDataPtr);
    } else { ///< configure GPS receiver if needed: OK < 0
		_configGPSReceiver(gGpsDataPtr);
    }
}

/** ****************************************************************************
 * @name _configGPSReceiver LOCAL configure GPS parses GPS stream data and fills
 *       GPS data into NAV filter structure.
 * @brief All GPS receiver configuration commands should be sent here if needed.
 * @author Dong An
 * @param [out] bytesFromBuffer - data from GPS data buffer
 * @param [out] GPSData - gps data structure
 * @retval N/A
 ******************************************************************************/
void _configGPSReceiver(GpsData_t     *GPSData)
{

    _handleGpsMessages(GPSData); // load GPSdata structure

	if (GPSData->GPSProtocol == SIRF_BINARY) { ///< 0 - internal GPS receiver is Origin SiRF binary or NMEA
         configSiRFGPSReceiver(GPSData, 230400);
	    _setGPSMessageSignature(GPSData);
	} else { 
         // assuming that rate and protocol configured properly from the beginning
         GPSData->GPSConfigureOK = 1;
    }
}

/** ****************************************************************************
 * @name _setGPSMessageSignature LOCAL input GPS message spec (or feature) into
 *       GPS structure
 * @author Dong An
 * @param [in] GPSData - GPS data
 * @retval status
 ******************************************************************************/
void _setGPSMessageSignature(GpsData_t* GPSData)
{
	switch(GPSData->GPSProtocol) {
        case SIRF_BINARY :
			GPSData->GPSMsgSignature.GPSheader              = SIRF_BINARY_HEADER; // 0xa0a2
			GPSData->GPSMsgSignature.GPSheaderLength        = 2;
			GPSData->GPSMsgSignature.lengthOfHeaderIDLength = 4; // header len + 2 payload len
			GPSData->GPSMsgSignature.crcLength              = 2;
			GPSData->GPSMsgSignature.binaryOrAscii          = 0;
		break;
        case NOVATEL_BINARY :
			GPSData->GPSMsgSignature.GPSheader              = NOVATEL_OME4_BINARY_HEADER; // 0xAA4412
	        GPSData->GPSMsgSignature.GPSheaderLength        = 3;
			GPSData->GPSMsgSignature.lengthOfHeaderIDLength = 5;
			GPSData->GPSMsgSignature.binaryOrAscii          = 0;
			GPSData->GPSMsgSignature.crcLength              = 4;
		break;
		case NMEA_TEXT :
			GPSData->GPSMsgSignature.GPSheader              = NMEA_HEADER; // "$GP"
	        GPSData->GPSMsgSignature.GPSheaderLength        = 3;
			GPSData->GPSMsgSignature.binaryOrAscii          = 1;
			GPSData->GPSMsgSignature.lengthOfHeaderIDLength = 3;
		    break;
        default:
            break;
	}
    GPSData->GPSMsgSignature.startByte = GPSData->GPSMsgSignature.GPSheader >> ( (GPSData->GPSMsgSignature.GPSheaderLength * 8) - 8);
}

#endif // GPS


