/** ***************************************************************************
 * @file driverGPSAllentrance.c GPS Driver for Internal/GPS NAV.
 * @author Dong An
 * @date   2009-04-10 23:20:59Z
 * @ver 8719
 * @copyright (c) 2013, 2014 All Rights Reserved.
 * @section LICENSE
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 * @details
 *  This module provides high level GPS management,
 *  based on product type and GPS receiver. This module
 *  is interfaced with NAV processing and other specific GPS
 *  process files. The functions are provided in this files are common
 *  for all GPS protocol. Protocol-specific process is provided in other GPS files
 * @version
 *  12.2005	DA	initial creation
 *	10.2006	DA  revised based on the GPS process code for ublox in UCB beta28
 *				and NAV420_08_beta4-7E.
 *				The updates include naming conventions, new functions, etc.
 *  03.2007 DA  Cleaned up, Doxygenized, and finalized for NAV440 release.
 *  01.2015 DH  Only SiRF, external NMEA, NoVatel for 380 INS
 *****************************************************************************/
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

#include "stm32f2xx_usart.h"
#include "uart.h"    // for uart_IrqOnOff()
#include "dmu.h"

#include "driverGPS.h"
#include "timer.h"
#include "extern_port_config.h"

//#define LOGGING_LEVEL LEVEL_DEBUG
#define LOGGING_LEVEL LEVEL_STREAM
#include "debug.h"
#include "debug_usart.h"
#include "gps.h" // the streaming flag
#include "calibrate.h"
#include "configureGPIO.h" // IO3 (B11) debug timing pin
#include "xbowsp_init.h"  // for gConfiguration in UseExtUSARTGps todo tm20160527 - do better than to need gConfiguration here

/// GPS data struct
// to change to NMEA then pass GPS out debug port: un-comment this and
// set LOGGING_LEVEL to LEVEL_STREAM. Nugget must have GPS config file loaded
// output stream is at 115200 baud even though the internal gps may be at 4800
#define STREAM_GPS GPS_NMEA_DEBUG_STREAM // [1]
GpsData_t gGpsData;
GpsData_t *gGpsDataPtr;

extern port_struct* gGpsUartPtr;

float getGpsHdop() {return gGpsDataPtr->HDOP;}

// local functions
void    _configGPSReceiver(int *bytesFromBuffer, GpsData_t* GPSData);
void    _parseGPSMsg(int *numInBuffPrt, GpsData_t *GPSData);
int16_t _getIndexLineFeed(uint16_t numInBuff, unsigned *msgLength);
void    _setGPSMessageSignature(GpsData_t* GPSData);
void    _loadEEPROMgpsCOMMSettings(GpsData_t* GPSData);
void    _userCmdBaudProtcol(GpsData_t* GPSData);

/** ****************************************************************************
 * @name initGPSHandler initializes data GPS structure using all configuration
 *       structure that is read from EEPROM.
 * @athor Dong An
 * @param [in] baudRate - enumeration to translate
 * @retval actual baud rate value
 ******************************************************************************/
void initGPSHandler(void)
{
	gGpsDataPtr = &gGpsData;

    memset((void*)&gGpsData, 0,  sizeof(GpsData_t));
    gGpsDataPtr->HDOP = 50.0f;

	gGpsDataPtr->GPSTopLevelConfig |= (1 << HZ2); // update to change internal (ublox) GPS to 2Hz
    /// Configure GPS structure, from Flash (EEPROM)
	_loadEEPROMgpsCOMMSettings(gGpsDataPtr);
    if ( IsInternalGPS() == true ) {
        initGpsUart( BAUD_4800 ); // default for the INTERNAL Sirf driverGPS.cpp
    } else { // external GPS
        // sets the circular buffer to gGpsData
        InitDebugSerialCommunication( baudEnumToBaudRate(gConfiguration.baudRateGPS) ); // use the USART for EXTERNAL
        gGpsData.GPSbaudRate        = gConfiguration.baudRateGPS;
        gGpsData.GPSProtocol        = (enumGPSProtocol)gConfiguration.protocolGPS;
    }
}

/** ****************************************************************************
 * @name GPSHandler GPS stream data and return GPS data for NAV algorithms
 * @athor Dong An
 * @retval N/A
 * @brief The _configGPSReceiver() has a 2 second delay in it that causes a
 * DAQ restart in the data acquisition task
 ******************************************************************************/
void GPSHandler(void)
{
	int        bytesInBuffer;
    static int baud_enum;

   	if( !IsInternalGPS() ) { // external can change
        if (baud_enum != gGpsDataPtr->GPSbaudRate) { // don't call unless chaged
            baud_enum = gGpsDataPtr->GPSbaudRate;
            _userCmdBaudProtcol(gGpsDataPtr);
        }
    }

	/// new data in circular buffer?
	bytesInBuffer               = gpsBytesAvailable(); // driverGPS.c
	gGpsDataPtr->Timer100Hz10ms = TimeNow(); ///< get system clock ticks
    gGpsDataPtr->isGPSBaudrateKnown = 1;

    // parse moved to top since is always called after init
  	if (gGpsDataPtr->GPSConfigureOK > 0 ) { /// Configuration is completed
    	_parseGPSMsg((int*)&bytesInBuffer,
                           gGpsDataPtr); // load GPSdata structure
    } else { ///< configure GPS receiver if needed: OK < 0
		_configGPSReceiver((int*)&bytesInBuffer,
                           gGpsDataPtr);
    }
}

// only used by internal (Origin) GPS to sequence through configurations
const struct { int baudRate; enum enumGPSProtocol protocol; } options[] = {
             { BAUD_4800,    NMEA_TEXT},   // most likely: cold boot
             { BAUD_230400,  SIRF_BINARY}, // simple reset
             { BAUD_4800,    SIRF_BINARY}, // shouldn't happen but there you go
             { BAUD_115200,  NMEA_TEXT} }; // external GPS

/** ****************************************************************************
 * @name _configGPSReceiver LOCAL configure GPS parses GPS stream data and fills
 *       GPS data into NAV filter structure.
 * @brief All GPS receiver configuration commands should be sent here if needed.
 * @author Dong An
 * @param [out] bytesFromBuffer - data from GPS data buffer
 * @param [out] GPSData - gps data structure
 * @retval N/A
 ******************************************************************************/
void _configGPSReceiver(int           *bytesFromBuffer,
                        GpsData_t     *GPSData)
{
	static enum enumGPSProtocol searchProtocol = INIT_SEARCH_PROTOCOL; ///< SiRF
	static unsigned char        enterFlag      = 0;
	static unsigned long        enterTime      = 0;

    _parseGPSMsg(bytesFromBuffer, GPSData); // load GPSdata structure

	if ( IsInternalGPS() ) { ///< 0 - internal GPS receiver is Origin SiRF binary or NMEA
        gAlgorithm.bitStatus.comStatus.bit.noExternalGPS = 1; // BIT status
        if ( GPSData->isGPSBaudrateKnown == 0) {
            // 2 second timeout can cause a DAQ restart in the data acquisiton task
            // if there is no GPS antena connected
            if ( (enterFlag == 0) || (( GPSData->Timer100Hz10ms - enterTime ) > MAX_PROTOCOL_SEARCHTIME)) {
                enterFlag = 1;
                DEBUG_INT("GPS trying ", GPSData->reconfigGPSCounter);
                DEBUG_ENDLINE();

                enterTime                   = GPSData->Timer100Hz10ms;

                GPSData->GPSbaudRate        = options[GPSData->reconfigGPSCounter].baudRate;
                GPSData->GPSProtocol        = options[GPSData->reconfigGPSCounter].protocol;

                initGpsUart(GPSData->GPSbaudRate);
#if STREAM_GPS == GPS_NMEA_DEBUG_STREAM
                gGpsDataPtr->GPSConfigureOK = 1; // force to NMEA
#endif
                GPSData->isGPSBaudrateKnown = 0;
                _setGPSMessageSignature(GPSData); // uses GPSProtocol to set parameters

                GPSData->reconfigGPSCounter++;
                GPSData->reconfigGPSCounter %= 4; // keep cycling through options table
            }
        } else { /// baud rate known - lock in the values
            DEBUG_STRING("GPS locking SiRF configuration\r\n");
            // switch from 4800, NMEA to 115,200, SiRf binary
            //configSiRFGPSReceiver(GPSData, BAUD_115200); // set to binary

            configSiRFGPSReceiver(GPSData, BAUD_230400);
			_setGPSMessageSignature(GPSData); // change signature to SiRf Binary
            GPSData->GPSConfigureOK = 1;
        }
	} else {  /// External GPS receiver
		GPSData->GPSTopLevelConfig |= 1 << EXTERNAL_GPS; // UBLOX
        gAlgorithm.bitStatus.comStatus.bit.noExternalGPS = 1; // BIT status
        /// unknown protocol
        if(( GPSData->GPSAUTOSetting & AUTOPROTOCOL) == AUTOPROTOCOL ) { // 2
            if (( GPSData->GPSAUTOSetting & AUTOBAUD)!= AUTOBAUD) {///< 1 baud rate known
                if(enterFlag == 0) {
                    InitDebugSerialCommunication( 4800 ); // external uses debug USART

                    GPSData->GPSProtocol        = searchProtocol;
                    _setGPSMessageSignature(GPSData);
                    enterTime                   = GPSData->Timer100Hz10ms;
                    GPSData->isGPSBaudrateKnown = 0;
                    enterFlag                   = 1;
                }
                if ( GPSData->isGPSBaudrateKnown == 1 )
                    GPSData->GPSConfigureOK = 1;
                if( GPSData->GPSConfigureOK != 1 && (GPSData->Timer100Hz10ms - enterTime) > MAX_PROTOCOL_SEARCHTIME) {
                    searchProtocol = (enumGPSProtocol)((int)searchProtocol - 1);
                    if (searchProtocol < AUTODETECT ) {
                        searchProtocol = DEFAULT_SEARCH_PROTOCOL;
                    }
                    enterFlag = 0;   ///< next cycle
                }
            } else { ///< baudrate unknown either
                if(enterFlag==0) {
                    GPSData->GPSProtocol     = searchProtocol;
                    _setGPSMessageSignature(GPSData);
                    GPSData->resetAutoBaud   = 0;
                    GPSData->autoBaudCounter = 0;
                    enterFlag                = 1;
                }
                GPSData->GPSConfigureOK = autobaud(GPSData);
                if(GPSData->GPSConfigureOK != 1 && GPSData->autoBaudCounter > 4) {
                    searchProtocol = (enumGPSProtocol)((int)searchProtocol - 1);
                    if (searchProtocol == AUTODETECT) {
                        searchProtocol = DEFAULT_SEARCH_PROTOCOL;
                    }
                    enterFlag = 0; // next cycle
                }
            }
        } else if (( GPSData->GPSAUTOSetting & AUTOBAUD) == AUTOBAUD) {
            /// know protocol but unknown baudrate
            GPSData->GPSConfigureOK = autobaud(GPSData);
        } else { /// know both protocol and baudrate
            _setGPSMessageSignature(GPSData);
            InitDebugSerialCommunication( baudEnumToBaudRate(GPSData->GPSbaudRate) ); // external uses debug USART
            GPSData->GPSConfigureOK = 1;
        }
    } ///end of "with an external GPS receiver"
}

/** ****************************************************************************
 * @name _parseGPSMsg LOCAL grab and sort GPS messages from GPS circular buffer
 *       into the GPS data structure
 * @author Dong An
 * @param [in] numInBuff - number of bytes in buffer
 * @param [out] GPSData - gps data structure
 * @retval N/A
 ******************************************************************************/
void _parseGPSMsg(int           *numInBuf,
                  GpsData_t     *GPSData)
{
	char            gpsMsg[MAX_MSG_LENGTH];
	unsigned int    msgLength = 0;
	uint32_t        bytesInCircularBuf = *numInBuf;
	static uint32_t startPacketTime = 0;

    while ( bytesInCircularBuf >= GPSData->GPSMsgSignature.GPSheaderLength ) {
        // pop bytes until header word found
        bytesInCircularBuf = findHeader(bytesInCircularBuf,
                                        GPSData);
		if( bytesInCircularBuf > 1 ) {
			if( startPacketTime == 0 )
              startPacketTime = TimeNow();
			else if(( TimeNow() - startPacketTime ) > GPS_PACKET_RECEIVE_TIMEOUT || // 0.5 sec
				      bytesInCircularBuf >= GPS_INTERFACE_RX_BUFFER_SIZE) { // 512
                // timeout or overflow - flush buffer
				startPacketTime        = 0;
				bytesInCircularBuf = delBytesGpsBuf( GPSData->GPSMsgSignature.GPSheaderLength );
				continue;
			}
		}

        // more data is in buffer than just the header
		if( bytesInCircularBuf >= GPSData->GPSMsgSignature.lengthOfHeaderIDLength )	{
            /// determine msg length which is protocol-specific
            switch ( GPSData->GPSProtocol ) {
                case NMEA_TEXT :
                    // peek (check) until find 'lf' 0x0a, msgLength = bytes to \lf
                    if ( !_getIndexLineFeed(bytesInCircularBuf, &msgLength)) {
                        return; // 'lf' not found yet in buffer
                    }
                    break;
                case SIRF_BINARY :
                    msgLength = peekWordGpsBuf(GPSData->GPSMsgSignature.lengthOfHeaderIDLength - 2);
                    msgLength += (GPSData->GPSMsgSignature.lengthOfHeaderIDLength + // + 4
                                  GPSData->GPSMsgSignature.crcLength); // + 2
                    break;
                case NOVATEL_BINARY :
                    // length of the body of the msg + length of the header
                    msgLength = peekByteGpsBuf(8) + peekByteGpsBuf(3);
                    if( bytesInCircularBuf >= msgLength ) { /// length of the body of the msg
                        msgLength += GPSData->GPSMsgSignature.crcLength; // + length of the CRC [4]
                    } else
                        return;
                    break;
                default:
                    break;

            }
            /// pop out rest of msg if it is complete
            if(	bytesInCircularBuf >= msgLength) {
                bytesInCircularBuf = retrieveGpsMsg(msgLength,
											        GPSData,
												    (uint8_t*)&gpsMsg[0]);
//STREAM_STRING(gpsMsg); // send raw message out

                if (bytesInCircularBuf > msgLength) { // more than 1 msg left in buffer
                    flushGPSRecBuf();
                    uart_IrqOnOff(1, USART_IT_RXNE, DISABLE );
                    DelayMs(600);
                    uart_IrqOnOff(1, USART_IT_RXNE, ENABLE );
                }
                startPacketTime = 0;

                switch (GPSData->GPSProtocol) { // have a complete message
                    case NMEA_TEXT :
                        processNMEAMessage(gpsMsg, &msgLength, GPSData);
                        break;
                    case SIRF_BINARY :
                        processSiRFBinaryMessage(gpsMsg, &msgLength, GPSData);
                        break;
                    case NOVATEL_BINARY : // pass in the body of the message
//                        processNovAtelBinaryMsg(&gpsMsg[0], &msgLength, GPSData);
                        processNovAtelBinaryMsg_Fast(&gpsMsg[0], &msgLength, GPSData);
                        break;
                    default:
                        break;
                }
                memset(gpsMsg, 0, sizeof( gpsMsg ) );
                continue; ///if return: one message picked up per one time call
            } else
                return;
		} else ///end of "if(bytesInCircularBuf >= GPSData->GPSMsgSignature.lengthOfHeaderIDLength)"
            return;
	} ///end of while ()
}

/** ****************************************************************************
 * @name _getIndexLineFeed LOCAL Find length of NMEA message
 *        Searches for 0x0A 'LF' Line Feed char in SCIB (Serial Circular Input
 *        Buffer).
 * @author Dong An
 * @param [in] numInBuff - number of items in circlar buffer
 * @param [out] msgLength - length (index to 'lf') of the message buffer
 * @retval 1 - found 'LF' 0 - didn't find it yet
 * 12/01/15 dkh switched to a do loop
 ******************************************************************************/
int16_t _getIndexLineFeed(uint16_t numInBuff,
                          unsigned *msgLength)
{
	uint16_t index = 0;

    do{
		if( 0x0A == peekByteGpsBuf(index)) { // search for 'lf' 0x0A
			*msgLength = index + 1;
			return  1; // success [1] found 'lf'
		}
	} while (index++ < numInBuff);

	return 0; // fail [0] didn't find 'lf'
}

/** ****************************************************************************
 * @name autobaud autobaud GPS message.
 * @author Dong An
 * @param [in] GPSData - GPS data
 * @retval status
 ******************************************************************************/
unsigned char autobaud(GpsData_t* GPSData)
{
	static unsigned long lastSentoutTime = 0;
	long                 deltaFromLastSendout;
	unsigned char        doneFlag = 0;

	if(GPSData->resetAutoBaud == 0) {
		GPSData->GPSbaudRate = INIT_SEARCH_BAUD;
        InitDebugSerialCommunication( baudEnumToBaudRate(GPSData->GPSbaudRate) ); // external uses debug USART

		flushGPSRecBuf();
		GPSData->isGPSBaudrateKnown = 0;

		if ( IsInternalGPS() )  /// for internal receiver only
			pollSiRFVersionMsg();
		GPSData->resetAutoBaud = 1;
		lastSentoutTime        = GPSData->Timer100Hz10ms;
	} else {
		deltaFromLastSendout  = GPSData->Timer100Hz10ms - lastSentoutTime;

	    if ( GPSData->isGPSBaudrateKnown == 0 ) {
	    	if( deltaFromLastSendout > QUERY_TIMEOUT ) { /// try again
				GPSData->GPSbaudRate--;
				if (GPSData->GPSbaudRate < 0) /// wrap aropund to highest baudrate
					GPSData->GPSbaudRate = INIT_SEARCH_BAUD;

                InitDebugSerialCommunication( baudEnumToBaudRate(GPSData->GPSbaudRate) ); // external uses debug USART

				flushGPSRecBuf();
				GPSData->isGPSBaudrateKnown = 0;
				if ( IsInternalGPS() )
        			pollSiRFVersionMsg();
				lastSentoutTime = GPSData->Timer100Hz10ms;
				GPSData->autoBaudCounter++;
			}/// end of timeout and try again
		} else { // end of non-version received
            GPSData->resetAutoBaud = 0;
            doneFlag               = 1;
		}
	}/// end of non-first
	return doneFlag;
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

/** ****************************************************************************
 * @name _loadEEPROMgpsCOMMSettings LOCAL load GPS baudrate and protocol from
 *       EEPROM or user Comm into GPS struct.
 * @author Dong An
 * @param [in] GPSData - GPS data
 * @retval status
 ******************************************************************************/
void _loadEEPROMgpsCOMMSettings(GpsData_t* GPSData)
{

	if ( IsInternalGPS() )   {  // internal GPS (hasGps)
        GPSData->GPSProtocol = NMEA_TEXT ; // default: if Binary, changes later in init
        SetConfigurationBaudRateGps(BAUD_4800); // system_init.c
        SetConfigurationProtocolGPS( GPSData->GPSProtocol ); // system_init.c
	} else { // external GPS
		/// check baud and protocol bounds
		if ( ConfigurationBaudRateGps() < BAUD_UNDEFINED || // -1
             ConfigurationBaudRateGps() > BAUD_115200) // 5
			SetConfigurationBaudRateGps( BAUD_UNDEFINED );

		if ( ConfigurationProtocolGPS() < AUTODETECT || // -1
             ConfigurationProtocolGPS() > NMEA_TEXT) // 3
			SetConfigurationProtocolGPS( AUTODETECT ); // -1

		GPSData->GPSAUTOSetting = 0;

        // External GPS using debug usart and hasGps [false] - internal disabled
        if ( UseExtUSARTGps() == true ) {
            // default - this gets changed later
            GPSData->GPSProtocol = NMEA_TEXT;
            GPSData->GPSbaudRate = BAUD_115200;
        } else {
            if( ConfigurationBaudRateGps() == AUTOBAUD) { // 1(BAUD_9600)
                GPSData->GPSAUTOSetting |=  AUTOBAUD; // 1
                GPSData->GPSbaudRate    = INIT_SEARCH_BAUD; /// 3 (BAUD_57600)
            } else
                GPSData->GPSbaudRate = ConfigurationBaudRateGps();

            if( ConfigurationProtocolGPS() == AUTODETECT) { // -1
                GPSData->GPSAUTOSetting |=  AUTOPROTOCOL; // 2
                GPSData->GPSProtocol    = INIT_SEARCH_PROTOCOL; // 4 SiRF
            } else
                GPSData->GPSProtocol = ( enum enumGPSProtocol )ConfigurationProtocolGPS();
        }
	}
  	_setGPSMessageSignature(GPSData);
	GPSData->GPSConfigureOK = 0;/// skip -1
}

/** ****************************************************************************
 * @name _userCmdBaudProtcol LOCAL keep watching if the GPS baudrate or
 *       protocol has been changed by the user.
 * @author Dong An
 * @param [in] GPSData - GPS data
 * @retval status
 ******************************************************************************/
void _userCmdBaudProtcol(GpsData_t* GPSData)
{
	char restart = 0;

	if( ConfigurationBaudRateGps() == BAUD_UNDEFINED) { // -1
		if ((GPSData->GPSAUTOSetting & AUTOBAUD) != AUTOBAUD) // 1(BAUD_9600)
			restart = 1;		///user input -1 now
	} else {
		if ((GPSData->GPSbaudRate != ConfigurationBaudRateGps() ) &&
			( ConfigurationBaudRateGps() > BAUD_UNDEFINED &&  // -1
              ConfigurationBaudRateGps() <= BAUD_4800)) // 3 max
		    restart = 1;
	}

	if( ConfigurationProtocolGPS() == AUTODETECT) { // -1
		if ((GPSData->GPSAUTOSetting & AUTOPROTOCOL) != AUTOPROTOCOL) // 2
			restart = 1; /// user input -1 now
	} else {
		if ((GPSData->GPSProtocol != ConfigurationProtocolGPS()) &&
		   (ConfigurationProtocolGPS() > AUTODETECT && // -1
            ConfigurationProtocolGPS() <= BAUD_4800)) // 3 max
	        restart = 1;
	}

	if ( restart )
		_loadEEPROMgpsCOMMSettings( GPSData );
}

