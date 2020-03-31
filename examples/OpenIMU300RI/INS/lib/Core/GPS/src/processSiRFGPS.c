/** ***************************************************************************
 * @file processSiRFGPS.c Processing for SiRF binary GPS receiver.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 * @details This file includes all specific processing, including configuring,
 *   for SiRF GPS receiver.
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

#include <stdlib.h>

#include <math.h>
#define LOGGING_LEVEL LEVEL_DEBUG
#include "driverGPS.h"
#include "SiRFPacketFormats.h"
#include "utilities.h"
#include <string.h>
#include "BITStatus.h"
#include "osapi.h"

static gpsDeltaStruct downVelDelta;

void _siRFStaticNavMsg(uint8_t flag);

/** ****************************************************************************
 * @name _computeSirfChecksum LOCAL calculate a checksum for SiRF
 * @param [in] payload - input buffer
 * @param [in] payloadLen - input buffer length
 * @retval calculated checksum
 ******************************************************************************/
static uint16_t _computeSirfChecksum(uint8_t      *payload,
                                     unsigned int payloadLen)
{
    unsigned int      i;
    uint16_t checksum = 0;

    for ( i = 0; i < payloadLen; i++) {
        checksum += payload[i];
    }
    return checksum;
}

/** ****************************************************************************
 * @name reorderBytes used in SiRFPacketFormats.h
 * @param [in] in - input buffer
 * @param [in] index - byte array index
 * @param [in] numBytes - number of bytes to process
 * @retval word with bytes swapped
 * used in SET_VALUE() macro
 ******************************************************************************/
static uint32_t reorderBytes(char *in,
                             int  *index,
                             int  numBytes)
{
    uint32_t out = 0;
    int      i;

    in += *index;
    for ( i = 0 ; i < numBytes; i++ ) {
        out = (out << 8) + *in;
        in++;
    }
    *index += numBytes;
    return out;
}

/** ****************************************************************************
 * @name _sirfSwitchToBinary LOCAL configure SiRF GPS receiver to binary
 * @brief configure SiRF GPS receiver using a NMEA $PSRF100 to go faster and send
 *        binary packets
 *  Send message 100: SetSerialPort, Set PORT A parameters and protocol
 *  $PSRF100,
 *  0, where 0 = binary, 1 = NMEA
 *  9600, baud rate
 *  8, data bits
 *  1, stop bits
 *  0 parity (0=none, 1=odd, 2=even)
 *  *0C checksum
 *  \r\n end message
 * @author Dong An
 * @param [out] realBaudRate -
 * @retval N/A
 ******************************************************************************/
void _sirfSwitchToBinary(uint32_t realBaudRate)
{
    char         msg[GPS_INTERFACE_TX_BUFFER_SIZE];
    char         check;
    char         nibble;
    unsigned int msgIndex = 0;

    memset(msg, 0, GPS_INTERFACE_TX_BUFFER_SIZE);

    strcpy(msg, "$PSRF100");
    msgIndex = strlen(msg);

    /// SiRF binary mode
    strcpy(&(msg[msgIndex]), ",0,");
    msgIndex = strlen(msg);

    itoa(realBaudRate, &(msg[msgIndex]), 10);
    msgIndex = strlen(msg);

    /// SiRF protocol is only valid for 8 data bits, 1 stop bit, and no parity.
    strcpy(&(msg[msgIndex]), ",8,1,0");
    msgIndex = strlen(msg);

    check = computeNMEAChecksum(msg, &msgIndex);
    msg[msgIndex] = '*'; msgIndex++;
    /// short itoa for hex-ing the checksum
    nibble = (check >> 4) & 0xF;
    if (nibble < 10) {
        nibble += '0';
    } else {
        nibble += 'A' - 10;
    }

    msg[msgIndex] = nibble; msgIndex++;
    nibble = check & 0xF;
    if (nibble < 10) {
        nibble += '0';
    } else {
        nibble += 'A' - 10;
    }
    msg[msgIndex] = nibble;
    msgIndex++;

    strcpy(&(msg[msgIndex]), "\r\n");
    msgIndex += 2;
    writeGps(msg, msgIndex);
}

/** ****************************************************************************
 * @name configSiRFGPSReceiver API configure SiRF GPS receiver to go faster,
 *       send bin packets
 * @author Dong An
 * @param [out] GPSData - gps data structure
 * @param [out] goalBaudRate - data from GPS data buffer
 * @retval N/A
 ******************************************************************************/
void configSiRFGPSReceiver(GpsData_t * GPSData, int goalBaudRate)
{
/* 
    static int timeout = 5;     //

    if (!GPSData->sirfInitialized) {
        _sirfSwitchToBinary(goalBaudRate);
        GPSData->sirfInitialized = 1;
        timeout = 0;
        ///< wait for message to finish  DEBUG: Can get stuck here if SiRF not attached
        while ( !isGpsTxEmpty() ) {};   // spin
    OS_Delay(100);
        initGpsUart(goalBaudRate);
    OS_Delay(100); ///< clear incoming remaining packets
    flushGPSRecBuf();
    pollSiRFVersionMsg(); // send request for version - if (binary) version recieved set msg rate
    GPSData->GPSFix = 1;
    }else if (++timeout > 5) {  // 5 cycles of GPS task
        GPSData->sirfInitialized = 0;
        GPSData->GPSbaudRate     = 4800;
        while ( !isGpsTxEmpty() ) {};   // spin
        initGpsUart(GPSData->GPSbaudRate);  // initial GPS receiver baudrate
        OS_Delay(100);      ///< clear incoming remaining packets
        GPSData->GPSFix = 0;
    }
*/

}


/** ****************************************************************************
 * @name _SiRFSendMessage LOCAL format and send a message to SiRF
 * @param [in] msg - dat to xmit
 * @param [in] messageLength - length of send buffer
 * @retval byte swappeed word
 ******************************************************************************/
static void _SiRFSendMessage(uint8_t *msg,
                             int     messageLength)
{
    uint16_t tmp;

    tmp = byteSwap16(SIRF_BINARY_HEADER);
    writeGps((char*)&tmp,
              sizeof(tmp));

    tmp = byteSwap16(messageLength);
    writeGps((char*)&tmp,
             sizeof(tmp));

    writeGps((char*)msg,
             messageLength);

    tmp = _computeSirfChecksum((uint8_t*)msg,
                               messageLength);
    tmp = byteSwap16(tmp);
    writeGps((char*)&tmp,
             sizeof(tmp));

    tmp = SIRF_END_SEQUENCE;
    tmp = byteSwap16(tmp);
    writeGps((char*)&tmp,
             sizeof(tmp));
}

/** ****************************************************************************
 * @name pollSiRFVersionMsg format and send a binary version request message
 *       to SiRF
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void pollSiRFVersionMsg()
{
    tSiRFPollVersion msg;

    msg.id       = SIRF_POLL_SOFTWARE_VERSION; // 0x84
    msg.reserved = 0;
    _SiRFSendMessage((uint8_t *)&msg,
                     sizeof(msg));
}

/** ****************************************************************************
 * @name siRFStaticNavMsg format and send a binary message to enable or disable
 *       static nvigationto SiRF
 * @param [in] flag 1 = enable, 0 = disable
 * @retval N/A
 * @details Static navigation is a position filter for use with motor vehicle
 *          applications. When the vehicle�s speed falls below a threshold, the
 *          position and heading are frozen, and speed is set to 0. This
 *          condition continues until the computed speed rises above 1.2 times
 *          the threshold, or until the computed position is at least a set
 *          distance from the frozen place. The threshold speed and set distance
 *          may vary with software versions.
 ******************************************************************************/
void _siRFStaticNavMsg(uint8_t flag)
{
    tSiRFStaticNavigation msg;

    msg.id   = SIRF_STATIC_NAVIGATION; // 0x8f ID 143
    msg.flag = flag;
    _SiRFSendMessage((uint8_t *)&msg,
                     sizeof(msg));
}


/** ****************************************************************************
 * @name sirfBinarySwitchBaud format and send a version request message to SiRF
 * @param [in] baud - the baudrate to set
 * @retval N/A
 ******************************************************************************/
void sirfBinarySwitchBaud(uint32_t baud)
{
    tSiRFPSetSerialPort msg;

    msg.id       = SIRF_SET_SERIAL_PORT;
    msg.baud     = baud;
    msg.databits = 8;
    msg.stopbits = 1;
    msg.parity   = 0;
    msg.reserved = 0;
    _SiRFSendMessage((uint8_t *)&msg,
                     sizeof(msg));
}

/** ****************************************************************************
 * @name _SiRFSendMessageRateMsg LOCAL format and send a message to SiRF
 * @param [in] mode - mode to set
 * @param [in] messageIdToBeSet -
 * @param [in] rate - new rate to be set
 * @retval byte swappeed word
 ******************************************************************************/
static void _SiRFSendMessageRateMsg(uint8_t mode,
                                    uint8_t messageIdToBeSet,
                                    uint8_t rate)
{
    tSiRFSetMessageRate msg;

    msg.id               = SIRF_SET_MESSAGE_RATE; // 0xA6 166d
    msg.mode             = mode;
    msg.messageIdToBeSet = messageIdToBeSet;
    msg.updateRate       = rate;
    msg.reserved         = 0;
    _SiRFSendMessage((uint8_t *)&msg,
                     sizeof(msg));
}


/** ****************************************************************************
 * @name _SiRFSendMessageRateMsg LOCAL format and send a message to SiRF
 * @param [in] mode - mode to set
 * @param [in] messageIdToBeSet -
 * @param [in] rate - new rate to be set
 * @retval byte swappeed word
 ******************************************************************************/
/*
static void _SiRFSendModeControlMsg(void)
{
    tSiRFSetModeControl msg;

    msg.id                    = SIRF_SET_NAV_MODE; // 0x88  (mid 136)
    msg.reserved              = 0x0000;
    msg.degMode               = 0x04;  // Disable degraded-mode operation
    msg.posCalcMode           = 0x04;  // Set 5 Hz output
    msg.deadRecogEnable       = 0x00;
    msg.altitudeInput         = 0x0000;
    msg.altHoldMode           = 0x00;
    msg.altHoldSource         = 0x00;
    msg.coastTimeout          = 0x00;
    msg.degradedTimeout       = 0x00;
    msg.deadRecogTimeout      = 0x0F;
    msg.measAndTrackSmoothing = 0x02;

    _SiRFSendMessage((uint8_t *)&msg,
                     sizeof(msg));
}
*/

//#define FIVE_HZ_GPS

/** ****************************************************************************
 * @name _configureSiRFBinaryMessages LOCAL format and send a request message to
 *       SiRF
 * @param N/A
 * @retval N/A
 * @details msg SIRF_GEODETIC_NAVIGATION_DATA [0x29] 41d
 ******************************************************************************/
static void _configureSiRFBinaryMessages()
{
    _SiRFSendMessageRateMsg(0x02, 0, 0); // disable all messages
    OS_Delay(10);
    _SiRFSendMessageRateMsg(0x00, SIRF_GEODETIC_NAVIGATION_DATA, 1); // 1 [sec]
#ifdef FIVE_HZ_GPS
    _SiRFSendModeControlMsg();
#endif
}


/** ****************************************************************************
 * @name _parseSiRFGeodeticNavMsg LOCAL parse message for nav data
 * @param [in] msg - [0x29] 41d data to be parsed
 * @param [in] msgLength - length of the message to be parsed
 * @param [in] GPSData - GPS data structure to parse message into
 * @retval N/A
 * @details The data in the input msg array is byte swapped
 * 12/01/15 dkh cast the input buffer to a packed structure to reduce parsing
 ******************************************************************************/
static void _parseSiRFGeodeticNavMsg(char          *msg,
                                     unsigned int  msgLength,
                                     GpsData_t     *GPSData)
{
    tSiRFGeoNav *geo = (tSiRFGeoNav *)msg;
    double      latitude = 0.0;
    double      longitude = 0.0;
    double radTrueCourse = 0.0;

    if (msgLength < sizeof(tSiRFGeoNav))
        return; // error

    /******* packet parsed, now put it in GPSData as best as makes sense ****/
    GPSData->totalGGA++;
    GPSData->totalVTG++;

    // done this way to match up to NMEA which is abs value and N or E
    latitude = (int32_t)byteSwap32(geo->latitude); // [degrees] * 10^7, + = N
        GPSData->lat = latitude * ONE_OVER_TEN_TO_THE_SEVENTH; /// TEN_TO_THE_SEVENTH;

    longitude = (int32_t)byteSwap32(geo->longitude);
        GPSData->lon = longitude * ONE_OVER_TEN_TO_THE_SEVENTH; /// TEN_TO_THE_SEVENTH;

    GPSData->alt = 0.01 * (int32_t)byteSwap32(geo->altitudeEllipsoid); // / 100; // [m] * 10^2

// Unused in FW
//    GPSData->LonLatH[0] = (int32_t)byteSwap32(geo->latitude); // [deg]*10^7 + = N
//    GPSData->LonLatH[1] = (int32_t)byteSwap32(geo->longitude); // [deg]*10^7 + = E
//    GPSData->LonLatH[2] = (int32_t)byteSwap32(geo->altitudeEllipsoid); // [m] ellipsoid *100

    // 0x0000 = valid navigation (any bit set implies navigation solution is not optimal);
    // Bit 0:
    // On = solution not yet overdetermined(1) (< 5 SVs),
    // Off = solution overdetermined(1) (> = 5 SV)
    // Switched at the sazme time as PPS 5 satellites
    if ( geo->navValid > 0) {
          GPSData->HDOP = 50.0;
          GPSData->gpsFixType = 1; // zero is valid anything else is degraded
          gBitStatus.hwStatus.bit.unlockedInternalGPS = 1; // no signal lock
          gBitStatus.swStatus.bit.noGPSTrackReference = 1; // no GPS track
    } else {
          GPSData->HDOP = geo->hdop;
          GPSData->gpsFixType = 0;
          gBitStatus.hwStatus.bit.unlockedInternalGPS = 0; // locked
          gBitStatus.swStatus.bit.noGPSTrackReference = 0; // GPS track
    }

	GPSData->trueCourse     = 0.01 * byteSwap16(geo->courseOverGround); // / 100.0; // [deg]
    GPSData->rawGroundSpeed = 0.01 * byteSwap16(geo->speedOverGround);  // / 100.0;

    radTrueCourse = (0.01 * byteSwap16(geo->courseOverGround)) * 0.017453292519943; // * PI / 180.0; // [rad]
    GPSData->vNed[0] = GPSData->rawGroundSpeed * cos(radTrueCourse); // [m/s] * 10^2 N
    GPSData->vNed[1] = GPSData->rawGroundSpeed * sin(radTrueCourse); // [m/s] * 10^2 E

    // filtered down velocity
    GPSData->vNed[2] = avgDeltaSmoother(-((int16_t)byteSwap16(geo->climbRate) * 0.01), &downVelDelta);

    GPSData->GPSmonth = geo->utcMonth; // mm
    GPSData->GPSday   = geo->utcDay;   // dd
    GPSData->GPSyear  = byteSwap16(geo->utcYear) - 2000;  // last two digits of year

    GPSData->geoidAboveEllipsoid = (float)byteSwap32(geo->altitudeEllipsoid - geo->altitudeMSL) / 100.;  // 0x2221201f in m * 10^2

	GPSData->GPSHour           = geo->utcHour;
	GPSData->GPSMinute         = geo->utcMinute;
    GPSData->GPSSecondFraction = byteSwap16(geo->utcSecond);
    GPSData->GPSSecondFraction *= 0.001; //MILLISECOND_TO_SECOND;  MILLISECOND_TO_SECOND    1000
	GPSData->GPSSecond         = (char) floor(GPSData->GPSSecondFraction);
	GPSData->GPSSecondFraction -= GPSData->GPSSecond;

    GPSData->itow   = byteSwap32(geo->tow);
}

// nut used
/** ****************************************************************************
 * @name _processSiRFNavigationData LOCAL process message from SiRF
 * @param [in] msg - [0x02] data to be parsed
 * @param [in] msgLength - length of the message to be parsed
 * @param [in] GPSData - GPS data structure to parse message into
 * @retval N/A
 ******************************************************************************/
static void _processSiRFNavigationData(char          *msg,
                                       unsigned int  msgLength,
                                       GpsData_t *GPSData)
{
    tSiRFNav nav;
    int      axis;
    int      index;
    unsigned int      i;

    if (msgLength < sizeof(nav))
        return; // error

    index = 0; // X, Y, Z [m]
    for ( axis = 0; axis < NUM_AXIS; axis++) {
        SET_VALUE(nav.pos[axis], msg, &index);
    }
    // X, Y, Z [m/sec]
    for ( axis = 0; axis < NUM_AXIS; axis++) {
        SET_VALUE(nav.vel[axis], msg, &index);
    }

    nav.mode1 = msg[index]; index++;
    nav.hdop  = msg[index]; index++;
    nav.mode2 = msg[index]; index++;

    SET_VALUE(nav.gpsWeek, msg, &index);
    SET_VALUE(nav.tow, msg, &index);

    for ( i = 0 ; i < sizeof(nav.prn); i++ ) {
        nav.prn[i] = msg[index];
        index++;
    }

    /******* packet parsed, now put it in GPSData as best as makes sense ****/
    GPSData->updateFlagForEachCall |= 1 << GOT_GGA_MSG;
    GPSData->totalGGA++;
    for ( axis = 0; axis < NUM_AXIS; axis++) {
        GPSData->vNed[axis] = 0.125 * (double)nav.vel[axis]; // now in m/s
    }
}

/** ****************************************************************************
 * @name _processSiRFMeasuredTrackerData LOCAL process message from SiRF
 * @param [in] msg - [0x04] data to be parsed
 * @param [in] msgLength - length of the message to be parsed
 * @param [in] GPSData - GPS data structure to parse message into
 * @retval N/A
 ******************************************************************************/
static void _processSiRFMeasuredTrackerData(char          *msg,
                                            unsigned int  msgLength,
                                            GpsData_t *GPSData)
{
    tSiRFTracker tracker;
    int          index = 0;
    unsigned int          i;
    unsigned int          c;

    if (msgLength < (sizeof(tracker) - sizeof(tracker.channel)) ) {
        return;
    }
    SET_VALUE(tracker.gpsWeek, msg, &index);
    SET_VALUE(tracker.tow,     msg, &index); // [s]

    tracker.numChannels = msg[index]; index++;

    for ( i = 0; i < tracker.numChannels && (unsigned int)index < msgLength; i++) {
        tracker.channel[i].id      = msg[index]; index++;
        tracker.channel[i].azimuth = msg[index]; index++;
        tracker.channel[i].state   = msg[index]; index++;
        for ( c = 0; c < (unsigned int)NUM_CNO_VALUES && (unsigned int)index < msgLength; c++) {
            tracker.channel[i].cno[c] = msg[index]; index++;
        }
    }
}

static uint8_t configSirf = FALSE;

/** ****************************************************************************
 * @name processSiRFBinaryMessage process message from SiRF
 * @param [in] msg - data to be parsed
 * @param [in] msgLength - length of the message to be parsed
 * @param [in] GPSData - GPS data structure to parse message into
 * @retval N/A
 ******************************************************************************/
void processSiRFBinaryMessage(char          *msg,
                              unsigned int  *msgLength,
                              GpsData_t	*GPSData)
{
    int     index = GPSData->GPSMsgSignature.lengthOfHeaderIDLength;
    uint8_t msgId = msg[index]; index++;
    static char sirfSoftVer[128];
    uint32_t len = *msgLength;
    static uint32_t sirfSoftVerCnt;

    switch (msgId) {
    case SIRF_GEODETIC_NAVIGATION_DATA: // 0x29, 41
        _parseSiRFGeodeticNavMsg(&msg[index],
                                 *msgLength - index,
                                 GPSData);
        break;
    case  SIRF_NAVIGATION_DATA: // 0x02
        _processSiRFNavigationData(&msg[index],
                                   *msgLength - index,
                                   GPSData);
        break;
    case SIRF_MEASURED_TRACKER_DATA: // 0x04
        _processSiRFMeasuredTrackerData(&msg[index],
                                        *msgLength - index,
                                        GPSData);
        break;
    case SIRF_SOFTWRE_VERSION_STRING: // 0x06
      if (len < 128) {
          memcpy(sirfSoftVer, msg, len);
          sirfSoftVerCnt++;
      }
        if (configSirf == FALSE)
            _configureSiRFBinaryMessages(); // set output data rate

        configSirf = TRUE;
        GPSData->GPSConfigureOK = 1;
        break;
    case SIRF_SOFTWARE_COMMAND_ACK:  // 0x0b
    case SIRF_SOFTWARE_COMMAND_NACK: // 0x0c
      // just to get rid of warnings:
//    case SIRF_EPHEMERIS:             // 0x38 = 56
//    case SIRF_STATISICS:             // 0xE1 = 225
//    case SIRF_TRACKER_MESSAGES:      // 0x41 = 65
        // do nothing, essentially we can't turn this off
        break;
    default: // got unexpected message, send a disable for the message
       _SiRFSendMessageRateMsg(0x00,
                               msgId,
                               0);  // turn off this message
       break;
    }
    GPSData->GPSConfigureOK = 1;
}

#endif // GPS