/** ***************************************************************************
 * @file processUbloxGPS.c Processing for ublox GPS receiver.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 * @details
 *	 This file includes all specific processing, including configuring, ..,
 *   for ublox GPS receiver.
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
#include <stdlib.h>

#include "driverGPS.h"
#include "platformAPI.h"
#include "gpsAPI.h"

#ifdef DISPLAY_DIAGNOSTIC_MSG
#include "debug.h"
#endif

/// constants used for connecting with a unknown ublox
#define AUTO_BAUD_PROCESS   0
#define RESTORE_TO_FACTORY  1
#define CONFIG_TO_57600     2
#define QUERY_VERSION       3

/** @struct NAV_POSLLH_PAYLOAD_STRUCT
	@brief NAV_POSLLH payload.
 */
typedef struct {
	unsigned long  itow;
	signed long    LonLatH[3];
	unsigned long  HMSL;
	unsigned long  Hacc;
	unsigned long  Vacc;

} NAV_POSLLH_PAYLOAD_STRUCT;

/** @union NAV_POSLLH_PAYLOAD_UNION
	@brief NAV_POSLLH payload.
 */
union NAV_POSLLHPayLoad
{	char                       myChar[28];
	NAV_POSLLH_PAYLOAD_STRUCT  myData;
} NAV_POSLLH_PAYLOAD_UNION;


/** @struct NAV_VELNED_PAYLOAD_STRUCT
	@brief NAV_VELNED payload.
 */
typedef struct {
	unsigned long  itow;
	signed long    Vel_NED[3];
	unsigned long  Speed;
	unsigned long  GSpeed;
	signed long    Heading;
	unsigned long  SAcc;
	unsigned long  CAcc;

} NAV_VELNED_PAYLOAD_STRUCT;

/** @union NAV_VELNED_PAYLOAD_UNION
	@brief NAV_VELNED payload.
 */
union NAV_VELNEDPayload
{	char                       myChar[36];
	NAV_VELNED_PAYLOAD_STRUCT  myData;
} NAV_VELNED_PAYLOAD_UNION;

/** @struct NAV_STATUS_PAYLOAD_STRUCT
	@brief NAV_STATUS payload.
 */
typedef struct {
	unsigned long  itow;
	unsigned char  GPSfix;
	unsigned char  Flags;
	unsigned char  Diffs;
	unsigned char  res;
	unsigned long  TTFF;
	unsigned long  MSSS;
} NAV_STATUS_PAYLOAD_STRUCT;

/** @union NAV_STATUS_PAYLOAD_UNION
	@brief NAV_STATUS payload.
 */
union NAV_STATUSPayload
{	char                        myChar[16];
	 NAV_STATUS_PAYLOAD_STRUCT  myData;
}  NAV_STATUS_PAYLOAD_UNION;


char         msgIDTurnedOFF[TEST_ID_RECORD][2];
char         msgIDOn[TEST_ID_RECORD][2];
unsigned int totalUnwantedMsg   = 0;
unsigned char   IDCounter1         = 0;
unsigned char   IDCounter2         = 0;
unsigned char   processedIDcounter = 0;

//void decodeUbloxGPSBinary(int *numInBuffPrt, GpsData_t* GPSData);
//unsigned char configurate_uBloxGPS (GpsData_t* GPSData);

//void pollUbloxMsg(ubloxIDTypeSTRUCT *IDInput, GpsData_t *GPSData);
void _computeUbloxCheckSumCrc(char         *msg,
                              unsigned int *msgLength,
                              unsigned int *cCKA,
                              unsigned int *cCKB);
void _decodeVersionMsg(char          *msg,
                       unsigned int  *msgLength,
                       GpsData_t *GPSData);
unsigned char _sendAcknowlegdeProcess(ubloxIDTypeSTRUCT *cfgID,
                                      GpsData_t     *GPSData,
                                      unsigned char     *firstCall,
                                      void (*pt2MyFunc)(ubloxIDTypeSTRUCT *cfgID,
                                      GpsData_t     *GPSData));
void _generateSendCFG_MSG(ubloxIDTypeSTRUCT *IDInput,
                          GpsData_t     *GPSData);
void _generateSendCFG_PRT(ubloxIDTypeSTRUCT *cfgID,
                          GpsData_t     *GPSData);
void _generateSendCFG_CFG(ubloxIDTypeSTRUCT *cfgID,
                          GpsData_t     *GPSData);

void (*pt2TmpFunc)(ubloxIDTypeSTRUCT *cfgID, GpsData_t *GPSData);
void (*pt2TmpFunc1)(ubloxIDTypeSTRUCT *cfgID, GpsData_t *GPSData);

void decodeNavPvt(char *msg, GpsData_t *GPSData);

/** ****************************************************************************
 * @name configUbloxGPSReceiver LOCAL NOT CALLED configure ublox GPS receiver
 * @brief All GPS receiver configuration commands should be sent here if needed.
 * @author Dong An
 * @param [out] bytesFromBuffer - data from GPS data buffer
 * @param [out] GPSData - gps data structure
 * @retval N/A
 ******************************************************************************/
void configUbloxGPSReceiver(int           *bytesFromBuffer,
                            GpsData_t     *GPSData)
{
	static unsigned char versionKnown    = 0;
	static unsigned char ubloxPerCfgDone = 0;
	static unsigned char ubloxIOCfgDone  = 0;

	 if (versionKnown == 0)
         versionKnown = getConnectedWithUnknownStatusUbloxGPS( GPSData );
	 else /// GPS FW version is identified
	 	if(ubloxPerCfgDone == 0)
          ubloxPerCfgDone = configurateUBloxGPSPerformance( GPSData );
	 		else if (ubloxIOCfgDone == 0)
                ubloxIOCfgDone = configurateUBloxGPSIOMsgRate(GPSData);	/// configure ublox

	 if(ubloxIOCfgDone == 1) { /// count after config
			GPSData->GPSConfigureOK = 1;
			GPSData->LLHCounter     = 0;
			GPSData->VELCounter     = 0;
			GPSData->STATUSCounter  = 0;
			GPSData->SBASCounter    = 0;
			versionKnown            = 0;  /// release for next time
			ubloxPerCfgDone         = 0;
			ubloxIOCfgDone          = 0;
	 }
}
/** ****************************************************************************
 * @name ubloxGPSMsgSender send out GPS configuration message
 * @author Dong An
 * @param [in] msgID - message identifier
 * @param [in] msgpayLoad - input buffer
 * @param [in] msgpayLoadlength - length
 * @param [in] GPSData structure to parse into
 * @retval N/A
 ******************************************************************************/
void ubloxGPSMsgSender(ubloxIDTypeSTRUCT *msgID,
                       char              *MsgPayLoad,
                       int               *msgPayLoadLength,
                       GpsData_t         *GPSData)
{
	char                ubloxMsg[MAX_UBLOX_BODY_LENGTH+8];
	int                 i;
	static unsigned int j; ///need static for optimizer.
						   ///Otherwise, it could be not incremented at the end
	unsigned int        msgLength;
	unsigned int CK_A_C;
    unsigned int CK_B_C;
	short        payloadLength = *msgPayLoadLength;

	j = 0;
	ubloxMsg[j++] = 0xB5;	///header1
	ubloxMsg[j++] = 0x62;	///header2

	ubloxMsg[j++] = msgID->classID;
	ubloxMsg[j++] = msgID->msgID;

	ubloxMsg[j++] = payloadLength&0xFF;
	ubloxMsg[j++] = (payloadLength >> 8)&0xFF;

	for (i = 0; i < *msgPayLoadLength; i++)
        ubloxMsg[j++] = MsgPayLoad[i];
	msgLength = j + 2;  ///include 2 bytes crc
	_computeUbloxCheckSumCrc(ubloxMsg,
                             &msgLength,
                             &CK_A_C,
                             &CK_B_C);

	ubloxMsg[j++] = CK_A_C;
	ubloxMsg[j++] = CK_B_C;
	ubloxMsg[j++] = 0;
    writeGps(ubloxMsg, (uint16_t)j);
}

/** ****************************************************************************
 * @name processUbloxBinaryMessage  parse a complete ubblox binary message.
 * @author Dong An
 * @param [in] msgID - message identifier
 * @param [in] msgpayLoad - input buffer
 * @param [in] msgpayLoadlength - length
 * @param [in] GPSData structure to parse into
 * @retval N/A
 ******************************************************************************/
void processUbloxBinaryMessage(char          *msg,
                               unsigned int  *msgLength,
                               GpsData_t     *GPSData)
{
	unsigned int         checksumACalcu;
    unsigned int         checksumBCalcu;
	unsigned int         checksumARec;
    unsigned int         checksumBRec;
	unsigned int         i;
    int                  j;
	ubloxIDTypeSTRUCT    msgID;
	unsigned short       classIDMessID;
	static unsigned char firstFlag     = 0;
	static unsigned char firstReceived = 0;

	///testing only
	msgIDOn[IDCounter2][0] = msg[2];
	msgIDOn[IDCounter2][1] = msg[3];
	IDCounter2++;
	if (IDCounter2 == TEST_ID_RECORD)
        IDCounter2=0;

	///checksum
	checksumARec = msg[*msgLength - 2];
	checksumBRec = msg[*msgLength - 1];
    _computeUbloxCheckSumCrc(msg,
                             msgLength,
                             &checksumACalcu,
                             &checksumBCalcu);

	if( checksumACalcu!=checksumARec ||  checksumBCalcu !=checksumBRec)
        return;

	classIDMessID = msg[2];
	classIDMessID = classIDMessID << 8 |(msg[3] & 0xFF);

	switch (classIDMessID) {
        case  UBLOX_NAV_PVT:
        {
            decodeNavPvt(msg, GPSData);
            break;
        }
		case  UBLOX_NAV_POSLLH:  ///LLH
            j = 6;
            for (i = 0; i < 28; i++)
            {
                NAV_POSLLH_PAYLOAD_UNION.myChar[i] = msg[i+j];
            }

            ///similiar to NMEA GGA
            GPSData->updateFlagForEachCall |= 1<<GOT_GGA_MSG;
            GPSData->totalGGA++;
            GPSData->lat = (double)NAV_POSLLH_PAYLOAD_UNION.myData.LonLatH[0]*1e-7; ///deg (*1e7)
            GPSData->lon = (double)NAV_POSLLH_PAYLOAD_UNION.myData.LonLatH[1]*1e-7; /// deg (*1e7)
            GPSData->alt = NAV_POSLLH_PAYLOAD_UNION.myData.LonLatH[2]*1e-3 ; ///meters (*1e-3)
//            GPSData->LonLatH[2] = NAV_POSLLH_PAYLOAD_UNION.myData.LonLatH[2]*10e-3 ; ///meters (*1e-3)
            GPSData->itow       = NAV_POSLLH_PAYLOAD_UNION.myData.itow;
            GPSData->LLHCounter++;
		break;
		case UBLOX_NAV_VELNED:	///VEL-NED
            j = 6;
            for (i = 0; i < 36; i++)
            {
                NAV_VELNED_PAYLOAD_UNION.myChar[i] = msg[j + i] ;
            }
            ///NOT similiar to NMEA VTG message (speed and heading)
            // this is NOT FILTERED
            GPSData->updateFlagForEachCall |= 1<<GOT_VTG_MSG;
            GPSData->totalVTG++;
            GPSData->vNed[0]   = (double)NAV_VELNED_PAYLOAD_UNION.myData.Vel_NED[0] * 1e-2;		///m/s
            GPSData->vNed[1]   = (double)NAV_VELNED_PAYLOAD_UNION.myData.Vel_NED[1] * 1e-2;
            GPSData->vNed[2]   = (double)NAV_VELNED_PAYLOAD_UNION.myData.Vel_NED[2] * 1e-2;
            GPSData->itow      = NAV_VELNED_PAYLOAD_UNION.myData.itow;
            GPSData->GPSVelAcc = (double)NAV_VELNED_PAYLOAD_UNION.myData.SAcc * 1e-2; ///changed to m/s

            GPSData->updateFlagForEachCall |= 1 << GOT_UBLOX_VNED;
            GPSData->VELCounter++;
		break;
		case UBLOX_NAV_STATUS: ///NAV_STATUS
            GPSData->gpsFixType = msg[10]; ///fix indicator
            if((msg[11]&0x02) == 0x02)  ///6+5
                GPSData->GPSStatusWord |=1 << DGPS_ON;	///on
            else
                GPSData->GPSStatusWord &= !(1 << DGPS_ON);	///off
            GPSData->STATUSCounter++;
		break;
		case UBLOX_CFG_NAV:	///CFG NAV-Mess
		break;
		case UBLOX_NAV_SBAS:	///CFG NAV-SBAS
            GPSData->GPSStatusWord &= CLEAR_DGPS_SOURCES; ///clear
            switch(msg[12]) {
                case 0:
                    GPSData->GPSStatusWord |=1<<WAAS_ON;	///on
                    break;
                case 1:
                    GPSData->GPSStatusWord |=1<<EGNOS_ON;	///on
                    break;
                case 2:
                    GPSData->GPSStatusWord |=1<<MSAS_ON;	///on
                    break;
                default:
                    break;
            }
            GPSData->updateFlagForEachCall |= (1 << GOT_UBLOX_NAVSBAS);
            GPSData->SBASCounter++;
		break;
		case UBLOX_ACK_ACK:	///ACK message
            GPSData->reClassID = msg[6];
            GPSData->reMsgID   = msg[7];
            break;
		case UBLOX_MON_VER:
			_decodeVersionMsg(msg, msgLength, GPSData);
			GPSData->isGPSFWVerKnown = 1;
            break;
		case UBLOX_CFG_NAV2:
            break;
		default: ///other Messages
			///firewall: only after GPS cfg is done, because this may
			/// disturb the GPS cfg process
			if ((GPSData->GPSTopLevelConfig & (1<<EXTERNAL_GPS)) == (1<<EXTERNAL_GPS))
				break;  //if external GPS, do not do firewall

			if (GPSData->GPSConfigureOK != 0) {
				if (firstReceived == 0) {
                    msgIDTurnedOFF[IDCounter1][0] = msg[2];
                    msgIDTurnedOFF[IDCounter1][1] = msg[3];
                    firstReceived = 1;
                    IDCounter1++;
                    totalUnwantedMsg++;
                    if (IDCounter1 == TEST_ID_RECORD)
                        IDCounter1=0;
                } else {
					j = 0;
					for (i = 0; i < totalUnwantedMsg; i++) {
                        if (msgIDTurnedOFF[j][0] != msg[2] ||
                            msgIDTurnedOFF[j][1] != msg[3]) {  ///new one
                                msgIDTurnedOFF[IDCounter1][0] = msg[2];
                                msgIDTurnedOFF[IDCounter1][1] = msg[3];
                                IDCounter1++;
                                if (IDCounter1 == TEST_ID_RECORD)
                                    IDCounter1=0;
                                totalUnwantedMsg++;
                                break;
                         }///end of new one
                         j++;
                         if (j == TEST_ID_RECORD)
                             j = 0;
                    }///end of scanning
                }///end of not first time
                if (processedIDcounter!=IDCounter1) {
                    if ((msgIDTurnedOFF[processedIDcounter][0]&0xFF)!=0x04 &&
                        (msgIDTurnedOFF[processedIDcounter][1]&0xFF)!=0x01) {
                        // if it is a non-warning message
                         msgID.classID    = (UBLOX_CFG_MSG >> 8) & 0xFF;
                         msgID.msgID      = UBLOX_CFG_MSG & 0xFF;

                         msgID.cfgClassID = msgIDTurnedOFF[processedIDcounter][0];
                         msgID.cfgMsgID   = msgIDTurnedOFF[processedIDcounter][1];
                         msgID.rateRatio  = 0;
                         msgID.whatTagert = 0;  ///current port

                         pt2TmpFunc =&_generateSendCFG_MSG;

                         if(firstFlag == 0) {
                             _sendAcknowlegdeProcess(&msgID, GPSData, &firstFlag,pt2TmpFunc);
                             firstFlag = 1;
                         } else { ///after first time
                             if (_sendAcknowlegdeProcess(&msgID, GPSData, &firstFlag, pt2TmpFunc)) {
                                 firstFlag = 0; ///reset for next time
                                 processedIDcounter++;
                                 if (processedIDcounter == TEST_ID_RECORD)
                                     processedIDcounter = 0;
                                 GPSData->firewallCounter++;
                              }
                        }/// end non-first time
                         GPSData->firewallRunCounter++;
                     } else { //skip if it is a warning message
                        processedIDcounter++;
                        if (processedIDcounter == TEST_ID_RECORD)
                            processedIDcounter=0;
                     }
                }///end of "processedIDcounter!=IDCounter1"
        }/// end of not configurating
		break;
	}	///end of switch
}

void decodeNavPvt(char *msg, GpsData_t *GPSData)
{
    // Decode
    ubloxNavPvtSTRUCT *navPvt;
    navPvt = (ubloxNavPvtSTRUCT*)(&msg[UBLOX_BINAERY_HEADER_LEN]);

    // Get decoded results
    // fix type
    GPSData->gpsFixType = INVALID;
    if (navPvt->flags & 0x01)
    {
        GPSData->gpsFixType = SPP;
        if (navPvt->flags & 0x02)
        {
            GPSData->gpsFixType = DGPS;
        }
        uint8_t carrSoln = navPvt->flags >> 6;
        if (carrSoln == 1)
        {
            GPSData->gpsFixType = RTK_FLOAT;
        }
        else if(carrSoln == 2)
        {
            GPSData->gpsFixType = RTK_FIX;
        }
    }

    // num of satellites
    GPSData->numSatellites = navPvt->numSV;

    // lat
    GPSData->lat = navPvt->lat * 1.0e-7;

    // lon
    GPSData->lon = navPvt->lon * 1.0e-7;

    // altitude
    GPSData->alt = navPvt->height * 1.0e-3;
    // NED velocity
    GPSData->vNed[0] = navPvt->velN * 1.0e-3;
    GPSData->vNed[1] = navPvt->velE * 1.0e-3;
    GPSData->vNed[2] = navPvt->velD * 1.0e-3;
    // heading and ground speed
    GPSData->trueCourse = navPvt->headMot * 1.0e-5;
    GPSData->rawGroundSpeed = navPvt->gSpeed * 1.0e-3;
    // fractional second
    GPSData->GPSSecondFraction = navPvt->nano * 1.0e-9;
    // ellipsoid height
    GPSData->geoidAboveEllipsoid = (navPvt->height - navPvt->hMSL) * 1.0e-3;
    // itow
    GPSData->itow = navPvt->iTOW;
    if(GPSData->itow%1000 != 0){
        GPSData->itow+=1;           // correction of numerical issue
    }
    platformUpdateITOW(GPSData->itow);
    // year month day hour min sec
    GPSData->GPSmonth = navPvt->month;
    GPSData->GPSday = navPvt->day;
    GPSData->GPSyear = navPvt->year % 100;  
    GPSData->GPSHour = navPvt->hour;
    GPSData->GPSMinute = navPvt->min; 
    GPSData->GPSSecond = navPvt->sec;
    // accuracy
    GPSData->HDOP = navPvt->pDOP * 7.0e-3;  // scaling is 0.01 for pdop, hdop^2 is less than half of pdop^2
                                            // hdop is less than sqrt(2) (about 0.7) x pdop 
    GPSData->GPSHorizAcc = navPvt->hAcc * 1.0e-3;
    GPSData->GPSVertAcc = navPvt->vAcc * 1.0e-3;
    // update flag
    GPSData->updateFlagForEachCall |= 1 << GOT_GGA_MSG;
    GPSData->updateFlagForEachCall |= 1 << GOT_VTG_MSG;
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
int parseUbloBinaryMessage(uint8_t inByte, uint8_t *gpsMsg, GpsData_t *GPSData)
{
    static unsigned int len = 0;
    static uint8_t *ptr;
	static uint32_t sync = 0;
	unsigned static int  synced = 0;
    unsigned static int totalLen = 0;

    // wait for ublox binary header to sync
    sync   = (sync << 8) | inByte;
    if (synced == 0)
    {
        if ((sync & 0x0000ffff) == UBLOX_BINAERY_SYNC)
        {
            gpsMsg[0] = (UBLOX_BINAERY_SYNC >> 8) & 0xff;
            gpsMsg[1] = UBLOX_BINAERY_SYNC & 0xff;
            synced = 1;
            ptr = &gpsMsg[2];
            len = 2;
        }
        return 0;
    }
    // synced, wait for a complete packet
    //  class, ID, length
    *ptr++ = inByte;
    len++;
    // header bytes are all received
    if (len == UBLOX_BINAERY_HEADER_LEN)
    {
        // total length = headerlen + payload length + 2bytes CRC
        int payloadLen = gpsMsg[5];
        payloadLen = payloadLen << 8 | gpsMsg[4];
        totalLen = UBLOX_BINAERY_HEADER_LEN + payloadLen + 2;
        if (totalLen > MAX_MSG_LENGTH)
        {
            len = 0;
            totalLen = 0;
            synced = 0;
            return 0;
        }
    }
    // a complete packet is received
    if (len == totalLen)
    {
        processUbloxBinaryMessage((char *)gpsMsg, &len, GPSData);
        len = 0;
        totalLen = 0;
        synced = 0;
    }
    

    return 0;
}

/** ****************************************************************************
 * @name _computeUbloxCheckSumCrc compute checksum of a complete ubblox binary
 *       message.
 * @author Dong An
 * @param [in] msg- message
 * @param [in] msgLength - input buffer length
 * @param [out] cCKA - checksum A
 * @param [out] cCKB - checksum B
 * @retval N/A
 ******************************************************************************/
void _computeUbloxCheckSumCrc(char         *msg,
                              unsigned int *msgLength,
                              unsigned int *cCKA,
                              unsigned int *cCKB)
{
	int          i;
    int          CKRange;
	unsigned int checksumACalcu = 0;
    unsigned int checksumBCalcu = 0;

	CKRange = *msgLength - 4;  ///exclude header and crc itself
	for (i = 0; i < CKRange; i++)		{
		checksumACalcu = checksumACalcu + (msg[i+2] & 0xFF);
		checksumACalcu = checksumACalcu& 0xFF;

		checksumBCalcu = checksumBCalcu + checksumACalcu;
		checksumBCalcu = checksumBCalcu & 0xFF;
	}
	*cCKA = checksumACalcu;
	*cCKB = checksumBCalcu;
}

/** ****************************************************************************
 * @name generateSendCFG_NAV generate and send out CFG-NAV message.
 * @author Dong An
 * @param [in] cfgID- message
 * @param [in] GPSData - GPS data structure to pull message from
 * @retval N/A
 ******************************************************************************/
void generateSendCFG_NAV(ubloxIDTypeSTRUCT *cfgID,
                         GpsData_t	   *GPSData)
{
	int               i;
	char              tmpMsgPayload[MAX_UBLOX_BODY_LENGTH ];
	ubloxIDTypeSTRUCT tmpMsgID;

	tmpMsgID.classID = cfgID->classID;
	tmpMsgID.msgID   = cfgID->msgID;

	i = 0;					  ///offset
	tmpMsgPayload[i++] = 0x3;  ///0: Platform, default=0x03=Automotive;
						     ///configured to 0x6=airborne with <2g
						     //changed to 0x03 Automotive for UCB 2.2.1
	tmpMsgPayload[i++] = 0x04;  ///1: MinSVs, default=0x03
								  ///configured to 04
	tmpMsgPayload[i++] = 0x10; ///2: MaxSVs, default=0x10
	tmpMsgPayload[i++] = 0x18; ///3: MinCNO,default=0x18
	tmpMsgPayload[i++] = 0x14; ///4: AbsCNO, default=0x14
	tmpMsgPayload[i++] = 0x05; ///5: MinELE, defautl=0x05
	tmpMsgPayload[i++] = 0x00; ///6: DGPSTTR, default=0x00
	tmpMsgPayload[i++] = 0x3C; ///7: DGPSTO, default=0x3C
	tmpMsgPayload[i++] = 0x3C; ///8: PRCAGE, default=0x3C
	tmpMsgPayload[i++] = 0x14; ///9: CPCAGE, default=0x14
	tmpMsgPayload[i++] = 0xE8; ///10: U2, MinCLT, default=0xE8
	tmpMsgPayload[i++] = 0x03; ///11: default=0x03
	tmpMsgPayload[i++] = 0x00; ///12: U2, AbsCLT, default=0x00
	tmpMsgPayload[i++] = 0x00; ///13: default=0x00
	tmpMsgPayload[i++] = 0x00; ///14: MaxDR, default=0x00
	tmpMsgPayload[i++] = 0x07; ///15: NAVOPT, default=0x17
									///configured to 0x07
	tmpMsgPayload[i++] = 0x64; ///16: U2, PDOP, default=0xFA
									///configured to 0x64
	tmpMsgPayload[i++] = 0x00; ///17: Default=0x00
	tmpMsgPayload[i++] = 0x64; ///18: U2, TDOP, default=0xFA
									///configured to 0x64
	tmpMsgPayload[i++] = 0x00; ///19: default=0x00
	tmpMsgPayload[i++] = 0x64; ///20: U2, PACC, default=0x64
	tmpMsgPayload[i++] = 0x00; ///21: default=0x00
	tmpMsgPayload[i++] = 0x2C; ///22: U2, TACC, default=0x2C
	tmpMsgPayload[i++] = 0x01; ///23: default=0x01
	tmpMsgPayload[i++] = 0x0F; ///24: U2, FACC, default=0F
	tmpMsgPayload[i++] = 0x00; ///25: default=0x00
	tmpMsgPayload[i++] = 0x00; ///26: StaticThres, default=0x00
	tmpMsgPayload[i++] = 0x00; ///27: reserved.

	ubloxGPSMsgSender(&tmpMsgID,
                      tmpMsgPayload,
                      &i,
                      GPSData);
}

/** ****************************************************************************
 * @name generateSendCFG_NAV2 generate and send out CFG-NAV2 message.
 * @author Dong An
 * @param [in] cfgID- message
 * @param [in] GPSData - GPS data structure to pull message from
 * @retval N/A
 ******************************************************************************/
void generateSendCFG_NAV2(ubloxIDTypeSTRUCT *cfgID,
                          GpsData_t	    *GPSData)
{
	int               i;
    int               j;
	char              tmpMsgPayload[MAX_UBLOX_BODY_LENGTH ];
	ubloxIDTypeSTRUCT tmpMsgID;


	tmpMsgID.classID = cfgID->classID;
	tmpMsgID.msgID   = cfgID->msgID;

	i = 0;					  ///offset
	tmpMsgPayload[i++] = 0x3;  ///0: Platform; default=0x03=automotive;
								  ///configured to 0x6=airborne with <2g
								  //changed to automotive for UCB 2.1.1
	for (j = 0; j < 3; j++)
        tmpMsgPayload[i++] = 0x00; /// reserved 1-3
	tmpMsgPayload[i++] = 0x03; ///4: MinSVInitial, default=0x03
	tmpMsgPayload[i++] = 0x04; //5  MinSVs, default=0x03;
								 ///configured to 0x04
	tmpMsgPayload[i++] = 0x10; ///6: MaxSVs, default=0x10;
	tmpMsgPayload[i++] = 0x02; ///7: FixMode, default=0x02;
	tmpMsgPayload[i++] = 0x50; ///8: I4, FixedAltitude, default=0x50;
	tmpMsgPayload[i++] = 0xC3; ///9:  default=0xC3
	tmpMsgPayload[i++] = 0x00; ///10: default=0x00
	tmpMsgPayload[i++] = 0x00; ///11: default=0x00
	tmpMsgPayload[i++] = 0x18; ///12: MinCN0Initial, default=0x18
	tmpMsgPayload[i++] = 0x14; ///13: MinCN0After, default=0x14
	tmpMsgPayload[i++] = 0x05; ///14: MinElE default=0x05
	tmpMsgPayload[i++] = 0x3C; ///15: DGPSTO, default=0x3C
	tmpMsgPayload[i++] = 0x00; ///16: MaxDR, default=0x00
	tmpMsgPayload[i++] = 0x01; ///17: NAVOPT, default=0x01
	for (j = 0; j < 2; j++)
        tmpMsgPayload[i++] = 0x00; ///18-19: U2 reserved
	tmpMsgPayload[i++] = 0x64; ///20: U2,PDOP, default=0xFA
									/// configured to 0x64
	tmpMsgPayload[i++] = 0x00; ///21: default=0x00
	tmpMsgPayload[i++] = 0x64; ///22: U2, TDOP, default=0xFA
								  ///configured to 0x64
	tmpMsgPayload[i++] = 0x00; ///23: default=00
	tmpMsgPayload[i++] = 0x64; ///24: U2, PACC, defautl=0x64
	tmpMsgPayload[i++] = 0x00; ///25: default=0x00
	tmpMsgPayload[i++] = 0x2C; ///26: U2, TACC, default=0x2C
	tmpMsgPayload[i++] = 0x01; ///27: default=0x01
	tmpMsgPayload[i++] = 0x00; ///28: StaticThres, default=0x00
	for (j = 0; j < 11; j++)
      tmpMsgPayload[i++] = 0x00; /// reserved 29-39

	ubloxGPSMsgSender(&tmpMsgID,
                      tmpMsgPayload,
                      &i,
                      GPSData);
}

/** ****************************************************************************
 * @name GenerateSendCFG_RXM  generate and send out CFG-RXM message.
 * @author Dong An
 * @param [in] cfgID- message
 * @param [in] GPSData - GPS data structure to pull message from
 * @retval N/A
 ******************************************************************************/
void GenerateSendCFG_RXM(ubloxIDTypeSTRUCT *cfgID,
                         GpsData_t     *GPSData)
{
	int               i;
	char              tmpMsgPayload[MAX_UBLOX_BODY_LENGTH ];
	ubloxIDTypeSTRUCT tmpMsgID;


	tmpMsgID.classID = cfgID->classID;
	tmpMsgID.msgID   = cfgID->msgID;

	i = 0;
	tmpMsgPayload[i++] = 1;
	tmpMsgPayload[i++] = 0;

	ubloxGPSMsgSender(&tmpMsgID,
                      tmpMsgPayload,
                      &i,
                      GPSData);
}

/** ****************************************************************************
 * @name generateSendCFG_SBAS  generate and send out CFG-SBAS message.
 * @author Dong An
 * @param [in] cfgID- message
 * @param [in] GPSData - GPS data structure to pull message from
 * @retval N/A
 ******************************************************************************/
void generateSendCFG_SBAS(ubloxIDTypeSTRUCT *cfgID,
                          GpsData_t     *GPSData)
{
	int               i;
	char              tmpMsgPayload[MAX_UBLOX_BODY_LENGTH ];
	ubloxIDTypeSTRUCT tmpMsgID;

	tmpMsgID.classID = cfgID->classID;
	tmpMsgID.msgID   = cfgID->msgID;

	i = 0;
	tmpMsgPayload[i++] = 0x1;  ///SBAS Enabled(1)
	tmpMsgPayload[i++] = 0x7;  ///Bit 0: Use SBAS GEOs as a ranging source (for navigation)
							   ///Bit 1: Use SBAS Differential Corrections
							   ///Bit 2: Use SBAS Integrity Information
	tmpMsgPayload[i++] = 0x1;  ///1 SBAS search channel (valid range: 0 - 3) to use
	tmpMsgPayload[i++] = 0;    ///reserved
	tmpMsgPayload[i++] = 0;    ///auto-search
	tmpMsgPayload[i++] = 0;
	tmpMsgPayload[i++] = 0;
	tmpMsgPayload[i++] = 0;

	ubloxGPSMsgSender(&tmpMsgID,
                      tmpMsgPayload,
                      &i,
                      GPSData);
}

/** ****************************************************************************
 * @name generateSendCFG_RATE  generate and send out CFG-RATE message.
 * @author Dong An
 * @param [in] cfgID- message
 * @param [in] GPSData - GPS data structure to pull message from
 * @retval N/A
 ******************************************************************************/
void generateSendCFG_RATE(ubloxIDTypeSTRUCT *cfgID,
                          GpsData_t     *GPSData)
{
	int               i;
	char              tmpMsgPayload[MAX_UBLOX_BODY_LENGTH ];
	ubloxIDTypeSTRUCT tmpMsgID;
	unsigned short    Meas;
    unsigned short    Nav;
    unsigned short    Time;

	tmpMsgID.classID = cfgID->classID;
	tmpMsgID.msgID   = cfgID->msgID;

	if( (GPSData->GPSTopLevelConfig & (1 << HZ2)) == (1 << HZ2))
		Meas = 500;   ///ms
	else
        Meas = 250;   ///ms

	Nav  = 1;
	Time = 0;
	i    = 0;
	tmpMsgPayload[i++] = Meas & 0xFF;
	tmpMsgPayload[i++] = (Meas >> 8) & 0xFF;

	tmpMsgPayload[i++] = Nav & 0xFF;
	tmpMsgPayload[i++] = (Nav >> 8) & 0xFF;

	tmpMsgPayload[i++] = Time & 0xFF;
	tmpMsgPayload[i++] = (Time >> 8) & 0xFF;

	ubloxGPSMsgSender(&tmpMsgID,
                      tmpMsgPayload,
                      &i,
                      GPSData);
}

/** ****************************************************************************
 * @name pollUbloxMsg  poll a ublox message.
 * @author Dong An
 * @param [in] cfgID- message
 * @param [in] GPSData - GPS data structure to pull message from
 * @retval N/A
 ******************************************************************************/
void pollUbloxMsg(ubloxIDTypeSTRUCT *IDInput,
                  GpsData_t     *GPSData)
{
	int               i;
	char              tmpMsgPayload[MAX_UBLOX_BODY_LENGTH ];
	ubloxIDTypeSTRUCT tmpMsgID;

	tmpMsgID.classID = IDInput->classID;
	tmpMsgID.msgID   = IDInput->msgID;

	i = 0;	/// o bytes in payload
	ubloxGPSMsgSender(&tmpMsgID,
                      tmpMsgPayload,
                      &i,
                      GPSData);
}

/** ****************************************************************************
 * @name _generateSendCFG_MSG LOCAL generate and send out CFG-MSG message.
 * @author Dong An
 * @param [in] cfgID- message
 * @param [in] GPSData - GPS data structure to pull message from
 * @retval N/A
 ******************************************************************************/
void _generateSendCFG_MSG(ubloxIDTypeSTRUCT *IDInput,
                          GpsData_t     *GPSData)
{
	int               i;
	char              tmpMsgPayload[MAX_UBLOX_BODY_LENGTH ];
	ubloxIDTypeSTRUCT tmpMsgID;

	tmpMsgID.classID = (UBLOX_CFG_MSG >> 8) & 0xFF;
	tmpMsgID.msgID   = UBLOX_CFG_MSG & 0xFF;

	i = 0;
	switch (IDInput->whatTagert) {
		case 0: ///current
            tmpMsgPayload[i++] = IDInput->cfgClassID;
            tmpMsgPayload[i++] = IDInput->cfgMsgID;

            tmpMsgPayload[i++] = IDInput->rateRatio;	///Target 0
            break;
		case 4:/// all target
            tmpMsgPayload[i++] = IDInput->cfgClassID;
            tmpMsgPayload[i++] = IDInput->cfgMsgID;

            tmpMsgPayload[i++] = IDInput->rateRatio;	///Target 0
            tmpMsgPayload[i++] = IDInput->rateRatio;	///Target 1
            tmpMsgPayload[i++] = IDInput->rateRatio;	///Target 2
            tmpMsgPayload[i++] = IDInput->rateRatio;	///Target 3
            break;
		default:
            break;
	}

	ubloxGPSMsgSender(&tmpMsgID,tmpMsgPayload,&i, GPSData);

}

/** ****************************************************************************
 * @name configurateUBloxGPSPerformance configure ublox NAV performance.
 * @author Dong An
 * @param [in] GPSData - GPS data structure to pull message from
 * @retval N/A
 ******************************************************************************/
unsigned char configurateUBloxGPSPerformance (GpsData_t* GPSData)
{
	static unsigned char firstFlag1 = 0;
	static int           CFGIndex1  = UBLOX_CFG_RXM;
	ubloxIDTypeSTRUCT    msgID;
	unsigned char        finish     = 0;

	switch (CFGIndex1) {
		case UBLOX_CFG_RXM: ///CFG RXM
            msgID.classID = (UBLOX_CFG_RXM >> 8) & 0xFF;
            msgID.msgID   = UBLOX_CFG_RXM & 0xFF;
            pt2TmpFunc1   = &GenerateSendCFG_RXM;    ///fast aquisition mode
            if(firstFlag1 == 0) {
                _sendAcknowlegdeProcess(&msgID,
                                       GPSData,
                                       &firstFlag1,
                                       pt2TmpFunc1);
                firstFlag1 = 1;
            } else { ///after first time
                if (_sendAcknowlegdeProcess(&msgID,
                                           GPSData,
                                           &firstFlag1,
                                           pt2TmpFunc1)) {
                    firstFlag1 = 0; //reset for next case
                    CFGIndex1  = UBLOX_CFG_SBAS; ///exit
                }
            }/// end non-first time
            break;
		case UBLOX_CFG_SBAS:    ///CFG-SBAS
            msgID.classID = (UBLOX_CFG_SBAS >> 8) & 0xFF;
            msgID.msgID   = UBLOX_CFG_SBAS & 0xFF;
            pt2TmpFunc1   = &generateSendCFG_SBAS;
            if(firstFlag1 == 0) {
                _sendAcknowlegdeProcess(&msgID,
                                       GPSData,
                                       &firstFlag1,
                                       pt2TmpFunc1);
                firstFlag1 = 1;
            } else { ///after first time
                if (_sendAcknowlegdeProcess(&msgID,
                                           GPSData,
                                           &firstFlag1,
                                           pt2TmpFunc1)) {
                    firstFlag1 = 0; ///reset for next case
                    CFGIndex1  = UBLOX_CFG_NAV; ///exit
                }
            }/// end non-first time
            break;
		case UBLOX_CFG_NAV:    ///CFG-NAV or CFG-NAV2
            if (GPSData->ubloxOldVersion == 0) { /// old version
                msgID.classID = (UBLOX_CFG_NAV >> 8) & 0xFF;
                msgID.msgID   = UBLOX_CFG_NAV & 0xFF;
                pt2TmpFunc1   = &generateSendCFG_NAV;
                if(firstFlag1 == 0) {
                    _sendAcknowlegdeProcess(&msgID,
                                           GPSData,
                                           &firstFlag1,
                                           pt2TmpFunc1);
                    firstFlag1 = 1;
                } else { ///after first time
                    if (_sendAcknowlegdeProcess(&msgID,
                                               GPSData,
                                               &firstFlag1,
                                               pt2TmpFunc1)) {
                        firstFlag1 = 0; ///reset for next case
                        CFGIndex1  = UBLOX_CFG_RATE; ///exit
                    }
                }/// end non-first time
            } else {/// for the latest ublox version
                msgID.classID = (UBLOX_CFG_NAV2 >> 8) & 0xFF;
                msgID.msgID   = UBLOX_CFG_NAV2 & 0xFF;
                pt2TmpFunc1   = &generateSendCFG_NAV2;
                if(firstFlag1 == 0) {
                    _sendAcknowlegdeProcess(&msgID,
                                           GPSData,
                                           &firstFlag1,
                                           pt2TmpFunc1);
                    firstFlag1 = 1;
                } else { ///after first time
                    if (_sendAcknowlegdeProcess(&msgID,
                                               GPSData,
                                               &firstFlag1,
                                               pt2TmpFunc1)) {
                        firstFlag1 = 0; //reset for next case
                        CFGIndex1  = UBLOX_CFG_RATE; //exit
                    }
                }/// end non-first time
            }	///end of "for latest version"
            break;
		case UBLOX_CFG_RATE:/// CFG-RATE (last increase the processor load)
            msgID.classID = (UBLOX_CFG_RATE >> 8) & 0xFF;
            msgID.msgID   = UBLOX_CFG_RATE & 0xFF;
            pt2TmpFunc1   = &generateSendCFG_RATE;
            if(firstFlag1 == 0) {
                _sendAcknowlegdeProcess(&msgID,
                                       GPSData,
                                       &firstFlag1,
                                       pt2TmpFunc1);
                firstFlag1 = 1;
            } else { ///after first time
                if (_sendAcknowlegdeProcess(&msgID,
                                           GPSData,
                                           &firstFlag1,
                                           pt2TmpFunc1)) {
                    firstFlag1 = 0; ///reset for next case
                    CFGIndex1  = CFG_DONE; ///exit
                }
            }/// end non-first time
            break;
		case CFG_DONE:
            finish    = 1;
            CFGIndex1 = UBLOX_CFG_RXM;
            break;
	}
	return finish;
}

/** ****************************************************************************
 * @name configurateUBloxGPSIOMsgRate configure ublox I/O message.
 * @author Dong An
 * @param [in] GPSData - GPS data structure to pull message from
 * @retval N/A
 ******************************************************************************/
unsigned char configurateUBloxGPSIOMsgRate (GpsData_t* GPSData)
{
	static unsigned char firstFlag = 0;
	static unsigned int  CFGIndex  = UBLOX_NAV_POSLLH;
	ubloxIDTypeSTRUCT    msgID;
	unsigned char        finish    = 0;

	msgID.classID = (UBLOX_CFG_MSG >> 8) & 0xFF;/// All cases use the same CFG-MSG(set message rate)
	msgID.msgID   = UBLOX_CFG_MSG & 0xFF;

	switch (CFGIndex) {
		///the following for binary message AHRS actually needs
		case UBLOX_NAV_POSLLH:
            msgID.cfgClassID = (UBLOX_NAV_POSLLH >> 8) & 0xFF;
            msgID.cfgMsgID   = UBLOX_NAV_POSLLH & 0xFF;
            msgID.rateRatio  = 1;
            msgID.whatTagert = 0;  ///current port
            pt2TmpFunc       = &_generateSendCFG_MSG;
            if(firstFlag == 0) {
                _sendAcknowlegdeProcess(&msgID,
                                       GPSData,
                                       &firstFlag,
                                       pt2TmpFunc);
                firstFlag = 1;
            } else { ///after first time
                if (_sendAcknowlegdeProcess(&msgID,
                                           GPSData,
                                           &firstFlag,
                                           pt2TmpFunc)) {
                    firstFlag = 0; ///reset for next case
                    CFGIndex  = UBLOX_NAV_VELNED; ///exit
                }
            }/// end non-first time
            break;
		case UBLOX_NAV_VELNED:
            msgID.cfgClassID = (UBLOX_NAV_VELNED >> 8) & 0xFF;
            msgID.cfgMsgID   = UBLOX_NAV_VELNED & 0xFF;
            msgID.rateRatio  = 1;
            msgID.whatTagert = 0;  ///current port
            pt2TmpFunc       = &_generateSendCFG_MSG;
            if(firstFlag == 0) {
                _sendAcknowlegdeProcess(&msgID,
                                       GPSData,
                                       &firstFlag,
                                       pt2TmpFunc);
                firstFlag = 1;
            } else { ///after first time
                if (_sendAcknowlegdeProcess(&msgID,
                                           GPSData,
                                           &firstFlag,
                                           pt2TmpFunc)) {
                    firstFlag = 0; ///reset for next case
                    CFGIndex  = UBLOX_NAV_STATUS; ///exit
                }
            }/// end non-first time
            break;
		case UBLOX_NAV_STATUS:
            msgID.cfgClassID = (UBLOX_NAV_STATUS >> 8) & 0xFF;
            msgID.cfgMsgID   = UBLOX_NAV_STATUS & 0xFF;
            msgID.rateRatio  = 1;
            msgID.whatTagert = 0;  ///current port
            pt2TmpFunc       = &_generateSendCFG_MSG;
            if(firstFlag == 0) {
                _sendAcknowlegdeProcess(&msgID,
                                       GPSData,
                                       &firstFlag,
                                       pt2TmpFunc);
                firstFlag = 1;
            }
            else { ///after first time
                if (_sendAcknowlegdeProcess(&msgID,
                                           GPSData,
                                           &firstFlag,
                                           pt2TmpFunc)) {
                    firstFlag = 0; ///reset for next case
                    CFGIndex  = UBLOX_NAV_SBAS; ///exit
                }
            }/// end non-first time
            break;
		case UBLOX_NAV_SBAS:
            msgID.cfgClassID = (UBLOX_NAV_SBAS >> 8) & 0xFF;
            msgID.cfgMsgID   = UBLOX_NAV_SBAS & 0xFF;
            msgID.rateRatio  = NAV_SBAS_RATE_RATIO;
            msgID.whatTagert = 0;  ///current port
            pt2TmpFunc       = &_generateSendCFG_MSG;
            if(firstFlag == 0) {
                _sendAcknowlegdeProcess(&msgID,
                                       GPSData,
                                       &firstFlag,
                                       pt2TmpFunc);
                firstFlag = 1;
            } else { ///after first time
                if (_sendAcknowlegdeProcess(&msgID,
                                           GPSData,
                                           &firstFlag,
                                           pt2TmpFunc)) {
                    firstFlag = 0; ///reset for next case
                    CFGIndex  = UBLOX_NMEA_GLL; ///exit
                }
            }/// end non-first time
            break;
		///the following for NMEA
		case UBLOX_NMEA_GLL: ///CFG RXM
            msgID.cfgClassID = (UBLOX_NMEA_GLL >> 8) & 0xFF;
            msgID.cfgMsgID   = UBLOX_NMEA_GLL & 0xFF;
            msgID.rateRatio  = 0;
            msgID.whatTagert = 4;  ///GPS port
            pt2TmpFunc       = &_generateSendCFG_MSG;
            if(firstFlag == 0) {
                _sendAcknowlegdeProcess(&msgID,
                                       GPSData,
                                       &firstFlag,
                                       pt2TmpFunc);
                firstFlag = 1;
            } else { ///after first time
                if (_sendAcknowlegdeProcess(&msgID,
                                           GPSData,
                                           &firstFlag,
                                           pt2TmpFunc)) {
                    firstFlag = 0; ///reset for next case
                    CFGIndex  = UBLOX_NMEA_GSA; ///exit
                }
            }/// end non-first time
            break;
        case UBLOX_NMEA_GSA:
		///NMEA Messages: GSA
            msgID.cfgClassID = (UBLOX_NMEA_GSA >> 8) & 0xFF;
            msgID.cfgMsgID   = UBLOX_NMEA_GSA & 0xFF;
            msgID.rateRatio  = 0;
            msgID.whatTagert = 4;  ///GPS port
            pt2TmpFunc       = &_generateSendCFG_MSG;
            if(firstFlag == 0) {
                _sendAcknowlegdeProcess(&msgID,
                                       GPSData,
                                       &firstFlag,
                                       pt2TmpFunc);
                firstFlag = 1;
            } else { //after first time
                if (_sendAcknowlegdeProcess(&msgID,
                                           GPSData,
                                           &firstFlag,
                                           pt2TmpFunc)) {
                    firstFlag = 0; ///reset for next case
                    CFGIndex  = UBLOX_NMEA_GSV; ///exit
                }
            }/// end non-first time
            break;
        case UBLOX_NMEA_GSV:
            ///NMEA Messages: GSV
            msgID.cfgClassID = (UBLOX_NMEA_GSV >> 8) & 0xFF;
            msgID.cfgMsgID   = UBLOX_NMEA_GSV & 0xFF;
            msgID.rateRatio  = 0;
            msgID.whatTagert = 4;  ///GPS port
            pt2TmpFunc       = &_generateSendCFG_MSG;
            if(firstFlag == 0) {
                _sendAcknowlegdeProcess(&msgID,
                                       GPSData,
                                       &firstFlag,
                                       pt2TmpFunc);
                firstFlag = 1;
            }
            else { ///after first time
                if (_sendAcknowlegdeProcess(&msgID,
                                           GPSData,
                                           &firstFlag,
                                           pt2TmpFunc)) {
                    firstFlag = 0; ///reset for next case
                    CFGIndex  = UBLOX_NMEA_ZDA; ///exit
                }
            }/// end non-first time
            break;
        case UBLOX_NMEA_ZDA:
            msgID.cfgClassID = (UBLOX_NMEA_ZDA >> 8) & 0xFF;
            msgID.cfgMsgID   = UBLOX_NMEA_ZDA & 0xFF;
            msgID.rateRatio  = 0;
            msgID.whatTagert = 4;  ///GPS port
            pt2TmpFunc       = &_generateSendCFG_MSG;
            if(firstFlag == 0) {
                _sendAcknowlegdeProcess(&msgID,
                                       GPSData,
                                       &firstFlag,
                                       pt2TmpFunc);
                firstFlag = 1;
            } else { ///after first time
                if (_sendAcknowlegdeProcess(&msgID,
                                           GPSData,
                                           &firstFlag,
                                           pt2TmpFunc)) {
                    firstFlag = 0; ///reset for next case
                    CFGIndex  = UBLOX_NMEA_VTG; ///exit
                }
            }/// end non-first time
            break;
        case UBLOX_NMEA_VTG:
            ///NMEA Messages: VTG
            msgID.cfgClassID = (UBLOX_NMEA_VTG >> 8) & 0xFF;
            msgID.cfgMsgID   = UBLOX_NMEA_VTG & 0xFF;
            msgID.rateRatio  = 0;
            msgID.whatTagert = 4;  ///GPS port
            msgID.rateRatio  = 1;
            pt2TmpFunc       = &_generateSendCFG_MSG;
            if(firstFlag == 0) {
                _sendAcknowlegdeProcess(&msgID,
                                       GPSData,
                                       &firstFlag,
                                       pt2TmpFunc);
                firstFlag = 1;
            } else { //after first time
                if (_sendAcknowlegdeProcess(&msgID,
                                           GPSData,
                                           &firstFlag,
                                           pt2TmpFunc)) {
                    firstFlag = 0; ///reset for next case
                    CFGIndex  = UBLOX_NMEA_GGA; ///exit
                }
            }/// end non-first time
            break;
        case UBLOX_NMEA_GGA:
            msgID.cfgClassID = (UBLOX_NMEA_GGA >> 8) & 0xFF;
            msgID.cfgMsgID   = UBLOX_NMEA_GGA & 0xFF;
            msgID.rateRatio  = 0;
            msgID.whatTagert = 4;  //GPS port
            msgID.rateRatio  = 1;
            pt2TmpFunc = &_generateSendCFG_MSG;

            if(firstFlag == 0) {
                _sendAcknowlegdeProcess(&msgID,
                                       GPSData,
                                       &firstFlag,
                                       pt2TmpFunc);
                firstFlag = 1;
            } else { ///after first time
                if (_sendAcknowlegdeProcess(&msgID,
                                           GPSData,
                                           &firstFlag,
                                           pt2TmpFunc)) {
                    firstFlag = 0; ///reset for next case
                    CFGIndex  = UBLOX_NMEA_RMC; ///exit
                }
            }/// end non-first time
            break;
        case UBLOX_NMEA_RMC:
            msgID.cfgClassID = (UBLOX_NMEA_RMC >> 8) & 0xFF;
            msgID.cfgMsgID   = UBLOX_NMEA_RMC & 0xFF;
            msgID.rateRatio  = 0;
            msgID.whatTagert = 4;  ///GPS port

            if ( (GPSData->GPSTopLevelConfig & (1 << GPSPORT_SUPPORT420)) == (1 << GPSPORT_SUPPORT420)) {
                msgID.rateRatio = 1;
            } else {
                msgID.rateRatio = 0;
            }

            pt2TmpFunc = &_generateSendCFG_MSG;

            if(firstFlag == 0) {
                _sendAcknowlegdeProcess(&msgID,
                                       GPSData,
                                       &firstFlag,
                                       pt2TmpFunc);
                firstFlag = 1;
            } else { ///after first time
                if (_sendAcknowlegdeProcess(&msgID,
                                           GPSData,
                                           &firstFlag,
                                           pt2TmpFunc)) {
                    firstFlag = 0; ///reset for next case
                    CFGIndex  = CFG_DONE; ///exit
                }
            }/// end non-first time
            break;
        case CFG_DONE:
        default:
			finish   = 1;
			CFGIndex = UBLOX_NAV_POSLLH;
            break;
	}
	return finish;
}

/** ****************************************************************************
 * @name getConnectedWithUnknownStatusUbloxGPS performs configuring steps for
 *       getting connected with an unknown ublox.
 * @author Dong An
 * @param [in] GPSData - GPS data structure to pull message from
 * @retval known version
 ******************************************************************************/
unsigned char  getConnectedWithUnknownStatusUbloxGPS(GpsData_t* GPSData)
{
	unsigned char        knownVersion    = 0;
/*
	static unsigned char firstFlag       = 0;
	ubloxIDTypeSTRUCT    msgID;
	static unsigned long lastSentoutTime = 0;
	long                 deltaFromLastSendout;
	static unsigned char sendCFGFlag     = 0;
	static unsigned char sendPRTFlag     = 0;
	static unsigned int  squenceProcess  = AUTO_BAUD_PROCESS;

	msgID.classID = (UBLOX_MON_VER >> 8) & 0xFF;
	msgID.msgID   = UBLOX_MON_VER & 0xFF;

	switch (squenceProcess) {
        case AUTO_BAUD_PROCESS:
			if ( autobaud(GPSData) ) {
				if (sendCFGFlag == 0)
                    squenceProcess = RESTORE_TO_FACTORY;
				else
                    squenceProcess = CONFIG_TO_57600;
			}
            break;
        case RESTORE_TO_FACTORY:
			if(sendCFGFlag == 0) {
				msgID.classID = (UBLOX_CFG_CFG >> 8) & 0xFF;
				msgID.msgID   = UBLOX_CFG_CFG & 0xFF;
		    	_generateSendCFG_CFG(&msgID,
                                    GPSData);
	 		    sendCFGFlag = 1;
			}

			if ( isGpsTxEmpty() ) {
	    		firstFlag = 0;/// reset
				squenceProcess = AUTO_BAUD_PROCESS;
			}
            break;
        case CONFIG_TO_57600:
	    	if( GPSData->GPSbaudRate == 3 ) {
				squenceProcess = QUERY_VERSION;
				firstFlag      = 0;
			} else {
				if( sendPRTFlag == 0 ) {
					msgID.classID = (UBLOX_CFG_PRT >> 8) & 0xFF;
					msgID.msgID   = UBLOX_CFG_PRT & 0xFF;
			    	_generateSendCFG_PRT(&msgID,
                                        GPSData);
					sendPRTFlag   = 1;
				}

				if ( isGpsTxEmpty() ) {
		    		firstFlag      = 0;/// reset
					sendPRTFlag    = 0;
					squenceProcess = AUTO_BAUD_PROCESS;
				}
	    	}
            break;
        case QUERY_VERSION:
			if(firstFlag == 0 && GPSData->isGPSFWVerKnown != 1) {
				flushGPSRecBuf(); ///clean up the buffer
				GPSData->isGPSFWVerKnown = 0;
				pollUbloxMsg(&msgID,
                             GPSData); ///reuest Version Message
				firstFlag       = 1;
				lastSentoutTime = GPSData->Timer100Hz10ms;
			} else {
				deltaFromLastSendout = GPSData->Timer100Hz10ms - lastSentoutTime;
			    if ( GPSData->isGPSFWVerKnown != 1 ) {
			    	if( deltaFromLastSendout>QUERY_TIMEOUT ) { ///try again
						flushGPSRecBuf(); ///clean up the buffer
						pollUbloxMsg(&msgID,
                                     GPSData); ///reuest Version Message
						lastSentoutTime = GPSData->Timer100Hz10ms;
					}/// end of timeout and try again
				}/// end of non-version received
				else {
					firstFlag      = 0;
					knownVersion   = 1;
					squenceProcess = AUTO_BAUD_PROCESS;
				}
			} /// end of non-first
            break;
	}
*/
	return knownVersion;
}

/** ****************************************************************************
 * @name decodeVersionMsg LOCAL parse ublox FW version message.
 * @author Dong An
 * @param [in] msg - message to parse
 * @param [in] msgLength - length of buffer
 * @param [in] GPSData - GPS data structure to pull message from
 * @retval N/A
 ******************************************************************************/
void _decodeVersionMsg(char          *msg,
                      unsigned int  *msgLength,
                      GpsData_t	*GPSData)
{
	char tmpBuff[10];
	int  i;

	for (i = 0; i < 4; i++)
        tmpBuff[i] = msg[6 + i];
	tmpBuff[i]='\0';

	GPSData->UbloxSoftwareVer = atof(tmpBuff);

	if ( GPSData->UbloxSoftwareVer < NAV2_ENABLED_VERSION )
        GPSData->ubloxOldVersion = 0;
	else
        GPSData->ubloxOldVersion = 1;	///0: old 1:new, CFG-NAV2 allowed
}

/** ****************************************************************************
 * @name _sendAcknowlegdeProcess performs sending CFG message and receiving ACK
 *       message.
 * @author Dong An
 * @param [in] cfgID - classs id
 * @param [in] GPSData - GPS data structure to pull message from
 * @param [in] firstCall - flag
 * @param [in] pt2MyFunc - callback function pointer
 * @retval acknowledge
 ******************************************************************************/
unsigned char _sendAcknowlegdeProcess(ubloxIDTypeSTRUCT *cfgID,
                                      GpsData_t     *GPSData,
                                      unsigned char     *firstCall,
                                      void (*pt2MyFunc)(ubloxIDTypeSTRUCT *cfgID,
                                                        GpsData_t     *GPSData))
{
	unsigned char        sendAcknowlegde = 0;
/*
	static unsigned long lastSentoutTime = 0;
	unsigned char        firstFlag       = *firstCall;
	long                 deltaFromLastSendout;

	if(firstFlag == 0) {
		GPSData->ubloxClassID = cfgID->classID;/// register the ID
		GPSData->ubloxMsgID   = cfgID->msgID;
		GPSData->reClassID    = 0; ///clear the received ID
		GPSData->reMsgID      = 0;
		if (GPSData->GPSConfigureOK <= 0 )
            flushGPSRecBuf(); ///clean up the buffer only during configuring

	    (*pt2MyFunc)(cfgID, GPSData);     ///calling the specific CFG-Msg

		lastSentoutTime = GPSData->Timer100Hz10ms;
		sendAcknowlegde = 0;
	} else { ///after first time
		deltaFromLastSendout = GPSData->Timer100Hz10ms - lastSentoutTime;

		if( GPSData->ubloxClassID == GPSData->reClassID &&
			GPSData->ubloxMsgID == GPSData->reMsgID ){
				GPSData->updateFlagForEachCall &= !(1 << GOT_UBLOX_ACK); ///reset
				sendAcknowlegde = 1;
		} else {
		    if( deltaFromLastSendout > QUERY_TIMEOUT ) { ///timeout try again
				if (GPSData->GPSConfigureOK <= 0 )
                    flushGPSRecBuf(); ///clean up the buffer only during configuring
			    (*pt2MyFunc)(cfgID, GPSData); ///calling the specific CFG-Msg
				lastSentoutTime = GPSData->Timer100Hz10ms;
			}
		}/// end of "not received ACK"
	}/// end if "after first time" send
*/    
	return sendAcknowlegde;
}

/** ****************************************************************************
 * @name _generateSendCFG_PRT  generate and send out CFG-PRT message.
 * @author Dong An
 * @param [in] cfgID - classs id
 * @param [in] GPSData - GPS data structure to pull message from
 * @param [in] firstCall - flag
 * @param [in] pt2MyFunc - callback function pointer
 * @retval N/A
 ******************************************************************************/
void _generateSendCFG_PRT(ubloxIDTypeSTRUCT *cfgID,
                          GpsData_t     *GPSData)
{
	int               i;
	char              tmpMsgPayload[MAX_UBLOX_BODY_LENGTH ];
	ubloxIDTypeSTRUCT tmpMsgID;

	tmpMsgID.classID = cfgID->classID;
	tmpMsgID.msgID   = cfgID->msgID;

	i = 0;					  ///offset
	tmpMsgPayload[i++] = 2;     /// 0:Port ID: 2 for the port connected with DSP
	tmpMsgPayload[i++] = 0;     /// 1:reserved
	tmpMsgPayload[i++] = 0;     /// 2:reserved
	tmpMsgPayload[i++] = 0;     /// 3:reserved
	unsigned long tmp  = 0;
	tmp |=(0x1 << 4) | (0x3 << 6) | (0x4 << 9) | (0x0 << 12);
		  ///unknown/8 bits /no parity/1 stop/ LSB first/16X over sampling
    tmpMsgPayload[i++] = tmp & 0xFF; ///4:
	tmpMsgPayload[i++] = (tmp >>  8) & 0xFF; ///5:
	tmpMsgPayload[i++] = (tmp >> 16) & 0xFF; ///6:
	tmpMsgPayload[i++] = (tmp >> 24) & 0xFF; ///7:

	tmp = 57600;
   	tmpMsgPayload[i++] = tmp & 0xFF; 		///4:
	tmpMsgPayload[i++] = (tmp >> 8) & 0xFF;   ///5:
	tmpMsgPayload[i++] = (tmp >> 16) & 0xFF; ///6:
	tmpMsgPayload[i++] = (tmp >> 24) & 0xFF; ///7:

	unsigned short tmp1 = 0x07;
   	tmpMsgPayload[i++] = tmp1&0xFF; 		///4:
	tmpMsgPayload[i++] = (tmp1 >> 8) & 0xFF; ///5:

	tmp1 = 0x03;
   	tmpMsgPayload[i++] = tmp1 & 0xFF; 		///4:
	tmpMsgPayload[i++] = (tmp1 >> 8) & 0xFF; ///5:

	tmp1 = 0x0;
   	tmpMsgPayload[i++] = tmp1 & 0xFF; 		///4:
	tmpMsgPayload[i++] = (tmp1 >> 8) & 0xFF; ///5:

	tmp1 = 0x0;
   	tmpMsgPayload[i++] = tmp1 & 0xFF; 		///4:
	tmpMsgPayload[i++] = (tmp1 >> 8) & 0xFF; ///5:

	ubloxGPSMsgSender(&tmpMsgID,
                      tmpMsgPayload,
                      &i,
                      GPSData);
}

/** ****************************************************************************
 * @name _generateSendCFG_CFG  generate and send out CFG-CFG message.
 * @author Dong An
 * @param [in] cfgID - classs id
 * @param [in] GPSData - GPS data structure to pull message from
 * @param [in] firstCall - flag
 * @param [in] pt2MyFunc - callback function pointer
 * @retval N/A
 ******************************************************************************/
void _generateSendCFG_CFG(ubloxIDTypeSTRUCT *cfgID,
                          GpsData_t     *GPSData)
{
	int               i;
	char              tmpMsgPayload[MAX_UBLOX_BODY_LENGTH ];
	ubloxIDTypeSTRUCT tmpMsgID;

	tmpMsgID.classID = cfgID->classID;
	tmpMsgID.msgID   = cfgID->msgID;

	i = 0;					  ///offset
	tmpMsgPayload[i++] = 0xFF;     /// 0:Port ID: 2 for the port connected with DSP
	tmpMsgPayload[i++] = 0xFF;     /// 1:reserved
	tmpMsgPayload[i++] = 0xFF;     /// 2:reserved
	tmpMsgPayload[i++] = 0xFF;     /// 3:reserved

	tmpMsgPayload[i++] = 0x0;     /// 0:Port ID: 2 for the port connected with DSP
	tmpMsgPayload[i++] = 0x0;     /// 1:reserved
	tmpMsgPayload[i++] = 0x0;     /// 2:reserved
	tmpMsgPayload[i++] = 0x0;     /// 3:reserved

	tmpMsgPayload[i++] = 0xFF;     /// 0:Port ID: 2 for the port connected with DSP
	tmpMsgPayload[i++] = 0xFF;     /// 1:reserved
	tmpMsgPayload[i++] = 0xFF;     /// 2:reserved
	tmpMsgPayload[i++] = 0xFF;     /// 3:reserved

	ubloxGPSMsgSender(&tmpMsgID,
                      tmpMsgPayload,
                      &i,
                      GPSData);
}

#endif // GPS
