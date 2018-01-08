/** ***************************************************************************
 * @file ucb_packet.c utility functions for interfacing with Memsic proprietary
 *       UCB (unified code base) packets
 * @brief UCB Packet Type (char to byte) eg 'P' 'K' -> 0x504b or bytes to chars
 *        and CRC calculation and conversion
 * @Author rhilles
 * @date   September, 2008
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY O ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 * @version
 * 10.02.14 DKH added UcbGetSysRange() changed UcbSetSysType() to UcbGetSysType()
 * 04.20.15 DKH changed to substring search for sys type and hi-lo range
 *****************************************************************************/
#include <string.h> // strcmp, strstr

#include "stm32f2xx.h"
#include "dmu.h"
#include "ucb_packet.h"
#include "xbowsp_generaldrivers.h"

#define CODE(first, second)	(((first << 8) | second) & 0xffff)

static const UcbPacketCodeType UCB_PACKET_CODE [] = {
    CODE('P','K'), // 0x504B
    CODE('C','H'), // 0x4348
    CODE('G','P'), // 0x4750
    CODE('S','F'), // 0x5346
    CODE('G','F'), // 0x4746
    CODE('R','F'), // 0x5246
    CODE('W','F'), // 0x5746
    CODE('U','E'), // 0x5545
    CODE('R','E'), // 0x5245
    CODE('W','E'), // 0x4745
    CODE('P','R'), // 0x5052
    CODE('S','R'), // 0x5352
    CODE('A','R'), // 0x4152
    CODE('W','C'), // 0x5743
    CODE('I','D'), // 0x4944
    CODE('V','R'), // 0x5652
    CODE('V','A'), // 0x5641
    CODE('A','1'), // 0x4131
    CODE('A','2'), // 0x4132
    CODE('A','4'), // 0x4134
    CODE('A','5'), // 0x4135
    CODE('A','U'), // 0x4155
    CODE('S','0'), // 0x5330
    CODE('S','1'), // 0x5331
    CODE('T','0'), // 0x5430
    CODE('T','1'), // 0x5431
    CODE('F','1'), // 0x4631
    CODE('F','2'), // 0x4632
    CODE('F','3'), // 0x4633
    CODE('C','B'), // 0x4342
    CODE('C','D'), // 0x4344
    CODE('N','0'), // 0x4E30
    CODE('N','1'), // 0x4E31
    CODE('N','2'),  // 0x4E32
	CODE('J','I'),
	CODE('R','A'),
	CODE('W','A')
};

#define NUM_ARRAY_ITEMS(a)	(sizeof(a) / sizeof(a[0]))

static const uint16_t NUM_PACKET_CODES = NUM_ARRAY_ITEMS(UCB_PACKET_CODE);

/** ****************************************************************************
 * @name UcbGetSysType
 * @brief return the system type for message limiting
 * Trace:
 * @param N/A
 * @Retval N/A
 ******************************************************************************/
uint8_t UcbGetSysType (void)
{
    char* pString;
    uint8_t sysType;

    // All system will be considered 6DOF system unless otherwise detected
    sysType = IMU_6DOF_SYS; // default

    // "IMU380ZA-209"
    // "IMU380ZA-409"
    // "IMU380ZA-1009"
    // "IMU380SA-209"
    // "IMU380SA-409"
    pString = strstr(gCalibration.versionString, "09");
    if ( pString ) {
        sysType = IMU_9DOF_SYS;
    }

    // "VG380ZA-200"
    // "VG380SA-200"
    // "VG380ZA-400"
    // "VG380SA-400"
    pString = strstr(gCalibration.versionString, "VG");
    if ( pString ) {
        if( gConfiguration.userBehavior.bit.useGPS ) {
            sysType = AIDED_VG_SYS;
        } else {
            sysType = UNAIDED_VG_SYS;
        }
    }
    
    pString = strstr(gCalibration.versionString, "MTLT");
    if ( pString ) {
        if( gConfiguration.userBehavior.bit.useGPS ) {
            sysType = AIDED_VG_SYS;
        } else {
            sysType = UNAIDED_VG_SYS;
        }
    }
    // "AHRS380ZA-200"
    // "AHRS380ZA-400"
    // "AHRA380SA-200"
    pString = strstr(gCalibration.versionString, "AHRS");
    if ( pString ) {
        if( gConfiguration.userBehavior.bit.useGPS ) {
            sysType = AIDED_AHRS_SYS;
        } else {
            sysType = UNAIDED_AHRS_SYS;
        }
    }

    //"INS380ZA-200"
    //"INS380ZA-400"
    //"INS380SA-300"
    pString = strstr(gCalibration.versionString, "INS");
    if ( pString ) {
        sysType = INS_SYS;
    }

    return sysType;
}

/** ****************************************************************************
 * @name UcbGetSysRange return the system range for sensor config
 * @brief sensor config is set using config file downloaded from NavView
 * Trace:
 * @param N/A
 * @Retval N/A
 ******************************************************************************/
uint8_t UcbGetSysRange (void)
{
    char* pString;
    uint8_t sysRange = _200_DPS_RANGE;

    // "IMU380ZA-200"
    // "INS380ZA-200"
    // "AHRS380ZA-200"
    // "VG380ZA-200"
    // "IMU380ZA-209"
    // "INS380SA-200"
    // "AHRS380SA-200"
    // "VG380SA-200"
    // "IMU380SA-209"
    pString = strstr(gCalibration.versionString, "20");
    if ( pString ) {
        sysRange = _200_DPS_RANGE;
    }

    // "INS380ZA-210"
    // "INS380SA-210"
    pString = strstr(gCalibration.versionString, "21");
    if ( pString ) {
        sysRange = _200_DPS_RANGE;
    }

    // "INS380ZA-400"
    // "AHRS380ZA-400"
    // "VG380ZA-400"
    // "IMU380ZA-409"
    // "INS380SA-400"
    // "AHRS380SA-400"
    // "VG380SA-400"
    // "IMU380SA-409"
    pString = strstr(gCalibration.versionString, "40");
    if ( pString ) {
        sysRange = _400_DPS_RANGE;
    }

    // "INS380ZA-410"
    // "INS380SA-410"
    pString = strstr(gCalibration.versionString, "41");
    if ( pString ) {
        sysRange = _400_DPS_RANGE;
    }

    // "IMU380ZA-1000"
    // "IMU380ZA-1009"
    pString = strstr(gCalibration.versionString, "100");
    if ( pString ) {
        sysRange = _1000_DPS_RANGE;
    }

    return sysRange;
}


/** ****************************************************************************
 * @name UcbPacketBytesToPacketType
 * @brief Convert the packet bytes into the packet type enum eg "PK" -> 0x504B
 * Trace:
 * [SDD_UCB_UNKNOWN_01 <-- SRC_UCB_PKT_ENUM]
 * [SDD_HANDLE_PKT  <-- SRC_UCB_PKT_ENUM]
 * [SDD_UCB_VALID_PACKET <-- SRC_UCB_PKT_ENUM]
 * @param [in] byte array, containing one byte
 * @Retval packet type enum
 ******************************************************************************/
UcbPacketType UcbPacketBytesToPacketType (const uint8_t bytes [])
{
	UcbPacketType packetType = 0;
	UcbPacketCodeType receivedCode = (UcbPacketCodeType)(((bytes[0] & 0xff) << 8) |
                                                          (bytes[1] & 0xff));
    BOOL valid = FALSE;

    /// search through the packet code table for a matching code - check type
    /// against valid types
	while ((packetType < NUM_PACKET_CODES) && (valid == FALSE)) {
        if ( (receivedCode == UCB_PACKET_CODE[packetType]) && (packetType != UCB_NAK) ) {
			valid = TRUE;
        } else {
			++packetType;
		}
	}

	if (valid == FALSE) {
		packetType = UCB_ERROR_INVALID_TYPE;
	}
	return packetType;
}
/* end UcbPacketBytesToPacketType */

/** ****************************************************************************
 * Function name:	UcbPacketBytesToPacketType
* @brief Convert the packet type enum into bytes 0x504B -> ['P'] ['K']
 * Trace:
 * [SDD_UCB_UNKNOWN_01 <-- SRC_UCB_PKT_STR]
 * [SDD_HANDLE_PKT  <-- SRC_UCB_PKT_STR]
 * @param [in] byte array, containing one byte
 * @Retval length
 ******************************************************************************/
void UcbPacketPacketTypeToBytes (UcbPacketType type,
                                 uint8_t       bytes [])
{
    if (type < NUM_PACKET_CODES) {
        bytes[0] = (uint8_t)((UCB_PACKET_CODE[type] >> 8) & 0xff);
        bytes[1] = (uint8_t)(UCB_PACKET_CODE[type] & 0xff);
	} else {
		bytes[0] = 0;
		bytes[1] = 0;
	}
}
/* end UcbPacketPacketTypeToBytes */

/** ****************************************************************************
 * Function name:	UcbPacketBytesToPayloadLength
 * @brief Convert the packet bytes into the packet payload length
 * Trace:
 * [SDD_UCB_STORE_DATA <-- SRC_UCB_PAYLEN]
 * [SDD_UCB_PKT_PAYLEN <-- SRC_UCB_PAYLEN]
 * @param [in] byte array, containing one byte
 * @Retval length
 ******************************************************************************/
uint8_t UcbPacketBytesToPayloadLength (const uint8_t bytes [])
{
	return (uint8_t)(bytes[0] & 0xff);
}
/* end UcbPacketBytesToPayloadLength */

/** ****************************************************************************
 * @name UcbPacketPayloadLengthToBytes
 * @brief	Convert the payload length into bytes
 * Trace:
 * [SDD_UCB_PROCESS_OUT <-- SRC_UCB_PKT_LENBYT]
 * [SDD_UCB_PKT_LENBYT <-- SRC_UCB_PKT_LENBYT]
 * @param [in] type - payload type
 * @param [out] byte array, containing one byte
 * @Retval none
 ******************************************************************************/
void	UcbPacketPayloadLengthToBytes (uint8_t length,
                                       uint8_t bytes [])
{
	bytes[0] = (uint8_t)(length & 0xff);
}
/* end UcbPacketBytesToPayloadLength */

/** ****************************************************************************
 * @name UcbPacketBytesToCrc
 * Description:
 * @param [in] byte array with byte[0] containing input value
 * @retval N/A
 ******************************************************************************/
UcbPacketCrcType UcbPacketBytesToCrc (const uint8_t bytes [])
{
	return ((UcbPacketCrcType)(((bytes[0] & 0xff) << 8) | (bytes[1] & 0xff)));
}
/* end UcbPacketBytesToCrc */

/** ****************************************************************************
 * @name UcbPacketCrcToBytes
 * @broef This function converts a value into a 2 byte string.
 * Trace: [SDD_UCB_PROCESS_OUT <-- SRC_UCB_PKT_CRCSTR]
 * @param [in] crc - 16-bit value (crc value)
 * @param [out] byte array with byte[0] containing the upper byte of input value
 * @retval N/A
 ******************************************************************************/
void UcbPacketCrcToBytes (const UcbPacketCrcType crc,
                          uint8_t                bytes [])
{
	bytes[0] = (uint8_t)((crc >> 8) & 0xff);
	bytes[1] = (uint8_t)(crc & 0xff);
}
/* end UcbPacketCrcToBytes */

/** ****************************************************************************
 * @name UcbPacketCalculateCrc
 * @brief This function returns the CCITT-16 CRC on the passed in data of
 *               length given with an initial CRC value.
 * Trace: [SDD_UCB_CRC_FAIL_01  <-- SRC_CRC_UCB_PKT]
 *        [SDD_UCB_VALID_PACKET <-- SRC_CRC_UCB_PKT]
 * @param [in] data[] - byte array to CRC
 * @param [in] length - number of bytes in byte array
 * @param [in] seed - initial or continuing value of CRC
 * @retval 16 bit CRC
 ******************************************************************************/
UcbPacketCrcType UcbPacketCalculateCrc (const uint8_t          data [],
                                        uint16_t               length,
                                        const UcbPacketCrcType seed)
{
	return (CrcCcitt(data, length, seed));
}
/* end UcbPacketCalculateCrc */

/** ****************************************************************************
 * @name UcbPacketIsAnInputPacket
 * @brief Returns TRUE if given packet type is an input packet type
 * Trace:
 * [SDD_UCB_UNKNOWN_01 <-- SRC_UCB_PKT_INTYPE]
 * [SDD_UCB_UNKNOWN_02 <-- SRC_UCB_PKT_INTYPE]
 * [SDD_UCB_VALID_PACKET <-- SRC_UCB_PKT_INTYPE]
 * @parame [in]	UCB packet type enum
 * @retval TRUE if output packet type, FALSE otherwise
 ******************************************************************************/
BOOL UcbPacketIsAnInputPacket (UcbPacketType type)
{
	BOOL isAnInputPacket;

	switch (type) {
        case UCB_PING:
        case UCB_ECHO:
        case UCB_GET_PACKET:
        case UCB_SET_FIELDS:
        case UCB_GET_FIELDS:
        case UCB_READ_FIELDS:
        case UCB_WRITE_FIELDS:
        case UCB_UNLOCK_EEPROM:
        case UCB_READ_EEPROM:
        case UCB_WRITE_EEPROM:
        case UCB_PROGRAM_RESET:
        case UCB_SOFTWARE_RESET:
        case UCB_ALGORITHM_RESET:
        case UCB_WRITE_CAL:
            isAnInputPacket = TRUE;
            break;
		default:
          isAnInputPacket = FALSE;
	}

	return isAnInputPacket;
}
/* end UcbPacketIsAnInputPacket */

/** ****************************************************************************
 * @name UcbPacketIsAnOutputPacket API
 * @brief Returns TRUE if given packet type is an output packet type
 * Trace: [SDD_PORT_CFG_VALID_03 <-- SRC_UCB_PKT_OUTTYPE]
 * @param [in] UCB packet type
 * @retval TRUE if output packet type, FALSE otherwise
 ******************************************************************************/
BOOL UcbPacketIsAnOutputPacket (UcbPacketType type)
{
	BOOL isAnOutputPacket;

	switch (type) {
        case UCB_IDENTIFICATION:
        case UCB_VERSION_DATA:
        case UCB_VERSION_ALL_DATA:
        case UCB_SCALED_0:
        case UCB_SCALED_1:
        case UCB_TEST_0:
        case UCB_TEST_1:
        case UCB_FACTORY_1:
        case UCB_FACTORY_2:
        case UCB_FACTORY_3:
            isAnOutputPacket = TRUE;
            break;
        case UCB_ANGLE_1:
        case UCB_ANGLE_2:
        case UCB_ANGLE_5:
        case UCB_ANGLE_U:
// If an IMU then this packet is not valid
            if( UcbGetSysType() <= IMU_9DOF_SYS ) {
                isAnOutputPacket = FALSE;
            } else {
                isAnOutputPacket = TRUE;
            }
            break;
        case UCB_NAV_0:
        case UCB_NAV_1:
        case UCB_NAV_2:
            if( UcbGetSysType() == INS_SYS ) {
                isAnOutputPacket = TRUE;
            } else {
                isAnOutputPacket = FALSE;
            }
            isAnOutputPacket = TRUE;   // FIXME: remove for release (JSM)
            break;
		default:
          isAnOutputPacket = FALSE;
	}
	return isAnOutputPacket;
}
/* end UcbPacketIsAnOutputPacket */

