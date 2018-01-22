/** ***************************************************************************
  * @file ucb_packet.h utility functions for interfacing with Memsic proprietary
  *       UCB (unified code base) packets.  UCB packet structure
  * @Author rhilles
  * @date  2010-08-03 10:20:52 -0700 (Tue, 03 Aug 2010)
  * @rev 15924
  * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
  *
  * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY O ANY
  * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
  * PARTICULAR PURPOSE.
  *
  * @brief these are in ucb_packet_types.def on the 440 these were in
  *        xbowProtocol.h
  *****************************************************************************/
#ifndef UCB_PACKET_H
#define UCB_PACKET_H
#include "crc.h"

#define MATCHES 0

#define UCB_SYNC_LENGTH				2
#define UCB_PACKET_TYPE_LENGTH		2
#define UCB_PAYLOAD_LENGTH_LENGTH	1
#define UCB_MAX_PAYLOAD_LENGTH		255
#define UCB_CRC_LENGTH				CRC_CCITT_LENGTH

#define UCB_SYNC_INDEX				0
#define UCB_PACKET_TYPE_INDEX		(UCB_SYNC_INDEX + UCB_SYNC_LENGTH)
#define UCB_PAYLOAD_LENGTH_INDEX    (UCB_PACKET_TYPE_INDEX + UCB_PACKET_TYPE_LENGTH)

/// preamble sync bytes
static const uint8_t UCB_SYNC [UCB_SYNC_LENGTH] = { 0x55, 0x55 };

/// packet field type definitions
typedef uint16_t       UcbPacketCodeType;
typedef CrcCcittType   UcbPacketCrcType;


typedef enum {
    UCB_PING,               //  0 PK 0x504B input packets
    UCB_ECHO,               //  1 CH 0x4348
    UCB_GET_PACKET,         //  2 GP 0x4750
    UCB_SET_FIELDS,         //  3 SF 0x5346
    UCB_GET_FIELDS,         //  4 GF 0x4746
    UCB_READ_FIELDS,        //  5 RF 0x4246
    UCB_WRITE_FIELDS,       //  6 WF 0x4746
    UCB_UNLOCK_EEPROM,      //  7 UE 0x5545
    UCB_READ_EEPROM,        //  8 RE 0x5245
    UCB_WRITE_EEPROM,       //  9 WE 0x4745
    UCB_PROGRAM_RESET,      // 10 PR 0x5052
    UCB_SOFTWARE_RESET,     // 11 SR 0x5352
    UCB_ALGORITHM_RESET,    // 12 AR 0x4152
    UCB_WRITE_CAL,          // 13 WC 0x4743
    UCB_IDENTIFICATION,     // 14 ID 0x4944 output packets
    UCB_VERSION_DATA,       // 15 VR 0x4652
    UCB_VERSION_ALL_DATA,   // 16 VA 0x5641
    UCB_ANGLE_1,            // 17 A1 0x4131
    UCB_ANGLE_2,            // 18 A2 0x4132
    UCB_ANGLE_4,            // 19 A5 0x4135
    UCB_ANGLE_5,            // 20 A5 0x4135
    UCB_ANGLE_U,            // 21 AU 0x4155
    UCB_SCALED_0,           // 22 S0 0x5330
    UCB_SCALED_1,           // 23 S1 0x5331
    UCB_TEST_0,             // 24 T0 0x5430
    UCB_TEST_1,             // 25 T1 0x5431
    UCB_FACTORY_1,          // 26 F1 0x4631
    UCB_FACTORY_2,          // 27 F2 0x4632
    UCB_FACTORY_3,          // 28 F3 0x4633
    UCB_MAG_CAL_1_COMPLETE, // 29 CB 0x4342
    UCB_MAG_CAL_3_COMPLETE, // 30 CD 0x4344
    UCB_NAV_0,              // 31 N0 0x4E30
    UCB_NAV_1,              // 32 N1 0x4E31
    UCB_NAV_2,              // 33 N2 0x4E32

    UCB_JUMP2_IAP,
    UCB_READ_APP,
    UCB_WRITE_APP,

    UCB_NAK,                // 32
    UCB_ERROR_INVALID_TYPE,	// 33 invalid packet type ID
    UCB_ERROR_TIMEOUT,      // 34 timeout reached before entire packet was received
    UCB_ERROR_CRC_FAIL,     // 35
	NUM_UCB_PACKET_TYPE_ENUM // 36
} UcbPacketType;

enum SystemType {
    IMU_6DOF_SYS     = 0,
    IMU_9DOF_SYS     = 1,
    UNAIDED_VG_SYS   = 2,
    UNAIDED_AHRS_SYS = 3,
    AIDED_VG_SYS     = 4,
    AIDED_AHRS_SYS   = 5,
    INS_SYS          = 6
};

enum SystemRange {
    _200_DPS_RANGE  = 0,
    _400_DPS_RANGE  = 1,
    _1000_DPS_RANGE = 2
};

typedef struct {
     uint8_t       systemType;
     uint8_t       spiAddress;
     uint8_t	   payloadLength;
     UcbPacketType packetType;
     uint8_t       payload[ UCB_MAX_PAYLOAD_LENGTH ];
} UcbPacketStruct;

#define UCB_IDENTIFICATION_LENGTH		69
#define UCB_VERSION_DATA_LENGTH			 5
#define UCB_VERSION_ALL_DATA_LENGTH		15
#define UCB_ANGLE_1_LENGTH				32
#define UCB_ANGLE_2_LENGTH				30
#define UCB_ANGLE_3_LENGTH				52
#define UCB_ANGLE_4_LENGTH			    42 // std A4 = 38 this is custom
#define UCB_ANGLE_5_LENGTH			    62
#define UCB_ANGLE_U_LENGTH			    42
#define UCB_SCALED_0_LENGTH			    30
#define UCB_SCALED_1_LENGTH			    24
#define UCB_SCALED_3_LENGTH			    54
#define UCB_TEST_0_LENGTH				28
#define UCB_TEST_1_LENGTH				32
#define UCB_FACTORY_1_LENGTH			54
#define UCB_FACTORY_2_LENGTH			66
#define UCB_FACTORY_3_LENGTH            38
#define UCB_FACTORY_4_LENGTH			54
#define UCB_FACTORY_5_LENGTH			70
#define UCB_FACTORY_6_LENGTH			66
#define UCB_FACTORY_7_LENGTH           134
#define UCB_MAG_CAL_1_COMPLETE_LENGTH	 4
#define UCB_MAG_CAL_3_COMPLETE_LENGTH	10
#define UCB_NAV_0_LENGTH			    32
#define UCB_NAV_1_LENGTH			    42
#define UCB_NAV_2_LENGTH			    46 // with ITOW

/// UCB packet-specific utility functions ucb_packet.c
extern uint8_t           UcbGetSysType                 (void);
extern uint8_t           UcbGetSysRange                (void);
extern UcbPacketType     UcbPacketBytesToPacketType    (const uint8_t bytes []);
extern void              UcbPacketPacketTypeToBytes    (UcbPacketType type, uint8_t bytes []);
extern uint8_t           UcbPacketBytesToPayloadLength (const uint8_t bytes []);
extern void              UcbPacketPayloadLengthToBytes (uint8_t type, uint8_t bytes []);
extern UcbPacketCrcType  UcbPacketBytesToCrc           (const uint8_t bytes []);
extern void              UcbPacketCrcToBytes           (const UcbPacketCrcType crc, uint8_t bytes []);
extern UcbPacketCrcType  UcbPacketCalculateCrc         (const uint8_t data [], uint16_t length, const UcbPacketCrcType seed);
extern BOOL              UcbPacketIsAnInputPacket      (UcbPacketType type);
extern BOOL              UcbPacketIsAnOutputPacket     (UcbPacketType type);

// send_packet.c
extern void SendUcbPacket   (uint16_t port, UcbPacketStruct *ptrUcbPacket);
extern void LoadUcbSPIBuffer( UcbPacketStruct *ptrUcbPacket);
// handle packet.c
extern void HandleUcbPacket (uint16_t port, UcbPacketStruct *ptrUcbPacket);
extern void SystemReset(void);

// Function used to write the Mag-Align parameters to the EEPROM by field
extern void WriteMagAlignParamsToMemory( uint16_t        port,
                                         UcbPacketStruct *ptrUcbPacket );
#endif
