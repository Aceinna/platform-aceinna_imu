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

#include "GlobalConstants.h"
#include "ucb_packet_struct.h"

// NEEDS TO BE CHECKED
//  Xbow Packet Code
typedef enum
{
    UCB_PING,               //  0 PK 0x504B input packets
    UCB_ECHO,               //  1 CH 0x4348
    UCB_GET_PACKET,         //  2 GP 0x4750
    UCB_SET_FIELDS,         //  3 SF 0x5346
    UCB_GET_FIELDS,         //  4 GF 0x4746
    UCB_READ_FIELDS,        //  5 RF 0x4246
    UCB_WRITE_FIELDS,       //  6 WF 0x5746
    UCB_UNLOCK_EEPROM,      //  7 UE 0x5545
    UCB_READ_EEPROM,        //  8 RE 0x5245
    UCB_WRITE_EEPROM,       //  9 WE 0x4745
    UCB_PROGRAM_RESET,      // 10 PR 0x5052
    UCB_SOFTWARE_RESET,     // 11 SR 0x5352
    UCB_WRITE_CAL,          // 12 WC 0x5743
    UCB_READ_CAL,           // 13 RC 0x5243
    UCB_WRITE_APP,          // 14 WA 0x5743
    UCB_JUMP2_BOOT,
    UCB_JUMP2_APP,
    UCB_J2BOOT,             // 15 JB 0x4A42
    UCB_J2IAP,              // 16 JI 0x4A49
    UCB_J2APP,              // 17 JA 0x4A41
    UCB_HARDWARE_TEST,       //    HT 0X4854
    UCB_INPUT_PACKET_MAX,
//**************************************************
    UCB_IDENTIFICATION,     // 18 ID 0x4944 output packets
    UCB_VERSION_DATA,       // 19 VR 0x4652
    UCB_VERSION_ALL_DATA,   // 20 VA 0x5641
    UCB_SCALED_0,           // 21 S0 0x5330
    UCB_SCALED_1,           // 22 S1 0x5331
    UCB_SCALED_M,           // 23 S1 0x534D
    UCB_TEST_0,             // 24 T0 0x5430
    UCB_FACTORY_1,          // 25 F1 0x4631
    UCB_FACTORY_2,          // 26 F2 0x4632
    UCB_FACTORY_M,          // 27 F3 0x464D
    UCB_ANGLE_1,            // 28 A1 0x4131
    UCB_MAG_CAL_1_COMPLETE, // 30 CB 0x4342
    UCB_MAG_CAL_3_COMPLETE, // 31 CD 0x4344
    UCB_PKT_NONE,           // 27   marker after last valid packet 
    UCB_NAK,                // 28
    UCB_ERROR_TIMEOUT,      // 29         
    UCB_ERROR_CRC_FAIL,     // 30 
} UcbPacketType;


#define UCB_IDENTIFICATION_LENGTH 69
#define UCB_VERSION_DATA_LENGTH 5
#define UCB_VERSION_ALL_DATA_LENGTH 15
#define UCB_ANGLE_1_LENGTH 32
#define UCB_ANGLE_2_LENGTH 30
#define UCB_ANGLE_3_LENGTH 30
#define UCB_ANGLE_4_LENGTH 42 // std A4 = 38 this is custom
#define UCB_ANGLE_5_LENGTH 62
#define UCB_ANGLE_U_LENGTH 42
#define UCB_SCALED_0_LENGTH 30
#define UCB_SCALED_1_LENGTH 24
#define UCB_SCALED_M_LENGTH 60
#define UCB_TEST_0_LENGTH 28
#define UCB_TEST_1_LENGTH 32
#define UCB_FACTORY_1_LENGTH 54
#define UCB_FACTORY_2_LENGTH 66
#define UCB_FACTORY_M_LENGTH 85
#define UCB_FACTORY_4_LENGTH 54
#define UCB_FACTORY_5_LENGTH 70
#define UCB_FACTORY_6_LENGTH 66
#define UCB_FACTORY_7_LENGTH 134
#define UCB_MAG_CAL_1_COMPLETE_LENGTH 4
#define UCB_MAG_CAL_3_COMPLETE_LENGTH 10
#define UCB_NAV_0_LENGTH 32
#define UCB_NAV_1_LENGTH 42
#define UCB_NAV_2_LENGTH 46 // with ITOW
#define UCB_KT_LENGTH 18

/// UCB packet-specific utility functions ucb_packet.c
extern UcbPacketType     UcbPacketBytesToPacketType    (const uint8_t bytes []);
extern void              UcbPacketPacketTypeToBytes    (UcbPacketType type, uint8_t bytes []);
extern BOOL UcbPacketIsAnInputPacket(UcbPacketType type);
extern BOOL UcbPacketIsAnOutputPacket(UcbPacketType type);

// send_packet.c
extern void SendUcbPacket(uint16_t port, UcbPacketStruct *ptrUcbPacket);
// handle packet.c
extern void HandleUcbPacket(UcbPacketStruct *ptrUcbPacket);
extern int  HandleUserInputPacket (UcbPacketStruct *ptrUcbPacket);
extern BOOL HandleUserOutputPacket (uint8_t *payload, uint8_t *payloadLen);


// Function used to write the Mag-Align parameters to the EEPROM by field
extern void WriteMagAlignParamsToMemory( uint16_t        port,
                                         UcbPacketStruct *ptrUcbPacket );
#endif
