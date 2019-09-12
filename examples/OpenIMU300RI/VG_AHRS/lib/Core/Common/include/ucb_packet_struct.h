/** ***************************************************************************
  * @file ucb_packet_struct.h utility functions for interfacing with Aceinna proprietary
  *       UCB (unified code base) packets.  UCB packet structure
  *
  * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY O ANY
  * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
  * PARTICULAR PURPOSE.
  *
  * @brief these are in ucb_packet_types.def on the 440 these were in
  *        xbowProtocol.h
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

#ifndef UCB_PACKET_STRUCT_H
#define UCB_PACKET_STRUCT_H
#include <stdint.h>

#define UCB_SYNC_LENGTH				2
#define UCB_PACKET_TYPE_LENGTH		2
#define UCB_PAYLOAD_LENGTH_LENGTH	1
#define UCB_CRC_LENGTH				2

#define UCB_SYNC_INDEX				0
#define UCB_PACKET_TYPE_INDEX		(UCB_SYNC_INDEX + UCB_SYNC_LENGTH)
#define UCB_PAYLOAD_LENGTH_INDEX    (UCB_PACKET_TYPE_INDEX + UCB_PACKET_TYPE_LENGTH)

/// preamble sync bytes
static const uint8_t UCB_SYNC [UCB_SYNC_LENGTH] = { 0x55, 0x55 };

/// packet field type definitions
typedef uint16_t       UcbPacketCodeType;
typedef uint16_t       UcbPacketCrcType;


#define UCB_MAX_PAYLOAD_LENGTH		255
#define UCB_USER_IN                 200         
#define UCB_USER_OUT                201      
#define UCB_ERROR_INVALID_TYPE      202     



typedef struct {
    uint16_t        packetType;
    uint16_t        packetCode;
}ucb_packet_t;

typedef struct {
    uint16_t        packetType;
    uint8_t         packetCode[2];
}usr_packet_t;


typedef struct {
     uint8_t       packetType;      // 0
     uint8_t       systemType;      // 1
     uint8_t       spiAddress;      // 2
     uint8_t       sync_MSB;        // 3
     uint8_t       sync_LSB;        // 4
     uint8_t       code_MSB;        // 5
     uint8_t       code_LSB;        // 6
     uint8_t	   payloadLength;   // 7
     uint8_t       payload[UCB_MAX_PAYLOAD_LENGTH + 3]; // aligned to 4 bytes 
} UcbPacketStruct;

#define NUM_ARRAY_ITEMS(a)	(sizeof(a) / sizeof(a[0]))
#define CODE(first, second)	(((first << 8) | second) & 0xffff)
#define SWAP(x)	            (((x >> 8) & 0xff) | ((x << 8) & 0xff00))

extern int getUserPayloadLength(void);
extern int checkUserPacketType(UcbPacketCodeType receivedCode);
extern void  userPacketTypeToBytes(uint8_t bytes[]);


#endif
