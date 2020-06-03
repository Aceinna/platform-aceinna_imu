/** ***************************************************************************
 * @file   user_message.h
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 ******************************************************************************/
/*******************************************************************************
Copyright 2020 ACEINNA, INC

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

#ifndef _USER_MESSAGE_H
#define _USER_MESSAGE_H

#include <stdint.h>

#include "constants.h"
#include "ucb_packet.h"


// total size of user packet structure should not exceed 255 bytes
#pragma pack(1)

// example of user payload structure
typedef struct {
    uint32_t   paramNum;                                             // parameter number in parameters structure   (little endian)
    uint8_t    parameter[64];                                        // up to 30 64-bit parameters  (little endian)
} userParamPayload;

#pragma pack()

typedef enum
{
    USR_IN_NONE = 0,
    USR_IN_PING,
    USR_IN_UPDATE_PARAM,
    USR_IN_SAVE_CONFIG,
    USR_IN_GET_ALL,
    USR_IN_GET_VERSION,
    USR_IN_MAX,
} UserInPacketType;

// User output packet codes, change at will
typedef enum {
    USR_OUT_NONE  = 0,
    USR_OUT_SCALED1,
    USR_OUT_POS,
    USR_OUT_SKY,
    USR_OUT_MAX
} UserOutPacketType;

//  user out packet struct
#pragma pack(1)

// payload structure of alternative IMU data message
typedef struct {
    uint32_t  week;
    double   timeOfWeek;
    float    accel_g[3];
    float    rate_dps[3];
    //float    temp_C;
} scaled1_payload_t;

// payload structure of standard pos data message
typedef struct {
    uint32_t week;
    double   timeOfWeek;
    uint32_t positionMode;
    double   latitude;
    double   longitude;
    double   height;
    uint32_t numberOfSVs;
    float    hdop;
    uint32_t velocityMode;
    float    velocityNorth;
    float    velocityEast;
    float    velocityDown;
    uint32_t insStatus;
    uint32_t insPositionType;
    float    roll;
    float    pitch;
    float    heading;
} pos_payload_t;

// payload structure of standard skyview data message
typedef struct {
    double   timeOfWeek;
    uint8_t  satelliteId;
    uint8_t  systemId;
    uint8_t  antennaId;
    uint8_t  l1cn0;
    uint8_t  l2cn0;
    float    azimuth;
    float    elevation;
} skyview_payload_t;

#pragma pack()


int checkUserPacketType(uint16_t receivedCode);
int checkUserOutPacketType(uint16_t receivedCode);
void userPacketTypeToBytes(uint8_t bytes[]);

BOOL setUserPacketType(uint8_t *data, BOOL fApply);

int  HandleUserInputPacket (UcbPacketStruct *ptrUcbPacket);
BOOL HandleUserOutputPacket(uint8_t *payload, uint8_t *payloadLen);

#endif /* _USER_MESSAGE_H */
