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



#define MAX_NUMBER_OF_USER_PARAMS_IN_THE_PACKET 30

// total size of user packet structure should not exceed 255 bytes
#pragma pack(1)

typedef struct
{
    uint8_t  packetPayload[252];    // maximum 252 bytes     
} userPacket;

// example of user payload structure
typedef struct
{
    uint32_t   numParams;                                            // number of consecutive parameters to update (little endian)
    uint32_t   paramOffset;                                          // parameter number in parameters structure   (little endian)
    uint64_t   parameters[MAX_NUMBER_OF_USER_PARAMS_IN_THE_PACKET];  // up to 30 64-bit parameters  (little endian)
} userConfigPayload;

// example of user payload structure
typedef struct {
    uint32_t   paramNum;                                             // parameter number in parameters structure   (little endian)
    uint8_t    parameter[64];                                        // up to 30 64-bit parameters  (little endian)
} userParamPayload;

// example of user payload structure
typedef struct {
    uint64_t   parameters[MAX_NUMBER_OF_USER_PARAMS_IN_THE_PACKET];  // up to 30 64-bit parameters  (little endian)
} allUserParamsPayload;

#pragma pack()

#define USER_OK      0x00
#define USER_NAK     0x80
#define USER_INVALID 0x81

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
    int32_t  week;
    double   timeOfWeek;
    float    accel_g[3];
    float    rate_dps[3];
    //float    temp_C;
} scaled1_payload_t;

// payload structure of standard pos data message
typedef struct {
    uint32_t systemTime;
    double   timeOfWeek;
    uint32_t positionMode;
    double   latitude;
    double   longitude;
    double   ellipsoidalHeight;
    double   mslHeight;
    double   positionRMS;
    uint32_t velocityMode;
    float    velocityNorth;
    float    velocityEast;
    float    velocityDown;
    float    velocityRMS;
    uint32_t insStatus;
    uint32_t insPositionType;
    float    roll;
    float    pitch;
    float    heading;
    float    rollRMS;
    float    pitchRMS;
    float    headingRMS;
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


extern int       checkUserPacketType(uint16_t receivedCode);
extern int       checkUserOutPacketType(uint16_t receivedCode);
extern void      userPacketTypeToBytes(uint8_t bytes[]);

extern int  HandleUserInputPacket (UcbPacketStruct *ptrUcbPacket);
extern BOOL HandleUserOutputPacket(uint8_t *payload, uint8_t *payloadLen);

#endif /* _USER_MESSAGE_H */
