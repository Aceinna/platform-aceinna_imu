/*******************************************************************************
 * File:   UserConfiguration.h
 * Created on Jan 25, 2017
 ******************************************************************************/
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

#ifndef USER_MESSAGING_H
#define USER_MESSAGING_H

#include <stdint.h>

#include "GlobalConstants.h"
#include "ucb_packet_struct.h"

#define USER_PACKET_OK      0
#define UNKNOWN_USER_PACKET 1
#define USER_PACKET_ERROR   2

// here is definition for packet rate divider
// considering that data acquisition task runs at 200 Hz 
typedef enum {
    PACKET_RATE_INVALID = -1,
    PACKET_RATE_QUIET = 0,      // quiet mode
    PACKET_RATE_200HZ = 200,     // packet rate 200 Hz
    PACKET_RATE_100HZ = 100,     // packet rate 100 Hz
    PACKET_RATE_50HZ  = 50,      // packet rate 50 Hz
    PACKET_RATE_25HZ  = 25,      // packet rate 25 Hz
    PACKET_RATE_20HZ  = 20,      // packet rate 20 Hz
    PACKET_RATE_10HZ  = 10,      // packet rate 10 Hz
    PACKET_RATE_5HZ   = 5,       // packet rate 5  Hz
    PACKET_RATE_2HZ   = 2,       // packet rate 2  Hz
    PACKET_RATE_1HZ   = 1,       // packet rate 1  Hz
}packet_rate_t;


// User Input packet payload has next structure:
// number      offset
// of          of  first 
// parameters  parameter    
// U2          U2          U4/I4/F    
// XXXX        YYYY       [parameters]
// User input packet codes, change at will
typedef enum {
    USR_IN_NONE         = 0 ,
    USR_IN_PING             ,     
    USR_IN_UPDATE_CONFIG    ,
    USR_IN_UPDATE_PARAM     ,
    USR_IN_UPDATE_ALL       ,
    USR_IN_SAVE_CONFIG      ,
    USR_IN_RESTORE_DEFAULTS ,
    USR_IN_GET_CONFIG       ,
    USR_IN_GET_PARAM        ,
    USR_IN_GET_ALL          ,
    USR_IN_GET_VERSION      ,
    USR_IN_RESET            ,
    // add new packet type here, before USR_IN_MAX
    USR_IN_MAG_ALIGN        ,
    USR_IN_MAX
}UserInPacketType;

// User output packet codes, change at will
typedef enum {
    USR_OUT_NONE  = 0,
    USR_OUT_TEST,
    USR_OUT_DATA1,
    USR_OUT_ANG1,
    USR_OUT_ANG2,
// add new output packet type here, before USR_OUT_MAX
    USR_OUT_SCALED1,
    USR_OUT_EKF1,
    USR_OUT_MAX
} UserOutPacketType;


// total size of user packet structure should not exceed 255 bytes
#pragma pack(1)
typedef struct {
    uint8_t  packetPayload[252];    // maximum 252 bytes     
}userPacket;
#define MAX_NUMBER_OF_USER_PARAMS_IN_THE_PACKET 30
#define FIRST_30_PARAMS 0xFFFFFFFF

// example of user payload structure
typedef struct {
    uint32_t   numParams;                                            // number of consecutive parameters to update (little endian)
    uint32_t   paramOffset;                                          // parameter number in parameters structure   (little endian)
    uint64_t   parameters[MAX_NUMBER_OF_USER_PARAMS_IN_THE_PACKET];  // up to 30 64-bit parameters  (little endian)
}userConfigPayload;

#pragma pack(1)
// example of user payload structure
typedef struct {
    uint32_t   paramNum;                                             // parameter number in parameters structure   (little endian)
    uint64_t   parameter;                                            // up to 30 64-bit parameters  (little endian)
}userParamPayload;
#pragma pack()

// example of user payload structure
typedef struct {
    uint64_t   parameters[MAX_NUMBER_OF_USER_PARAMS_IN_THE_PACKET];  // up to 30 64-bit parameters  (little endian)
}allUserParamsPayload;

#pragma pack()


#define USR_OUT_TEST_PAYLOAD_LEN    (4)    // test parameter (uint32_t)    
#define USR_OUT_DATA1_PAYLOAD_LEN   (4*9)  // 3accels (float LE) + 3gyros (float LE) + 3 mags (floatLE)    
#define USR_OUT_ANG1_PAYLOAD_LEN    (47)   // See message loading code,HandleUserOutputPacket(), for information
#define USR_OUT_ANG2_PAYLOAD_LEN    (48)
#define USR_OUT_SCALED1_PAYLOAD_LEN (52)
#define USR_OUT_EKF1_PAYLOAD_LEN    (75)

#define USER_OK      0x00
#define USER_NAK     0x80
#define USER_INVALID 0x81

extern int userPacketOut;

extern int       getUserPayloadLength(void);
extern int       checkUserPacketType(uint16_t receivedCode);
extern void      userPacketTypeToBytes(uint8_t bytes[]);
extern void      WriteResultsIntoOutputStream(void *results);

#endif /* USER_CONFIGURATION_H */
