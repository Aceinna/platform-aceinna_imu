/** ***************************************************************************
 * @file   user_message.c
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

#include <string.h>
#include <stdint.h>
#include <stdio.h>

#include "app_version.h"
#include "platformAPI.h"
#include "configurationAPI.h"
#include "sensorsAPI.h"
#include "calibrationAPI.h"
#include "ucb_packet.h"
#include "user_message.h"
#include "user_config.h"
#include "timer.h"
#include "rtklib_core.h"
#include "rtcm.h"

/// List of allowed packet codes 
usr_packet_t userInputPackets[] = {
    {USR_IN_NONE,               {0,0}},
    {USR_IN_PING,               "pG"},
    {USR_IN_UPDATE_PARAM,       "uP"},
    {USR_IN_SAVE_CONFIG,        "sC"},
    {USR_IN_GET_ALL,            "gA"},
    {USR_IN_GET_VERSION,        "gV"}, 
// place new input packet code here, before USR_IN_MAX
    {USR_IN_MAX,                {0xff, 0xff}},   //  "" 
};


// packet codes here should be unique - 
// should not overlap codes for input packets and system packets
// First byte of Packet code should have value  >= 0x61  
usr_packet_t userOutputPackets[] = {	
//   Packet Type                Packet Code
    {USR_OUT_NONE,              {0x00, 0x00}}, 
    {USR_OUT_SCALED1,           "s1"},
    {USR_OUT_POS,               "pS"},
    {USR_OUT_SKY,               "sK"},
// place new type and code here
    {USR_OUT_MAX,               {0xff, 0xff}},   //  "" 
};

static   int    _outputPacketType  = USR_OUT_MAX;
static   int    _inputPacketType   = USR_IN_MAX;


int checkUserPacketType(uint16_t receivedCode)
{
    int res     = UCB_ERROR_INVALID_TYPE;
    usr_packet_t *packet  = &userInputPackets[1];
    uint16_t code;

    // validate packet code here and memorise for further processing
    while(packet->packetType != USR_IN_MAX){
        code = (packet->packetCode[0] << 8) | packet->packetCode[1];
        if(code == receivedCode){
            _inputPacketType = packet->packetType;
            return UCB_USER_IN;
        }
        packet++;
    }

    packet  = &userOutputPackets[1];
    
    // validate packet code here and memorize for further processing
    while(packet->packetType != USR_OUT_MAX){
        code = (packet->packetCode[0] << 8) | packet->packetCode[1];
        if(code == receivedCode){
            _outputPacketType = packet->packetType;
            return UCB_USER_OUT;
        }
        packet++;
    }

    return res;
}

int checkUserOutPacketType(uint16_t receivedCode)
{
    usr_packet_t *packet  = &userOutputPackets[1];
    uint16_t code;

    while(packet->packetType != USR_OUT_MAX){
        code = (packet->packetCode[0] << 8) | packet->packetCode[1];
        if(code == receivedCode){
            return UCB_USER_OUT;
        }
        packet++;
    }
    return UCB_ERROR_INVALID_TYPE;
}

void   userPacketTypeToBytes(uint8_t bytes[])
{
    if(_inputPacketType && _inputPacketType <  USR_IN_MAX){
        // response to request. Return same packet code
        bytes[0] = userInputPackets[_inputPacketType].packetCode[0];
        bytes[1] = userInputPackets[_inputPacketType].packetCode[1];
        _inputPacketType = USR_IN_MAX;  // wait for next input packet
        return;
    }
    
    if(_outputPacketType && _outputPacketType < USR_OUT_MAX){
        // continuous packet
        bytes[0] = userOutputPackets[_outputPacketType].packetCode[0];
        bytes[1] = userOutputPackets[_outputPacketType].packetCode[1];
    } else {
        bytes[0] = 0;
        bytes[1] = 0;
    }

}

/******************************************************************************
 * @name  FillPingPacketPayload - API call ro prepare user output packet
 * @brief general handler
 * @param [in] payload pointer to put user data to
 * @param [in/out] number of bytes in user payload
 * @retval N/A
 ******************************************************************************/
static BOOL Fill_PingPacketPayload(uint8_t *payload, uint8_t *payloadLen)
{
    int len; 
    uint8_t *model           = (uint8_t*)GetUnitVersion();
    uint8_t *rev             = (uint8_t*)platformBuildInfo();
    unsigned int serialNum   = GetUnitSerialNum();
    len = snprintf((char*)payload, 250, "%s %s %s SN:%u", PRODUCT_NAME_STRING, model, rev, serialNum );
    *payloadLen = len;
    return TRUE;
}

/******************************************************************************
 * @name FillVersionPacketPayload - API call ro prepare user output packet
 * @brief general handler
 * @param [in] payload pointer to put user data to
 * @param [in/out] number of bytes in user payload
 * @retval N/A
 ******************************************************************************/
static BOOL Fill_VersionPacketPayload(uint8_t *payload, uint8_t *payloadLen)
{
   int len = snprintf((char*)payload, 250, "%s", APP_VERSION_STRING );
   *payloadLen = len;
    return TRUE;
}

/******************************************************************************
 * @name Fills1PacketPayloadt - API call ro prepare user output packet
 * @brief general handler
 * @param [in] payload pointer to put user data to
 * @param [in/out] number of bytes in user payload
 * @retval N/A
 ******************************************************************************/
static BOOL Fill_s1PacketPayload(uint8_t *payload, uint8_t *payloadLen)
{           
    scaled1_payload_t *pld = (scaled1_payload_t *)payload;  
    *payloadLen = sizeof(scaled1_payload_t);

    gtime_t time;
    int week;
    time.time = imu_time.time;
    time.sec = (double)imu_time.msec/1000;
    pld->timeOfWeek = time2gpst(time,&week); 
    
    if (pld->timeOfWeek < 0){
        pld->week = 0;
        pld->timeOfWeek = imu_time.time + (double)imu_time.msec/1000;
    } else {
        pld->week = week;
    }
    
    //pld->temp_C = GetUnitTemp();
    GetAccelData_g(pld->accel_g);
    GetRateData_radPerSec(pld->rate_dps);

    return TRUE;
}

/******************************************************************************
 * @name Fill_posPacketPayload - API call ro prepare user output packet
 * @brief general handler
 * @param [in] payload pointer to put user data to
 * @param [in/out] number of bytes in user payload
 * @retval N/A
 ******************************************************************************/
static BOOL Fill_posPacketPayload(uint8_t *payload, uint8_t *payloadLen)
{
    // pos_payload_t *pld = (pos_payload_t *)payload;

    *payloadLen = sizeof(pos_payload_t);
    return TRUE;
}

/******************************************************************************
 * @name Fill_skyviewPacketPayload - API call ro prepare user output packet
 * @brief general handler
 * @param [in] payload pointer to put user data to
 * @param [in/out] number of bytes in user payload
 * @retval N/A
 ******************************************************************************/
static BOOL Fill_skyviewPacketPayload(uint8_t *payload, uint8_t *payloadLen)
{
    // skyview_payload_t *pld = (skyview_payload_t *)payload;

    *payloadLen = sizeof(skyview_payload_t) * 10;
    return true;
}

/******************************************************************************
 * @name HandleUserInputPacket - API
 * @brief general handler
 * @param [out] packetPtr - filled in packet from the mapped physical port
 * @retval N/A
 ******************************************************************************/
int HandleUserInputPacket(UcbPacketStruct *ptrUcbPacket)
{
    BOOL valid = TRUE;
    int ret = USER_PACKET_OK;

    /// call appropriate function based on packet type
	switch (_inputPacketType) {
		case USR_IN_PING:
            {
                uint8_t len;
                Fill_PingPacketPayload(ptrUcbPacket->payload, &len);
                ptrUcbPacket->payloadLength = len;
            }
            // leave all the same - it will be bounced back unchanged
            break;
		case USR_IN_GET_VERSION:
            {
                uint8_t len;
                Fill_VersionPacketPayload(ptrUcbPacket->payload, &len);
                ptrUcbPacket->payloadLength = len;
            }
            break;
        case USR_IN_UPDATE_PARAM:
             UpdateUserParam((userParamPayload*)ptrUcbPacket->payload, &ptrUcbPacket->payloadLength);
             break;
        case USR_IN_SAVE_CONFIG:
            // payload length does not change
             if(!SaveUserConfig()){
                valid = FALSE;
             }
             break;
        case USR_IN_GET_ALL:
             if(!GetAllUserParams((allUserParamsPayload*)ptrUcbPacket->payload, &ptrUcbPacket->payloadLength)){
                valid = FALSE;
             }
             break;
        default:
             /// default handler - unknown packet
             valid = FALSE;
             break;
        }

        if(!valid){
             ptrUcbPacket->payloadLength = 0;
             ret = USER_PACKET_ERROR;
        }

        ptrUcbPacket->packetType = UCB_USER_OUT;    // do not remove - done for proper packet routing
        
        return ret;
}


/******************************************************************************
 * @name HandleUserOutputPacket - API call ro prepare continuous user output packet
 * @brief general handler
 * @param [in] payload pointer to put user data to
 * @param [in/out] number of bytes in user payload
 * @retval N/A
 ******************************************************************************/
BOOL HandleUserOutputPacket(uint8_t *payload, uint8_t *payloadLen)
{
    BOOL ret = TRUE;

    switch (_outputPacketType)
    {
    case USR_OUT_SCALED1:
    {
        uint8_t len;
        Fill_s1PacketPayload(payload, &len);
        *payloadLen = len;
    }
    break;

    case USR_OUT_POS:
    {
        uint8_t len;
        Fill_posPacketPayload(payload, &len);
        *payloadLen = len;
    }
    break;

    case USR_OUT_SKY: // skyview
    {
        uint8_t len;
        Fill_skyviewPacketPayload(payload, &len);
        *payloadLen = len;
    }
    break;

    default:
    {
        *payloadLen = 0;
        ret = FALSE;
    }
    break; /// unknown user packet, will send error in response
    }

    return ret;
}
