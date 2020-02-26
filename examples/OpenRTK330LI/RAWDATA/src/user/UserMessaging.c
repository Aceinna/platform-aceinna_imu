/** ***************************************************************************
 * @file   UserConfiguration.c
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
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

#include <string.h>
#include <stdint.h>
#include <stdio.h>

#include "algorithmAPI.h"
#include "gpsAPI.h"
#include "platformAPI.h"
#include "configurationAPI.h"
#include "sensorsAPI.h"
#include "userAPI.h"
#include "ucb_packet.h"
#include "CommonMessages.h"
#include "UserMessaging.h"
#include "UserConfiguration.h"
#include "UserAlgorithm.h"

#include "algorithm.h"
#include "timer.h"

// for EKFOutputDataStruct
// #include "EKF_Algorithm.h"
EKF_OutputDataStruct *algo_res;

// Example inputs:
//   Ping: 55 55 70 47 00 5D 5F
//   Mag Align: 55 55 6D 61 00 F0 2D
//              55 55 6D 61 01 00 F1 2E
//              55 55 6D 61 01 01 E1 0F
//              55 55 6D 61 01 02 D1 6C
//              55 55 6D 61 01 03 C1 4D
//              55 55 6D 61 01 04 B1 AA
//              55 55 6D 61 01 05 A1 8B
//              55 55 6D 61 01 06 91 E8
//              55 55 6D 61 01 07 81 C9   // Get stored values
//              55 55 6D 61 01 08 70 26
//              55 55 6D 61 01 09 60 07
//              55 55 6D 61 01 0A 50 64
//              55 55 6D 61 01 0B 
//              55 55 6D 61 01 0C 
//              55 55 6D 61 01 0D 
//              55 55 6D 61 01 0E 10 E0
//              55 55 6D 61 01 0F 

//  other: 55 55 ...

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
    {USR_OUT_TEST,              "zT"},   
    {USR_OUT_DATA1,             "z1"},   
    {USR_OUT_ANG1,              "a1"},   
    {USR_OUT_ANG2,              "a2"},   
// place new type and code here
    {USR_OUT_SCALED1,           "s1"},
    {USR_OUT_EKF1,              "e1"},
    {USR_OUT_EKF2,              "e2"},
    {USR_OUT_RTK1,              "K1"},
    {USR_OUT_POS,               "pS"},
    {USR_OUT_SKY,               "sK"},
    {USR_OUT_C1,                "C1"}, //4331
    {USR_OUT_MAX,               {0xff, 0xff}},   //  "" 
};

volatile char   *info;
static   int    _userPayloadLen = 0;
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


/** ***************************************************************************
 * @name setUserPacketType - set user output packet type 
 * @brief
 * @param [in] packet type
 * @retval  - TRUE if success, FALSE otherwise
 ******************************************************************************/
BOOL setUserPacketType(uint8_t *data, BOOL fApply)
{
    int type = -1;
    uint16_t *code = (uint16_t*)data;
    uint16_t tmp;
    BOOL result = TRUE;

    usr_packet_t *packet = &userOutputPackets[1];
    for(int i = 0; i < USR_OUT_MAX; i++, packet++){
        if(*code == *((uint16_t*)packet->packetCode)){
            type = packet->packetType;
            break;
        }
    }

    switch(type){
        case USR_OUT_TEST:              // simple test packet to check communication
            _outputPacketType = type;
            _userPayloadLen   = USR_OUT_TEST_PAYLOAD_LEN;
            break;
        case USR_OUT_DATA1:            // packet with sensors data. Change at will
            _outputPacketType = type;
            _userPayloadLen   = USR_OUT_DATA1_PAYLOAD_LEN;
            break;
        case USR_OUT_ANG1:            // packet with sensors data. Change at will
            _outputPacketType = type;
            _userPayloadLen   = USR_OUT_ANG1_PAYLOAD_LEN;
            break;
        case USR_OUT_ANG2:            // packet with sensors data. Change at will
            _outputPacketType = type;
            _userPayloadLen   = USR_OUT_ANG2_PAYLOAD_LEN;
            break;
        case USR_OUT_SCALED1:          // packet with arbitrary data
            _outputPacketType = type;
            _userPayloadLen   = USR_OUT_SCALED1_PAYLOAD_LEN;
            break;
        case USR_OUT_EKF1: // packet with EKF algorithm data
            _outputPacketType = type;
            _userPayloadLen   = USR_OUT_EKF1_PAYLOAD_LEN;
            break;
        case USR_OUT_EKF2: // packet with EKF algorithm data
            _outputPacketType = type;
            _userPayloadLen   = USR_OUT_EKF2_PAYLOAD_LEN;
            break;
        case USR_OUT_RTK1:
            _outputPacketType = type;
            _userPayloadLen   = USR_OUT_RTK1_PAYLOAD_LEN; // DW DEBUG
        case USR_OUT_C1:
            _outputPacketType = type;
            _userPayloadLen   = USR_OUT_RTK1_PAYLOAD_LEN; // DW DEBUG
            break;
        default:
            result = FALSE;
            break;
    }

    if(result == FALSE){
        return FALSE;
    }

    tmp = (data[0] << 8) | data[1];

    result = configSetOutputPacketCode(tmp, fApply);

    return result;
}


/** ***************************************************************************
 * @name getUserPayloadLength - get user payload length for sanity check 
 * @brief
 *
 * @retval  - user payload length
 ******************************************************************************/
int getUserPayloadLength(void)
{
    // ATTENTION: return actual user payload length, if user packet used    
    return _userPayloadLen;
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
    static uint32_t _testVal = 0;
    BOOL ret = TRUE;

	switch (_outputPacketType) {
        case USR_OUT_TEST:
            {
                uint32_t *testParam = (uint32_t*)(payload);
                *payloadLen = USR_OUT_TEST_PAYLOAD_LEN;
                *testParam  = _testVal++;
            }
            break;

        case USR_OUT_DATA1:
            {
                uint8_t len;
                Fill_z1PacketPayload(payload, &len);
                *payloadLen = len;
            }
			break;

        case USR_OUT_ANG1:
            {
                // Variables used to hold the EKF values
                uint8_t len;
                Fill_a1PacketPayload(payload, &len);
                *payloadLen = len;
            }
            break;

        case USR_OUT_ANG2:
            {
                uint8_t len;
                Fill_a2PacketPayload(payload, &len);
                *payloadLen = len;
            }
            break;

        case USR_OUT_SCALED1:
            {
                uint8_t len;
                Fill_s1PacketPayload(payload, &len);
                *payloadLen = len;
            }
            break;

        case USR_OUT_EKF1:
            {
                // Variables used to hold the EKF values
                uint8_t len;
                Fill_e1PacketPayload(payload, &len);
                *payloadLen = len;
            }
            break;

        case USR_OUT_EKF2:
            {
                // Variables used to hold the EKF values
                uint8_t len;
                Fill_e2PacketPayload(payload, &len);
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

#ifdef TEST_RTK_TASK
        case USR_OUT_RTK1:
            {
                uint8_t len;
                Fill_skyviewPacketPayload(payload, &len);
                *payloadLen = len;
            }
            break;
#endif
        case USR_OUT_C1:
            {
                int len = 0;
                char sensor_buf[512];
                double accel_data[3];
                double gyro_data[3];
                volatile mcu_time_base_t *get_time_s;
#if 1
                get_time_s = get_mcu_time();

                GetAccelData_g_AsDouble(accel_data);
                // GetAccelData_mPerSecSq(accel_data);
                GetMagData_G_AsDouble(gyro_data);
                //GetRateData_radPerSec(gyro_data);
#if 1
                //len = sprintf(sensor_buf, "%s", "time=");
                len += sprintf(sensor_buf + len, "%lld%c", (get_time_s->time), '_');
                len += sprintf(sensor_buf + len, "%ld  ", (long)(get_time_s->msec));
#endif
                len += sprintf(sensor_buf + len, "%lf  ", accel_data[0]);
                len += sprintf(sensor_buf + len, "%lf  ", accel_data[1]);
                len += sprintf(sensor_buf + len, "%lf  ", accel_data[2]);
                len += sprintf(sensor_buf + len, "%lf  ", gyro_data[0]);
                len += sprintf(sensor_buf + len, "%lf  ", gyro_data[1]);
                len += sprintf(sensor_buf + len, "%lf\n", gyro_data[2]);
                memcpy(payload, sensor_buf, len);
                *payloadLen = len;
#endif
            }
            break;

        default:
            {
                *payloadLen = 0;
                ret         = FALSE;
            }
            break;    /// unknown user packet, will send error in response
        }

        return ret;
}


void WriteResultsIntoOutputStream(void *results)
{
//  implement specific data processing/saving here 
    algo_res = results;
}
