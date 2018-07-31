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
#include "UserMessaging.h"
#include "UserConfiguration.h"
#include "algorithmAPI.h"
#include "userAPI.h"
#include "platformAPI.h"
#include "sensorsAPI.h"

#include "Indices.h"   // For X_AXIS, etc

// provided as example
char userVersionString[] = "IMU 1.0.0";


/// List of allowed packet codes 
usr_packet_t userInputPackets[] = {		//       
    {USR_IN_NONE,               {0,0}},   //  "  "
    {USR_IN_PING,               "pG"}, 
    {USR_IN_UPDATE_CONFIG,      "uC"}, 
    {USR_IN_UPDATE_PARAM,       "uP"}, 
    {USR_IN_UPDATE_ALL,         "uA"}, 
    {USR_IN_SAVE_CONFIG,        "sC"}, 
    {USR_IN_RESTORE_DEFAULTS,   "rD"}, 
    {USR_IN_GET_CONFIG,         "gC"}, 
    {USR_IN_GET_PARAM,          "gP"}, 
    {USR_IN_GET_ALL,            "gA"}, 
    {USR_IN_GET_VERSION,        "gV"}, 
    {USR_IN_RESET,              "rS"}, 
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
    {USR_OUT_DATA2,             "z2"},
// place new type and code here
    {USR_OUT_SCALED1,           "s1"},
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
    }else {
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
        case USR_OUT_DATA2:            // packet with arbitrary data
            _outputPacketType = type;
            _userPayloadLen   = sizeof(data2_payload_t);
            break;
        case USR_OUT_SCALED1:          // packet with arbitrary data
            _outputPacketType = type;
            _userPayloadLen   = USR_OUT_SCALED1_PAYLOAD_LEN;
            break;
        default:
            result = FALSE;
            break; 
    }

    if(result == FALSE){
        return FALSE;
    }

    tmp = (data[0] << 8) | data[1];

    result = platformSetOutputPacketCode(tmp, fApply);

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
//    userPacket *pkt =  (userPacket *)ptrUcbPacket->payload;

    /// call appropriate function based on packet type


	switch (_inputPacketType) {
		case USR_IN_RESET:
            Reset();
            break;
		case USR_IN_PING:
            {
                int len; 
                uint8_t *model = (uint8_t*)unitVersionString();
                uint8_t *rev   = (uint8_t*)platformBuildInfo();
                unsigned int serialNum          = unitSerialNumber();
                len = snprintf((char*)ptrUcbPacket->payload, 250, "%s %s SN:%u", model, rev, serialNum );
                ptrUcbPacket->payloadLength = len;
            }
            // leave all the same - it will be bounced back unchanged
            break;
		case USR_IN_GET_VERSION:
            {
                int len = snprintf((char*)ptrUcbPacket->payload, 250, "%s", userVersionString );
                ptrUcbPacket->payloadLength = len;
            }
            break;
		case USR_IN_SAVE_CONFIG:
            // payload length does not change
             if(!SaveUserConfig()){
                valid = FALSE;
             }
             break;
		case USR_IN_UPDATE_CONFIG:
             UpdateUserConfig((userConfigPayload*)ptrUcbPacket->payload, &ptrUcbPacket->payloadLength);
             break;
		case USR_IN_UPDATE_PARAM:
             UpdateUserParam((userParamPayload*)ptrUcbPacket->payload, &ptrUcbPacket->payloadLength);
             break;
		case USR_IN_UPDATE_ALL:
             UpdateAllUserParams((allUserParamsPayload*)ptrUcbPacket->payload, &ptrUcbPacket->payloadLength);
             break;
        case USR_IN_RESTORE_DEFAULTS:
             valid = RestoreDefaultUserConfig();
             break;
        case USR_IN_GET_CONFIG:
             if(!GetUserConfig((userConfigPayload*)ptrUcbPacket->payload, &ptrUcbPacket->payloadLength)){
                valid = FALSE;
             }
             break;
        case USR_IN_GET_PARAM:
             if(!GetUserParam((userParamPayload*)ptrUcbPacket->payload, &ptrUcbPacket->payloadLength)){
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


// Declare the leveler data structure
IMUDataStruct gIMU;

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
            {  uint32_t *testParam = (uint32_t*)(payload);
             *payloadLen = USR_OUT_TEST_PAYLOAD_LEN;
             *testParam  = _testVal++;
            }
            break;
        case USR_OUT_DATA1:
            {   int n = 0;
                double accels[3];
                double mags[3];
                double rates[3];
                data1_payload_t *pld = (data1_payload_t *)payload;  

                pld->timer  = getDacqTime();
                GetAccelData_mPerSecSq(accels);
                for (int i = 0; i < 3; i++, n++){
                    pld->sensorsData[n] = (float)accels[i];
                }
                GetRateData_degPerSec(rates);
                for (int i = 0; i < 3; i++, n++){
                    pld->sensorsData[n] = (float)rates[i];
                }
                GetMagData_G(mags);
                for (int i = 0; i < 3; i++, n++){
                    pld->sensorsData[n] = (float)mags[i];
                }
                *payloadLen = sizeof(data1_payload_t);
            }
            break;
        case USR_OUT_DATA2:
            {   
                data2_payload_t *pld = (data2_payload_t *)payload;  
                pld->timer  = getDacqTime();
                pld->c = 'A';
                pld->s = 1234;
                pld->i = -5;
                pld->ll = 1122334455667788LL;
                pld->d  = 1.23456789;
                *payloadLen = sizeof(data2_payload_t);
            }
            break;
        // place additional user packet preparing calls here
        // case USR_OUT_XXXX:
        //      *payloadLen = YYYY; // total user payload length, including user packet type
        //      payload[0]  = ZZZZ; // user packet type 
        //      prepare dada here
        //      break;
        case USR_OUT_SCALED1:
            {
                // The payload length (NumOfBytes) is based on the following:
                // 1 uint32_t (4 bytes) =  4 bytes
                // 1 double (8 bytes)   =  8 bytes
                // 3 floats (4 bytes)   = 12 bytes
                // 3 floats (4 bytes)   = 12 bytes
                // 3 floats (4 bytes)   = 12 bytes
                // 1 floats (4 bytes)   =  4 bytes
                // =================================
                //           NumOfBytes = 52 bytes
                *payloadLen = USR_OUT_SCALED1_PAYLOAD_LEN;

                // Output time as represented by gIMU.timerCntr (uint32_t
                // incremented at each call of the algorithm)
                uint32_t *algoData_1 = (uint32_t*)(payload);
                *algoData_1++ = gIMU.timerCntr;

                // Output a double representation of time generated from
                // gLeveler.itow
                double *algoData_2 = (double*)(algoData_1);
                *algoData_2++ = 1.0e-3 * (double)(gIMU.timerCntr);

                // Set the pointer of the sensor array to the payload
                float *algoData_3 = (float*)(algoData_2);
                *algoData_3++ = (float)gIMU.accel_g[X_AXIS];
                *algoData_3++ = (float)gIMU.accel_g[Y_AXIS];
                *algoData_3++ = (float)gIMU.accel_g[Z_AXIS];

                *algoData_3++ = (float)gIMU.rate_degPerSec[X_AXIS];
                *algoData_3++ = (float)gIMU.rate_degPerSec[Y_AXIS];
                *algoData_3++ = (float)gIMU.rate_degPerSec[Z_AXIS];

                *algoData_3++ = (float)gIMU.mag_G[X_AXIS];
                *algoData_3++ = (float)gIMU.mag_G[Y_AXIS];
                *algoData_3++ = (float)gIMU.mag_G[Z_AXIS];

                *algoData_3++ = (float)gIMU.temp_C;
            }
            break;
        default:
             *payloadLen = 0;  
             ret         = FALSE;
             break;      /// unknown user packet, will send error in response
        }

        return ret;
}


void WriteResultsIntoOutputStream(void *results)
{
//  implement specific data processing/saving here 

}
