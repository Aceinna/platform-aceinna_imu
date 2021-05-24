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

#include "gpsAPI.h"
#include "magAPI.h"
#include "platformAPI.h"
#include "sensorsAPI.h"
#include "userAPI.h"

#include "UserMessagingUART.h"
#include "UserAlgorithm.h"
#include "UserConfiguration.h"
#include "CommonMessages.h"

#include "MagAlign.h"

#include "algorithm.h"

// Version string
char userVersionString[] = "INS 1.1.0";

// for EKFOutputDataStruct
#include "EKF_Algorithm.h"
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
    {USR_IN_MAG_ALIGN,          "ma"},   // 0x6D 0x61
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
    {USR_OUT_ID,                "id"},
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
        case USR_OUT_ID:
            _outputPacketType = type;
            _userPayloadLen = USR_OUT_ID_PAYLOAD_LEN;
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
    
    uint8_t retVal;
    int8_t estimatedMagAlignVals[8] = {0};
    int8_t magAlignVals[8]          = {0};

//    userPacket *pkt =  (userPacket *)ptrUcbPacket->payload;

    /// call appropriate function based on packet type
	switch (_inputPacketType) {
		case USR_IN_RESET:
            Reset();
            break;
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
        case USR_IN_MAG_ALIGN:
            // Set the valid flag true, if the command is not valid then
            //   it will be set false upon entry into case 0.
            valid = TRUE;

            //
            retVal = ProcessMagAlignCmds((magAlignCmdPayload*)ptrUcbPacket->payload, &ptrUcbPacket->payloadLength);
            switch(retVal-1)
            {
                //
                uint8_t len;

                case 0:   // Return the Mag-Align status
                    //uint8_t *model = (uint8_t*)unitVersionString();
                    //uint8_t *rev   = (uint8_t*)platformBuildInfo();
                    //unsigned int serialNum = unitSerialNumber();
                    //len = snprintf((char*)ptrUcbPacket->payload, 250, "%s %s SN:%u", model, rev, serialNum );
                    //ptrUcbPacket->payloadLength = len;
                    if(gMagAlign.state == MAG_ALIGN_STATUS_START_CAL_WITH_AUTOEND) {
                        // Start (auto end)
                        len = snprintf((char*)ptrUcbPacket->payload, 250, "%c", (char)0x1 );
                    } else if(gMagAlign.state == MAG_ALIGN_STATUS_START_CAL_WITHOUT_AUTOEND) {
                        // Start (manual end)
                        len = snprintf((char*)ptrUcbPacket->payload, 250, "%c", (char)0x2 );
                    } else {
                        // Start (manual end)
                        len = snprintf((char*)ptrUcbPacket->payload, 250, "%c", (char)0x0 );
                    }

                    ptrUcbPacket->payloadLength = len;
                    break;

                case 1:   // Start mag-align w/ autoend
                case 2:   // Start mag-align w/ autoend
                case 3:   // Stop mag-align w/ autoend
                case 4:   // Accept results
                case 5:   // Accept results and write to EEPROM
                case 6:   // Abort Mag-Align or reject results
                case 8:   // Restore default mag-align values
                case 9:   // Restore default mag-align values and save in EEPROM
                    len = snprintf((char*)ptrUcbPacket->payload, 250, "%c", (char)retVal-1 );
                    ptrUcbPacket->payloadLength = len;
                    break;

                case 7:   // Return stored mag-align values
#if 0
                    // Test values:
                    gMagAlign.estParams.hardIronBias[X_AXIS] =  0.1;
                    gMagAlign.estParams.hardIronBias[Y_AXIS] = -0.2;
                    gMagAlign.estParams.softIronScaleRatio   = 0.98;
                    gMagAlign.estParams.softIronAngle        = -270.0 * DEG_TO_RAD;
#endif
                
                    // Bias can be +/- 8.0 [g] (full scale of sensor)
                    //   SF = 2^15 / maxVal = 2^15 / 8.0 = 4096
                    magAlignVals[0] = (char)( ( (int16_t)( gMagAlign.hardIronBias[X_AXIS] * (float)4096.0 ) >> 8 ) & 0xFF );
                    magAlignVals[1] = (char)( ( (int16_t)( gMagAlign.hardIronBias[X_AXIS] * (float)4096.0 ) >> 0 ) & 0xFF );
                    magAlignVals[2] = (char)( ( (int16_t)( gMagAlign.hardIronBias[Y_AXIS] * (float)4096.0 ) >> 8 ) & 0xFF );
                    magAlignVals[3] = (char)( ( (int16_t)( gMagAlign.hardIronBias[Y_AXIS] * (float)4096.0 ) >> 0 ) & 0xFF );

                    // Ratio can be 0 --> 1
                    //   SF = (2^16-1) / maxVal = (2^16-1) / 1.0 = 65535
                    magAlignVals[4] = (char)( ( (int16_t)( gMagAlign.softIronScaleRatio * (float)65535.0 ) >> 8 ) & 0xFF );
                    magAlignVals[5] = (char)( ( (int16_t)( gMagAlign.softIronScaleRatio * (float)65535.0 ) >> 0 ) & 0xFF );

                    //   SF = 2^15 / maxVal = 2^15 / pi = 10430.37835047045
                    magAlignVals[6] = (char)( ( (int16_t)( gMagAlign.softIronAngle * (float)10430.37835047046 ) >> 8 ) & 0xFF );
                    magAlignVals[7] = (char)( ( (int16_t)( gMagAlign.softIronAngle * (float)10430.37835047046 ) >> 0 ) & 0xFF );

                    // Bias can be +/- 8.0 [g] (full scale of sensor)
                    //   SF = 2^15 / maxVal = 2^15 / 8.0 = 4096
                    estimatedMagAlignVals[0] = (char)( ( (int16_t)( gMagAlign.estParams.hardIronBias[X_AXIS] * (float)4096.0 ) >> 8 ) & 0xFF );
                    estimatedMagAlignVals[1] = (char)( ( (int16_t)( gMagAlign.estParams.hardIronBias[X_AXIS] * (float)4096.0 ) >> 0 ) & 0xFF );
                    estimatedMagAlignVals[2] = (char)( ( (int16_t)( gMagAlign.estParams.hardIronBias[Y_AXIS] * (float)4096.0 ) >> 8 ) & 0xFF );
                    estimatedMagAlignVals[3] = (char)( ( (int16_t)( gMagAlign.estParams.hardIronBias[Y_AXIS] * (float)4096.0 ) >> 0 ) & 0xFF );

                    // Ratio can be 0 --> 1
                    //   SF = (2^16-1) / maxVal = (2^16-1) / 1.0 = 65535
                    estimatedMagAlignVals[4] = (char)( ( (int16_t)( gMagAlign.estParams.softIronScaleRatio * (float)65535.0 ) >> 8 ) & 0xFF );
                    estimatedMagAlignVals[5] = (char)( ( (int16_t)( gMagAlign.estParams.softIronScaleRatio * (float)65535.0 ) >> 0 ) & 0xFF );

                    // Angle can be +/- pi (in radians)
                    //   Correct for angles that exceed +/-180
                    if(gMagAlign.estParams.softIronAngle > PI) {
                        gMagAlign.estParams.softIronAngle = (float)PI - gMagAlign.estParams.softIronAngle;
                    } else if(gMagAlign.estParams.softIronAngle < -PI) {
                        gMagAlign.estParams.softIronAngle = TWO_PI + gMagAlign.estParams.softIronAngle;
                    }

                    //   SF = 2^15 / maxVal = 2^15 / pi = 10430.37835047045
                    estimatedMagAlignVals[6] = (char)( ( (int16_t)( gMagAlign.estParams.softIronAngle * (float)10430.37835047046 ) >> 8 ) & 0xFF );
                    estimatedMagAlignVals[7] = (char)( ( (int16_t)( gMagAlign.estParams.softIronAngle * (float)10430.37835047046 ) >> 0 ) & 0xFF );

#if 0
                    DebugPrintFloat("     ", (float)estimatedMagAlignVals[0], 1);
                    DebugPrintFloat(",  ", (float)estimatedMagAlignVals[1], 1);
                    DebugPrintFloat(",  ", (float)estimatedMagAlignVals[2], 1);
                    DebugPrintFloat(",  ", (float)estimatedMagAlignVals[3], 1);
                    DebugPrintFloat(",  ", (float)estimatedMagAlignVals[4], 1);
                    DebugPrintFloat(",  ", (float)estimatedMagAlignVals[5], 1);
                    DebugPrintFloat(",  ", (float)estimatedMagAlignVals[6], 1);
                    DebugPrintFloat(",  ", (float)estimatedMagAlignVals[7], 1);
                    DebugPrintEndline();
#endif

                    len = snprintf((char*)ptrUcbPacket->payload, 250, "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c", (char)magAlignVals[0], 
                                                                                                          (char)magAlignVals[1], 
                                                                                                          (char)magAlignVals[2], 
                                                                                                          (char)magAlignVals[3], 
                                                                                                          (char)magAlignVals[4], 
                                                                                                          (char)magAlignVals[5], 
                                                                                                          (char)magAlignVals[6], 
                                                                                                          (char)magAlignVals[7],
                                                                                                          (char)estimatedMagAlignVals[0], 
                                                                                                          (char)estimatedMagAlignVals[1], 
                                                                                                          (char)estimatedMagAlignVals[2], 
                                                                                                          (char)estimatedMagAlignVals[3], 
                                                                                                          (char)estimatedMagAlignVals[4], 
                                                                                                          (char)estimatedMagAlignVals[5], 
                                                                                                          (char)estimatedMagAlignVals[6], 
                                                                                                          (char)estimatedMagAlignVals[7] );
                    ptrUcbPacket->payloadLength = len;
                    break;

                case 10:   // Load user computed mag-align values
                    break;

                default:
                    valid = FALSE;
                    break;
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
        //      break;
        case USR_OUT_ID:
            {
                // The payload length (NumOfBytes) is based on the following:
                // 1 uint32_t (4 bytes) =   4 bytes   timer
                // 1 float  (4 bytes)   =   4 bytes   GPS heading
                // 1 uint32_t (4 bytes) =   4 bytes   GPS itow
                // 3 floats (4 bytes)   =  12 bytes   ea
                // 3 floats (4 bytes)   =  12 bytes   a
                // 3 floats (4 bytes)   =  12 bytes   aBias
                // 3 floats (4 bytes)   =  12 bytes   w
                // 3 floats (4 bytes)   =  12 bytes   wBias
                // 3 floats (4 bytes)   =  12 bytes   v
                // 3 floats (4 bytes)   =  12 bytes   gps NED velocity
                // 3 double (8 bytes)   =  24 bytes   lla
                // 3 double (8 bytes)   =  24 bytes   gps LLA
                // 1 uint8_t (1 byte)   =   1 bytes
                // 1 uint8_t (1 byte)   =   1 bytes
                // 1 uint8_t (1 byte)   =   1 bytes
                // =================================
                //           NumOfBytes = 147 bytes
                *payloadLen = USR_OUT_ID_PAYLOAD_LEN;

                // Output time as represented by gLeveler.timerCntr (uint32_t
                // incremented at each call of the algorithm)
                uint32_t *algoData_1 = (uint32_t*)(payload);
                *algoData_1++ = gIMU.timerCntr;

                // Set the pointer of the algoData array to the payload
                float *algoData_2_1 = (float*)(algoData_1);
                *algoData_2_1++ = (float)gEKFInput.trueCourse;
                uint32_t *algoData_2 = (uint32_t*)(algoData_2_1);
                // *algoData_2++ = (double)( 0.001 * gIMU.timerCntr );
                *algoData_2++ = gEKFInput.itow;

                // Set the pointer of the algoData array to the payload
                float *algoData_3 = (float*)(algoData_2);
                real EulerAngles[NUM_AXIS];
                EKF_GetAttitude_EA(EulerAngles);
                *algoData_3++ = (float)EulerAngles[ROLL];
                *algoData_3++ = (float)EulerAngles[PITCH];
                *algoData_3++ = (float)EulerAngles[YAW];

                // double accels[NUM_AXIS];
                *algoData_3++ = (float)gIMU.accel_g[X_AXIS];
                *algoData_3++ = (float)gIMU.accel_g[Y_AXIS];
                *algoData_3++ = (float)gIMU.accel_g[Z_AXIS];

                // float accelBias[NUM_AXIS];
                // EKF_GetEstimatedAccelBias(accelBias);
                // *algoData_3++ = (float)accelBias[X_AXIS];
                // *algoData_3++ = (float)accelBias[Y_AXIS];
                // *algoData_3++ = (float)accelBias[Z_AXIS];
                *algoData_3++ = (float)gEKFInput.HDOP;
                *algoData_3++ = (float)gEKFInput.GPSHorizAcc;
                *algoData_3++ = (float)gEKFInput.GPSVertAcc;

                // double rates[NUM_AXIS];
                *algoData_3++ = (float)gIMU.rate_degPerSec[X_AXIS];
                *algoData_3++ = (float)gIMU.rate_degPerSec[Y_AXIS];
                *algoData_3++ = (float)gIMU.rate_degPerSec[Z_AXIS];

                float rateBias[NUM_AXIS];
                EKF_GetEstimatedAngRateBias(rateBias);
                *algoData_3++ = (float)rateBias[X_AXIS];
                *algoData_3++ = (float)rateBias[Y_AXIS];
                *algoData_3++ = (float)rateBias[Z_AXIS];

                float vel[NUM_AXIS];
                EKF_GetEstimatedVelocity(vel);
                *algoData_3++ = (float)vel[X_AXIS];
                *algoData_3++ = (float)vel[Y_AXIS];
                *algoData_3++ = (float)vel[Z_AXIS];

                // double mags[NUM_AXIS];
                // GetMagData_G(mags);
                *algoData_3++ = (float)gEKFInput.vNedAnt[0];
                *algoData_3++ = (float)gEKFInput.vNedAnt[1];
                *algoData_3++ = (float)gEKFInput.vNedAnt[2];

                // Set the pointer of the algoData array to the payload
                double *algoData_4 = (double*)(algoData_3);
                double lla[NUM_AXIS];
                EKF_GetEstimatedLLA(lla);
                *algoData_4++ = (double)lla[LAT];
                *algoData_4++ = (double)lla[LON];
                *algoData_4++ = (double)lla[ALT];

                // debug
                *algoData_4++ = (double)gEKFInput.llaAnt[LAT] * R2D;
                *algoData_4++ = (double)gEKFInput.llaAnt[LON] * R2D;
                *algoData_4++ = (double)gEKFInput.llaAnt[ALT];

                // Set the pointer of the algoData array to the payload
                uint8_t *algoData_5 = (uint8_t*)(algoData_4);
                uint8_t opMode, linAccelSw, turnSw;
                EKF_GetOperationalMode(&opMode);
                EKF_GetOperationalSwitches(&linAccelSw, &turnSw);
                turnSw = turnSw << 1;
                turnSw |= gEKFInput.ppsDetected;
                turnSw = turnSw << 1;
                turnSw |= gEKFInput.gpsFixType;
                turnSw = turnSw << 1;
                turnSw |= gEKFInput.gpsUpdate;
                *algoData_5++ = opMode;
                *algoData_5++ = gEKFInput.numSatellites;
                *algoData_5++ = turnSw;
            }
            break;

        // place additional user packet preparing calls here
        // case USR_OUT_XXXX:
        //      *payloadLen = YYYY; // total user payload length, including user packet type
        //      payload[0]  = ZZZZ; // user packet type 
        //      prepare dada here
        //      break;
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
