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

#include "string.h"
#include "magAPI.h"
#include "platformAPI.h"
#include "UserConfiguration.h"
#include "UserMessagingUART.h"
#include "Indices.h"
#include "eepromAPI.h"


uint8_t UserDataBuffer[4096];
static  volatile char   *info;



/** ***************************************************************************
 * @name UpdateSystemParameter - updating of system configuration parameter based of user preferences 
 * @brief
 *
 * @param [in]  number - parameter number in user configuration structure
 * @param [in]  data   - value of the parameter in little endian format
 * @retval error (1), no error (0)
 ******************************************************************************/
// NEEDS TO BE CHECKED
BOOL  UpdateSystemParameter(uint32_t number, uint64_t data, BOOL fApply)
{
     BOOL result = TRUE;
     uint64_t *ptr = (uint64_t *)pUserUartConfig;
     uint16_t orientOut;
     if(number < USER_UART_CRC || number >= USER_UART_MAX_PARAM ){
         return FALSE;
     }

     switch (number) {
            case USER_UART_BAUD_RATE:
                result = platformSetBaudRate((int)data, fApply);
                if(fApply && result == TRUE){
                    UpdateEcuUartBaudrate(data);
                }
                break;
            case USER_UART_PACKET_TYPE:
                result = setUserPacketType((uint8_t*)&data, fApply);
                if(fApply && result == TRUE){
                    UpdateEcuUartPacketType(data);
                }
                break;
            case USER_UART_PACKET_RATE:
                result = platformSetPacketRate((int)data, fApply);
                if(fApply && result == TRUE){
                    UpdateEcuUartPacketRate(data);
                }
                break;
            case USER_UART_LPF_ACCEL_TYPE:
                result = platformSelectLPFilter(ACCEL_SENSOR, (uint32_t)data, fApply);
                if(fApply && result == TRUE){
                    UpdateEcuAccelFilterSettings((uint16_t)data);
                }
                break;
            case USER_UART_LPF_RATE_TYPE:
                result = platformSelectLPFilter(RATE_SENSOR, (uint32_t)data, fApply);
                if(fApply && result == TRUE){
                    UpdateEcuRateFilterSettings((uint16_t)data);
                }
                break;
            case  USER_UART_ORIENTATION:
                {
                    uint64_t tmp = data;
                    result = platformSetOrientation((uint16_t*)&data, &orientOut, fApply);
                    if(fApply && result == TRUE){
                        UpdateEcuOrientationSettings(orientOut);
                        data = tmp;
                    }
                }
                break;
            case  USER_UART_CRC:
            case  USER_UART_CONFIG_SIZE:
                return TRUE;
        
        // case USER_XXX:  add function calls here if parameter XXXX
        //                        required be updated on the fly
        //             break;
        default:
            // by default result should be FALSE for system parameter
            result = FALSE;
            break;
    }
    
    if(result == TRUE){
        ptr[number] = data;
    }

    return result;
}


void UserInitConfigureUart()
{
    platformSetBaudRate(pUserUartConfig->uartBaudRate, TRUE);
    setUserPacketType(pUserUartConfig->uartPacketType, TRUE);
    platformSetPacketRate(pUserUartConfig->uartPacketRate, TRUE);
    info = getBuildInfo();
} 



/** ****************************************************************************
 * @name UpdateUserConfig
 * @brief writes user data into user configuration structure, validates data if
 *        required, updates system parameters  
 *
 * @param [in] pointer to userData payload in the packet
 * @retval N/A
 ******************************************************************************/
BOOL UpdateUserConfig(userConfigPayload*  pld, uint8_t *payloadLen)
{
    uint32_t offset, i;
    BOOL offsetValid = TRUE;
    BOOL lenValid = TRUE;
    BOOL numValid = TRUE;
    BOOL ret = TRUE;
    int32_t result = 0;

    // Validate parameters numbers and quantity 
    if(pld->numParams  >= USER_UART_MAX_PARAM){
        lenValid = FALSE;
        result   = INVALID_PAYLOAD_SIZE;
    }


    if((pld->numParams + pld->paramOffset) >= USER_UART_MAX_PARAM){
        numValid = FALSE;        
        result   = INVALID_PARAM;
    }
    
    if(offsetValid && numValid && lenValid){
        // Validate parameters values first
        offset = pld->paramOffset;
        for (i = 0; i < pld->numParams && ret; i++){
            ret = UpdateSystemParameter(offset, pld->parameters[i], FALSE);
            offset++;
        }
        if(ret == TRUE){
            // Apply parameters values here
            offset = pld->paramOffset;
            for (i = 0; i < pld->numParams; i++){
                ret = UpdateSystemParameter(offset, pld->parameters[i], TRUE);
                offset++;
            }
        }
    }

    if(ret == FALSE){
        result = INVALID_VALUE;
    }

    pld->numParams  = result;
    *payloadLen     = 4;     

    return TRUE;
}


/** ****************************************************************************
 * @name UpdateUserParam
 * @brief writes user data into user configuration structure, validates data if
 *        required, updates system parameters  
 *
 * @param [in] pointer to userData payload in the packet
 * @retval N/A
 ******************************************************************************/
BOOL UpdateUserParam(userParamPayload*  pld, uint8_t *payloadLen)
{
    BOOL offsetValid;
    BOOL ret = TRUE;
    int32_t result = 0;

    offsetValid = pld->paramNum <=  USER_UART_MAX_PARAM;        
    
    if(offsetValid){
        // Validate parameter first
        ret = UpdateSystemParameter(pld->paramNum, pld->parameter, FALSE);
        if(ret == TRUE){
            // Apply parameter if valid
            ret = UpdateSystemParameter(pld->paramNum, pld->parameter, TRUE);
        }else{
            result = INVALID_VALUE;
        }    
    } else  {
        result = INVALID_PARAM;
    }
    
    pld->paramNum = result;
    *payloadLen   = 4;                  

    return TRUE;
}


/** ****************************************************************************
 * @name UpdateAllUserParams
 * @brief writes user data into user configuration structure, validates data if
 *        required, updates system parameters  
 *
 * @param [in] pointer to userData payload in the packet
 * @retval N/A
 ******************************************************************************/
/** ****************************************************************************
 * @name UpdateUserConfig
 * @brief writes user data into user configuration structure, validates data if
 *        required, updates system parameters  
 *
 * @param [in] pointer to userData payload in the packet
 * @retval N/A
 ******************************************************************************/
BOOL UpdateAllUserParams(allUserParamsPayload*  pld, uint8_t *payloadLen)
{
    uint32_t offset, i;
    BOOL numValid = TRUE;
    BOOL ret      = TRUE;
    int32_t    result = 0; 

    int    numParams = (*payloadLen)/8;

    if(numParams  > USER_UART_MAX_PARAM){
        numValid = FALSE;
        result   = INVALID_PAYLOAD_SIZE;
    }

    
    if(numValid){
        // Validate parameters here
        offset = 0;
        for (i = 0; i < numParams && ret; i++){
            ret = UpdateSystemParameter(offset, pld->parameters[i], FALSE);
            offset++;
        }
        if(ret == TRUE){
            // Apply parameters here
            offset = 0;
            for (i = 0; i < numParams; i++){
                UpdateSystemParameter(offset, pld->parameters[i], TRUE);
                offset++;
            }
        }
    }

    if(ret == FALSE){
        result = INVALID_VALUE;
    }

    pld->parameters[0] = result;
    *payloadLen        = 4;            // return error code

    return TRUE;

}


/** ****************************************************************************
 * @name  GetUserConfig
 * @brief Retrieves specified number of user configuration parameters data for 
 *        sending to the external host starting from specified offset in user 
 *        configuration structure (refer to UserConfigParamOffset structure for
 *        specific value of offsets)
 * @param [in] pointer to userData payload in the packet
 * @retval N/A
 ******************************************************************************/
BOOL GetUserConfig(userConfigPayload*  pld, uint8_t *payloadLen)
{
    uint32_t offset, i;
    BOOL lenValid = TRUE;
    uint64_t *ptr = (uint64_t *)pUserUartConfig;

    lenValid    = (pld->numParams + pld->paramOffset) <= USER_UART_MAX_PARAM;
    
    if(lenValid){
        offset = pld->paramOffset;
        for (i = 0; i < pld->numParams; i++, offset++)
        {
            pld->parameters[i] = ptr[offset];
        }
        *payloadLen     = (pld->numParams + 1) * 8;  
    } else  {
        *payloadLen    = 4;
        pld->numParams = INVALID_PARAM;
    }

    return TRUE;    

}


/** ****************************************************************************
 * @name  GetUserParam
 * @brief Retrieves specified number of user configuration parameters data for 
 *        sending to the external host starting from specified offset in user 
 *        configuration structure (refer to UserConfigParamOffset structure for
 *        specific value of offsets)
 * @param [in] pointer to userData payload in the packet
 * @retval N/A
 ******************************************************************************/
BOOL GetUserParam(userParamPayload*  pld, uint8_t *payloadLen)
{
    uint32_t offset;
    BOOL offsetValid;
    uint64_t *ptr = (uint64_t *)pUserUartConfig;

    offsetValid = pld->paramNum < USER_UART_MAX_PARAM;        
    
    if(offsetValid){
        offset = pld->paramNum;
        pld->parameter = ptr[offset];
        *payloadLen     = 8 + 4;           // parameter + number
    } else  {
       *payloadLen     = 4;                // number
        pld->paramNum  = INVALID_PARAM;    // invalid
    }

    return TRUE;

}


/** ****************************************************************************
 * @name  GetAllUserParams
 * @brief Retrieves specified number of user configuration parameters data for 
 *        sending to the external host starting from specified offset in user 
 *        configuration structure (refer to UserConfigParamOffset structure for
 *        specific value of offsets)
 * @param [in] pointer to userData payload in the packet
 * @retval N/A
 ******************************************************************************/
BOOL GetAllUserParams(allUserParamsPayload*  pld, uint8_t *payloadLen)
{
    uint32_t offset, i, numParams;
    uint64_t *ptr = (uint64_t *)pUserUartConfig;

    numParams   = USER_UART_MAX_PARAM;
    
    offset = 0;

    for (i = 0; i < numParams; i++, offset++){
            pld->parameters[i] = ptr[offset];
    }

    *payloadLen     = numParams* 8;  

    return TRUE;
}

BOOL  OrientationToAscii(uint8_t *asciiOrien, uint16_t hexOrien)
{

   uint64_t orientation = 0LL;

   uint16_t tmp = hexOrien;
   uint16_t fwdSign;
   uint16_t rightSign;
   uint16_t downSign;
   uint16_t fwdAxis;
   uint16_t rightAxis;
   uint16_t downAxis;

// Forward axis
    fwdSign   = tmp & 0x0001U;

    if(fwdSign == 0){
        fwdSign = 0x2B;
    }else {
        fwdSign = 0x2D;
    }
    fwdAxis   = (tmp >> 1U) & 0x0003U;
    
    switch(fwdAxis){
        case 0:
            fwdAxis = 0x58; // X
            break;
        case 1:
            fwdAxis = 0x59; // Y
            break;
        case 2:
            fwdAxis = 0x5A; // Z
            break;
        case 3:
            return FALSE;   // invalid
    }

// Right axis

   rightSign = (tmp >> 3U) & 0x0001U;
   rightAxis = (tmp >> 4U) & 0x0003U;

    if(rightSign == 0){
        rightSign = 0x2B;
    }else {
        rightSign = 0x2D;
    }
    
    switch(rightAxis){
        case 0:
            rightAxis = 0x59; // Y
            break;
        case 1:
            rightAxis = 0x5A; // Z
            break;
        case 2:
            rightAxis = 0x58; // X
            break;
        case 3:
            return FALSE;   // invalid
    }

// Down axis
   downSign  = (tmp >> 6U) & 0x0001U;
   downAxis  = (tmp >> 7U) & 0x0003U;

    if(downSign == 0){
        downSign = 0x2B;
    }else {
        downSign = 0x2D;
    }
    
    switch(downAxis){
        case 0:
            downAxis = 0x5A; // Z
            break;
        case 1:
            downAxis = 0x58; // X
            break;
        case 2:
            downAxis = 0x59; // Y
            break;
        case 3:
            return FALSE;   // invalid
    }

    orientation        = ((uint64_t)fwdSign)            |
                         ((uint64_t)fwdAxis   << 8)     |
                         ((uint64_t)rightSign << 16)    |
                         ((uint64_t)rightAxis << 24)    |
                         ((uint64_t)downSign  << 32)    |
                         ((uint64_t)downAxis  << 40)    ; 

    memcpy(asciiOrien, (uint8_t *)&orientation, 8);
    return TRUE;
}


void      UpdateUARTAccelFilterSettings(uint16_t data)
{
    pUserUartConfig->uartLpfAccelFilterFreq = data;
}

void      UpdateUARTRateFilterSettings(uint16_t data)
{
    pUserUartConfig->uartLpfRateFilterFreq = data;
}

void      UpdateUARTOrientationSettings(uint16_t data)
{
    OrientationToAscii((uint8_t *)&pUserUartConfig->uartOrientation,  data);
}

