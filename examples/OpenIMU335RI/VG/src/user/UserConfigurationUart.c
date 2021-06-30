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
#include "configurationAPI.h"
#include "UserConfigurationUart.h"
#include "UserMessagingUART.h"
#include "Indices.h"
#include "eepromAPI.h"
#include "EcuSettings.h"

uint8_t UserDataBuffer[4096];


/*******************************************
 * @brief 
 * 
 * @param pSettings ==
********************************************/
void GetCurrentUartSettings(userUartConfig_t *pSettings)
{
    memcpy(pSettings, pUserUartConfig, sizeof(userUartConfig_t));
}

/*******************************************
 * @brief 
 * 
 * @param pSettings ==
********************************************/
void FillCurrentUartSettings(userUartConfig_t *pSettings)
{
    memcpy(pUserUartConfig, pSettings, sizeof(userUartConfig_t));
}


/** ***************************************************************************
 * @name UpdateUserUartParameter - updating of uart configuration parameter based of user preferences 
 * @brief
 *
 * @param [in]  number - parameter number in user configuration structure
 * @param [in]  data   - value of the parameter in little endian format
 * @retval error (1), no error (0)
 ******************************************************************************/
// NEEDS TO BE CHECKED
BOOL  UpdateUserUartParameter(uint32_t number, uint64_t data, BOOL fApply)
{
     BOOL result = TRUE;
     uint64_t *ptr = (uint64_t *)pUserUartConfig;

     if(number < USER_UART_START || number >= USER_UART_MAX_PARAM ){
         return FALSE;
     }

     switch (number) {
             case USER_UART_BAUD_RATE:
                result = config_SetBaudRate((int32_t)data, fApply);
                if(result){
                    pUserUartConfig->userUartBaudRate = (int32_t)data;
                }
                break;
            case USER_UART_PACKET_TYPE:
                result = SetUserPacketType((uint8_t*)&data, fApply);
                if(result){
                    *(uint64_t*)pUserUartConfig->userPacketType = data;
                }
                break;
            case USER_UART_PACKET_RATE:
                result = config_SetPacketRate((uint32_t)data, fApply);
                if(result){
                    pUserUartConfig->userPacketRate = (int32_t)data;
                }
                break;
            case USER_UART_LPF_ACCEL_TYPE:
                result = config_SelectUserLPFilter(ACCEL_SENSOR, (int32_t)data, TRUE);
                if(result){
                    pUserUartConfig->lpfAccelFilterFreq = (int32_t)data;
                }
                break;
            case USER_UART_LPF_RATE_TYPE:
                result = config_SelectUserLPFilter(RATE_SENSOR, (int32_t)data, TRUE);
                if(result){
                    pUserUartConfig->lpfRateFilterFreq = (int32_t)data;
                }
                break;
            case USER_UART_ORIENTATION:
                result =  ApplyUserUartOrientation((uint16_t*)&data, TRUE);
                if(result){
                    *(int64_t*)pUserUartConfig->orientation = data;
                }
                break;
        // case USER_XXX:  add function calls here if parameter XXXX
        //                        required be updated on the fly
        //             break;
        default:
            result = TRUE;
            break;
    }
    
    if(result == TRUE){
        ptr[number] = data;
    }

    return result;
}


void UserInitConfigureUart()
{

    uint64_t *ptr  = (uint64_t*)pUserUartConfig;
   
    // apply UART parameters to the platform
    for(int i = USER_UART_BAUD_RATE; i <= USER_UART_PACKET_RATE; i++){
        UpdateUserUartParameter(i, ptr[i], TRUE);
    }
    // Fill parameters for UART communication
    BackFillUartDataStructure();

} 

/*******************************************
 * @brief 
 * 
 * @param pld ==
 * @param payloadLen ==
 * @return BOOL 
********************************************/
BOOL UpdateUserUartConfig(userConfigPayload*  pld, uint8_t *payloadLen)
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
            ret = UpdateUserUartParameter(offset, pld->parameters[i], FALSE);
            offset++;
        }
        if(ret == TRUE){
            // Apply parameters values here
            offset = pld->paramOffset;
            for (i = 0; i < pld->numParams; i++){
                ret = UpdateUserUartParameter(offset, pld->parameters[i], TRUE);
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

/*******************************************
 * @brief 
 * 
 * @param pld ==
 * @param payloadLen ==
 * @return BOOL 
********************************************/
BOOL UpdateUserUartParam(userParamPayload*  pld, uint8_t *payloadLen)
{
    BOOL offsetValid;
    BOOL ret = TRUE;
    int32_t result = 0;

    offsetValid = pld->paramNum <=  USER_UART_MAX_PARAM;        
    
    if(offsetValid){
        // Validate parameter first
        ret = UpdateUserUartParameter(pld->paramNum, pld->parameter, FALSE);
        if(ret == TRUE){
            // Apply parameter if valid
            ret = UpdateUserUartParameter(pld->paramNum, pld->parameter, TRUE);
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
BOOL UpdateAllUserUartParams(allUserParamsPayload*  pld, uint8_t *payloadLen)
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
            ret = UpdateUserUartParameter(offset, pld->parameters[i], FALSE);
            offset++;
        }
        if(ret == TRUE){
            // Apply parameters here
            offset = 0;
            for (i = 0; i < numParams; i++){
                UpdateUserUartParameter(offset, pld->parameters[i], TRUE);
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



/*******************************************************************************
 * @name  GetUserUartConfig
 * @brief Retrieves specified number of user configuration parameters data for 
 *        sending to the external host starting from specified offset in user 
 *        configuration structure (refer to UserConfigParamOffset structure for
 *        specific value of offsets)
 * @param [in] pointer to userData payload in the packet
 * @retval N/A
 ******************************************************************************/
BOOL GetUserUartConfig(userConfigPayload*  pld, uint8_t *payloadLen)
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

/*******************************************
 * @brief Get the User Param object
 * 
 * @param pld ==
 * @param payloadLen ==
 * @return BOOL 
********************************************/
BOOL GetUserUartParam(userParamPayload*  pld, uint8_t *payloadLen)
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

/*******************************************
 * @brief Get the All User Params object
 * 
 * @param pld ==
 * @param payloadLen ==
 * @return BOOL 
********************************************/
BOOL GetAllUserUartParams(allUserParamsPayload*  pld, uint8_t *payloadLen)
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


/*******************************************
 * @brief 
 * 
 * @param input ==
 * @param fApply ==
 * @return BOOL 
********************************************/
BOOL ApplyUserUartOrientation(uint16_t *input, BOOL fApply)
{
    BOOL res;
    uint32_t  orientation = 0;
    uint16_t  sel;
    volatile  int i,j;

    for(i = FORWARD; i <= DOWN; i++){
        sel = input[i];
        j = i;
        switch(sel){
            case PLUS_X:
                switch(j){
                    case FORWARD:
                        orientation |= FWD_X_PLUS_MASK;
                        break;  
                    case RIGHT:
                        orientation |= RIGHT_X_PLUS_MASK;
                        break;
                    case DOWN:
                        orientation |= DOWN_X_PLUS_MASK;
                        break;
                    default:   
                        return FALSE;
                }
                break;
            case PLUS_Y:
                switch(j){
                    case FORWARD:
                        orientation |= FWD_Y_PLUS_MASK;
                        break; 
                    case RIGHT:
                        orientation |= RIGHT_Y_PLUS_MASK;
                        break;
                    case DOWN:
                        orientation |= DOWN_Y_PLUS_MASK;;
                        break;
                    default:
                        return FALSE;
                }
                break;
            case PLUS_Z:
                switch(j){
                    case FORWARD:
                        orientation |= FWD_Z_PLUS_MASK;
                        break; 
                    case RIGHT:
                        orientation |= RIGHT_Z_PLUS_MASK;
                        break;
                    case DOWN:
                        orientation |= DOWN_Z_PLUS_MASK;
                        break;
                    default:
                        return FALSE;
                }
                break;
            case MINUS_X:
                switch(j){
                    case FORWARD:
                        orientation |= FWD_X_MINUS_MASK;
                        break;  
                    case RIGHT:
                        orientation |= RIGHT_X_MINUS_MASK;
                        break;
                    case DOWN:
                        orientation |= DOWN_X_MINUS_MASK;
                        break;
                    default:
                        return FALSE;
                }
                break;
            case MINUS_Y:
                switch(j){
                    case FORWARD:
                        orientation |= FWD_Y_MINUS_MASK;
                        break; 
                    case RIGHT:
                        orientation |= RIGHT_Y_MINUS_MASK;
                        break;
                    case DOWN:
                        orientation |= DOWN_Y_MINUS_MASK;
                        break;
                    default:
                        return FALSE;
                }
                break;
           case MINUS_Z:
                switch(j){
                    case FORWARD:
                        orientation |= FWD_Z_MINUS_MASK;
                        break; 
                    case RIGHT:
                        orientation |= RIGHT_Z_MINUS_MASK;
                        break;
                    case DOWN:
                        orientation |= DOWN_Z_MINUS_MASK;
                        break;
                    default:
                        return FALSE;
                }
                break;
            default:
                return FALSE;
        }

    }

    res = config_ApplyOrientation(orientation, fApply);

    if (res == FALSE || !fApply) {
        return res;
    }

    return TRUE;

}

/*******************************************
 * @brief 
 * 
 * @param asciiOrien ==
 * @return BOOL 
********************************************/
BOOL  OrientationToAscii(uint8_t *asciiOrien)
{

   uint64_t orientation = 0LL;

   uint16_t tmp = config_GetOrientation();
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


