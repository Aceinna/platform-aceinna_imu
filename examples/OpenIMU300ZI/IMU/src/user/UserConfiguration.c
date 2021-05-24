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

// Default user configuration structure
// Saved into EEPROM of first startup after reloading the code
// or as a result of processing "rD" command
// Do Not remove - just add extra parameters if needed
// Change default settings  if desired
const UserConfigurationStruct gDefaultUserConfig = {
    .dataCRC             =  0,
    .dataSize            =  sizeof(UserConfigurationStruct),
    .userUartBaudRate    =  115200,  
    .userPacketType      =  "z1",  
    .userPacketRate      =  100,  
    .lpfAccelFilterFreq  =  25,
    .lpfRateFilterFreq   =  25,
    .orientation         =  "+X+Y+Z",
    // add default parameter values here, if desired
    .uartGpsBaudRate     =  0, 
    .uartGpsProtocol     =  0,
    .hardIron_X          = 0.0,
    .hardIron_Y          = 0.0,
    .softIron_Ratio      = 1.0,
    .softIron_Angle      = 0.0,
    .appBehavior         = APP_BEHAVIOR_USE_EXT_SYNC,
    .spiOrientation      = 0x0000,          //+X +Y +Z
    .spiSyncRate         = 1,               // 200Hz
    .extSyncFreq         = 1,               // 1Hz
    .spiAccelLpfType     = IIR_20HZ_LPF,    // Butterworth 20Hz      
    .spiGyroLpfType      = IIR_20HZ_LPF,    // Butterworth 20Hz
};

UserConfigurationStruct gUserConfiguration;

uint8_t UserDataBuffer[4096];
volatile char   *info;
BOOL configValid = FALSE;

void setUserMagAlignParams(magAlignUserParams_t *params)
{
    gUserConfiguration.hardIron_X      = params->hardIron_X;
    gUserConfiguration.hardIron_Y      = params->hardIron_Y;
    gUserConfiguration.softIron_Ratio  = params->softIron_Ratio;
    gUserConfiguration.softIron_Angle  = params->softIron_Angle;
}

void getUserMagAlignParams(magAlignUserParams_t *params)
{
    params->hardIron_X     = gUserConfiguration.hardIron_X;
    params->hardIron_Y     = gUserConfiguration.hardIron_Y;
    params->softIron_Ratio = gUserConfiguration.softIron_Ratio;
    params->softIron_Angle = gUserConfiguration.softIron_Angle;
}


void userInitConfigureUnit()
{
    uint64_t *ptr       = (uint64_t*)&gUserConfiguration;
    int       size      = sizeof(gUserConfiguration);        // total size in bytes
    BOOL      factoryMode =   EEPROM_IsConfigSectorLocked()? FALSE : TRUE;
    
    // sanity check for maximum size of user config structure;
    if(size >= 0x4000){
        while(1);           
    }

    if(EEPROM_IsAppStartedFirstTime()) {
        // comment next line if want to keep previously stored in EEPROM parameters
        // after rebuilding and/or reloading new application 
        RestoreDefaultUserConfig();
    }

    // Validate checksum of user configuration structure
    configValid = EEPROM_ValidateUserConfig(&size);
    
    if(configValid == TRUE) {
        // Here we have validated User configuration image.
        // Load it from eeprom into ram on top of the default configuration
        EEPROM_LoadUserConfig((void*)&gUserConfiguration, &size);
    } else {
        // do not apply
        memcpy((void*)&gUserConfiguration, (void*)&gDefaultUserConfig, sizeof(UserConfigurationStruct));
        return;
    }

    // assign new actual size
    gUserConfiguration.dataSize = sizeof(UserConfigurationStruct);
    
    // apply parameters to the platform
    for(int i = USER_USER_BAUD_RATE; i <= USER_LAST_SYSTEM_PARAM && configValid; i++){
        UpdateSystemParameter(i, ptr[i], TRUE);
    }

    // validate and apply own parameters if desired
    for(int i = USER_LAST_SYSTEM_PARAM+1; i < USER_MAX_PARAM && configValid; i++){
        UpdateUserParameter(i, ptr[i], TRUE);
    }

    if(((gUserConfiguration.appBehavior & APP_BEHAVIOR_USE_EXT_SYNC) != 0) && (gUserConfiguration.extSyncFreq == 1) && !factoryMode){
        platformEnableGpsPps(TRUE);
    }
    bool fSPI = platformGetUnitCommunicationType() == SPI_COMM;
    if(fSPI && !factoryMode){
        platformApplyOrientation((uint16_t)gUserConfiguration.spiOrientation);
        platformUpdateAccelFilterType((uint8_t)gUserConfiguration.spiAccelLpfType);
        platformUpdateRateFilterType((uint8_t)gUserConfiguration.spiGyroLpfType);
    }

    info = getBuildInfo();
} 



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
     uint64_t *ptr = (uint64_t *)&gUserConfiguration;
     uint16_t orientOut;

     if(number < USER_CRC || number >= USER_MAX_PARAM ){
         return FALSE;
     }

     switch (number) {
            case USER_USER_BAUD_RATE:
                result = platformSetBaudRate((int)data, fApply);
                break;
            case USER_USER_PACKET_TYPE:
                result = setUserPacketType((uint8_t*)&data, fApply);
                break;
            case USER_USER_PACKET_RATE:
                result = platformSetPacketRate((int)data, fApply);
                break;
            case USER_LPF_ACCEL_TYPE:
                result = platformSelectLPFilter(ACCEL_SENSOR, (uint32_t)data, fApply);
                break;
            case USER_LPF_RATE_TYPE:
                result = platformSelectLPFilter(RATE_SENSOR, (uint32_t)data, fApply);
                break;
            case  USER_ORIENTATION:
                result = platformSetOrientation((uint16_t*)&data, &orientOut, fApply);
                break;
            case  USER_CRC:
            case  USER_DATA_SIZE:
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


/** ***************************************************************************
 * @name UpdateUserParameter - updating user configuration parameter based of preferences 
 * @brief
 *
 * @param [in]  number - parameter number in user configuration structure
 * @param [in]  data   - value of the parameter in little endian format
 * @retval error (1), no error (0)
 ******************************************************************************/
// NEEDS TO BE CHECKED
BOOL  UpdateUserParameter(uint32_t number, uint64_t data, BOOL fApply)
{
     BOOL result;
     uint64_t *ptr = (uint64_t *)&gUserConfiguration;

     if(number <= USER_LAST_SYSTEM_PARAM || number >= USER_MAX_PARAM ){
         return FALSE;
     }

     switch (number) {
        case USER_SPI_ORIENTATION:
            platformApplyOrientation((uint16_t )data);
            break;
        case USER_SPI_ACCEl_LPF:
            platformUpdateAccelFilterType((uint8_t )data);
            break;
        case USER_SPI_RATE_LPF:
            platformUpdateRateFilterType((uint8_t )data);
            break;
        //case: 
        //    add function calls here if parameter XXXX
        //    required be updated on the fly and/or validated
        //    break;
        default:
            // by default result should be true if there is no special
            // consideration or criteria for parameter validity
            result = TRUE;
            break;
    }

    if(result == TRUE){
        ptr[number] = data;
    }
   
    return result;
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
    uint32_t offset, i, maxParam;
    BOOL offsetValid = TRUE;
    BOOL lenValid = TRUE;
    BOOL numValid = TRUE;
    BOOL ret = FALSE;
    int32_t result = 0;

    maxParam    = sizeof(UserConfigurationStruct)/8;

    // Validate parameters numbers and quantity 
    if(pld->numParams  > MAX_NUMBER_OF_USER_PARAMS_IN_THE_PACKET){
        lenValid = FALSE;
        result   = INVALID_PAYLOAD_SIZE;
    }

    if(pld->paramOffset >= maxParam){
        offsetValid = FALSE;        
        result      = INVALID_PARAM;
    }

    if((pld->numParams + pld->paramOffset) > maxParam){
        numValid = FALSE;        
        result   = INVALID_PARAM;
    }
    
    if(offsetValid && numValid && lenValid){
        // Validate parameters values first
        offset = pld->paramOffset;
        for (i = 0; i < pld->numParams; i++){
            ret = UpdateSystemParameter(offset, pld->parameters[i], FALSE);
            if (ret != TRUE){
                ret = UpdateUserParameter(offset, pld->parameters[i], FALSE);
                if (ret != TRUE){
                    result  = INVALID_VALUE;
                    break;
                }
            }
            offset++;
        }
        if(ret == TRUE){
            // Apply parameters values here
            offset = pld->paramOffset;
            for (i = 0; i < pld->numParams; i++){
                ret = UpdateSystemParameter(offset, pld->parameters[i], TRUE);
                if (ret != TRUE){
                    ret = UpdateUserParameter(offset, pld->parameters[i], TRUE);
                }
                offset++;
            }
        }
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
    uint32_t maxParam;
    BOOL offsetValid;
    BOOL ret = TRUE;
    int32_t result = 0;

    maxParam    = sizeof(UserConfigurationStruct)/8;
    offsetValid = pld->paramNum <  maxParam;        
    
    if(offsetValid){
        // Validate parameter first
        ret = UpdateSystemParameter(pld->paramNum, pld->parameter, FALSE);
        if (ret != TRUE){
            ret = UpdateUserParameter(pld->paramNum, pld->parameter, FALSE);
        }
        if(ret == TRUE){
            // Apply parameter if valid
            ret = UpdateSystemParameter(pld->paramNum, pld->parameter, TRUE);
            if (ret != TRUE){
                ret = UpdateUserParameter(pld->paramNum, pld->parameter, TRUE);
            }
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
    uint32_t offset, i, maxParam;
    BOOL lenValid = TRUE;
    BOOL numValid = TRUE;
    BOOL ret = FALSE;
    int32_t    result = 0; 

    int    numParams = (*payloadLen)/8;
    maxParam  = sizeof(UserConfigurationStruct)/8;

    if(numParams  > MAX_NUMBER_OF_USER_PARAMS_IN_THE_PACKET){
        lenValid = FALSE;
        result   = INVALID_PAYLOAD_SIZE;
    }

    if(numParams  > maxParam){
        numValid = FALSE;
        result   = INVALID_PARAM;
    }
    
    if(numValid && lenValid){
        // Validate parameters here
        offset = 0;
        for (i = 0; i < numParams; i++){
            ret = UpdateSystemParameter(offset, pld->parameters[i], FALSE);
            if (ret != TRUE){
                ret = UpdateUserParameter(offset, pld->parameters[i], FALSE);
                if (ret != TRUE){
                    result  = INVALID_VALUE;
                    break;
                }
            }
            offset++;
        }
        if(ret == TRUE){
            // Apply parameters here
            offset = 0;
            for (i = 0; i < numParams; i++){
                ret = UpdateSystemParameter(offset, pld->parameters[i], TRUE);
                if (ret != TRUE){
                    ret = UpdateUserParameter(offset, pld->parameters[i], TRUE);
                }
                offset++;
            }
        }
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
    uint32_t offset, i, maxParam;
    BOOL offsetValid = TRUE;
    BOOL lenValid = TRUE;
    uint64_t *ptr = (uint64_t *)&gUserConfiguration;

    maxParam    = sizeof(UserConfigurationStruct)/8;

    offsetValid = pld->paramOffset < maxParam;        

    lenValid    = ((pld->numParams + pld->paramOffset) <= maxParam) && 
                   (pld->numParams <= MAX_NUMBER_OF_USER_PARAMS_IN_THE_PACKET);   
    
    if(offsetValid && lenValid){
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
    uint32_t offset, maxParam;
    BOOL offsetValid;
    uint64_t *ptr = (uint64_t *)&gUserConfiguration;

    maxParam    = sizeof(UserConfigurationStruct)/8;
    offsetValid = pld->paramNum < maxParam;        
    
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
    uint64_t *ptr = (uint64_t *)&gUserConfiguration;

    numParams   = sizeof(UserConfigurationStruct)/8;
    
    offset = 0;
    for (i = 0; i < numParams; i++, offset++){
            pld->parameters[i] = ptr[offset];
    }

    *payloadLen     = numParams* 8;  

    return TRUE;
}



/** ***************************************************************************
 * @name SaveUserConfig - saving of user configuration structure un the 
 *       predefined flash sector
 * @brief
 *
 * @param [in] N/A
 * @retval error (0), no error (1)
 ******************************************************************************/
BOOL  SaveUserConfig(void)
{
    int size;
    BOOL status;

    size   = sizeof(UserConfigurationStruct);
    status = EEPROM_SaveUserConfig((uint8_t *)&gUserConfiguration, size);

    if(status){
        return TRUE; 
    }

    return FALSE;

}


BOOL RestoreDefaultUserConfig(void)
{
    BOOL valid = TRUE;
  // Load default user configuration
    memcpy((void*)&gUserConfiguration, (void*)&gDefaultUserConfig, sizeof(UserConfigurationStruct));
    if(!SaveUserConfig()){
        valid = FALSE;
    }
    return valid;
}

BOOL UpdateBias(void) {
    fUpdateBias = TRUE;
    return  TRUE;
}


BOOL ExtSyncEnabled()
{
    return (gUserConfiguration.appBehavior & APP_BEHAVIOR_USE_EXT_SYNC) != 0;
}

int ExtSyncFrequency()
{
    return  gUserConfiguration.extSyncFreq;
}


uint8_t SpiSyncRate()
{
    return (uint8_t)gUserConfiguration.spiSyncRate;
}

uint8_t SpiAccelLpfType()
{
    return (uint8_t)gUserConfiguration.spiAccelLpfType;
}

uint8_t SpiGyroLpfType()
{
    return (uint8_t)gUserConfiguration.spiGyroLpfType;
}

uint16_t SpiOrientation()
{
    return (uint16_t)gUserConfiguration.spiOrientation;
}

