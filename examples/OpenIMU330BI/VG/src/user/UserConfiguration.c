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
#include "eepromAPI.h"
#include "configurationAPI.h"
#include "hwAPI.h"

#include "UserConfiguration.h"
#include "Indices.h"
#include "UserCommunicationSPI.h"


// Default user configuration structure
// Saved into EEPROM of first startup after reloading the code
// or as a result of processing "rD" command
// Do Not remove - just add extra parameters if needed
// Change default settings  if desired
const UserConfigurationStruct gDefaultUserConfig = {
    .dataCRC             =  0,
    .dataSize            =  sizeof(UserConfigurationStruct),
    .userUartBaudRate    =  115200,  
    .userPacketType      =  "a1",  
    .userPacketRate      =  50,  
    .lpfAccelFilterFreq  =  25,
    .lpfRateFilterFreq   =  25,
    .orientation         =  "+X+Y+Z",
    .gyroRange           = SPI_RATE_SENSOR_RANGE_500,   // 500 DPS
    .accelRange          = SPI_ACCEL_SENSOR_RANGE_8G,     // 8G
    // add default parameter values here, if desired
    .spiOrientation         = 0x006b,           //-X -Y -Z
    .spiSyncRate         = 1,               // 200Hz
    .appBehavior         = APP_BEHAVIOR_USE_EXT_SYNC,
    .extSyncFreq         = 1,               // 1Hz
    .spiAccelLpfType        = IIR_20HZ_LPF,    // Butterworth 20Hz      
    .spiGyroLpfType         = IIR_20HZ_LPF,    // Butterworth 20Hz
};

UserConfigurationStruct gUserConfiguration;
UserConfigurationStruct gTmpUserConfiguration;

uint8_t UserDataBuffer[2048];
BOOL configValid = FALSE;


void ApplyUserConfiguration()
{
    uint64_t *ptr       = (uint64_t*)&gUserConfiguration;
    int       size      = sizeof(gUserConfiguration);             // total size in bytes
    BOOL factoryMode    = EEPROM_IsFactoryMode();                 // should be always FALSE when vital system parameters are intact 

    // sanity check for maximum size of user config structure;
    if(size >= 0x800){
        while(1);           
    }

    // Validate checksum of user configuration structure
    configValid = validateUserConfigInEeprom(&size);
    
    if(!configValid || factoryMode) {
        // Load default user parameters but do not apply them to the system 
        memcpy((void*)&gUserConfiguration, (void*)&gDefaultUserConfig, sizeof(UserConfigurationStruct));
    } else {
        // Here we have validated User configuration image.
        // Load it from eeprom into ram on top of the default configuration
        loadUserConfigFromEeprom((void*)&gUserConfiguration, &size);
    }

    // assign new actual size
    gUserConfiguration.dataSize = sizeof(UserConfigurationStruct);
    
    // apply parameters to the platform
    for(int i = USER_UART_BAUD_RATE; i <= USER_LAST_SYSTEM_PARAM && !factoryMode; i++){
        UpdateSystemParameter(i, ptr[i], TRUE);
    }

    if(fSPI){
        configApplyOrientation((uint16_t )gUserConfiguration.spiOrientation);
        configSetAccelSensorFilterTypeForSPI((uint16_t )gUserConfiguration.spiAccelLpfType);
        configSetRateSensorFilterTypeForSPI((uint16_t )gUserConfiguration.spiGyroLpfType);
    }

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
    int tmp;

     if(number < USER_CRC || number >= USER_MAX_PARAM ){
         return FALSE;
     }

     switch (number) {
            case USER_UART_BAUD_RATE:
                result = configSetBaudRate((int)data, fApply);
                break;
            case USER_UART_PACKET_TYPE:
                result = setUserPacketType((uint8_t*)&data, fApply);
                break;
            case USER_UART_PACKET_RATE:
                result = configSetPacketRate((int)data, fApply);
                break;
            case USER_LPF_ACCEL_TYPE:
                result = configSelectUserLPFilter(ACCEL_SENSOR, (uint32_t)data, fApply);
                break;
            case USER_LPF_RATE_TYPE:
                result = configSelectUserLPFilter(RATE_SENSOR, (uint32_t)data, fApply);
                break;
            case  USER_ORIENTATION:
                result = configSetUserOrientation((uint16_t*)&data, fApply);
                break;
            case  USER_GYRO_RANGE:
                tmp = *(int*)&data;
                tmp = tmp*125/2; 
                result = configSetGyroRange(&tmp, fApply); 
                break;
            case  USER_ACCEL_RANGE:
                result = configSetAccelRange((int*)&data, fApply);
                break;
            case  USER_CRC:
            case  USER_DATA_SIZE:
            case  USER_SPI_SYNC_RATE:
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
            configApplyOrientation((uint16_t )data);
            break;
        case USER_SPI_ACCEl_LPF:
            configSetAccelSensorFilterTypeForSPI((uint16_t )data);
            break;
        case USER_SPI_RATE_LPF:
            configSetRateSensorFilterTypeForSPI((uint16_t )data);
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
    status = saveUserConfigInEeprom((uint8_t *)&gUserConfiguration, size);

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

