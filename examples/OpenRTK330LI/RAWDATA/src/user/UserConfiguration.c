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
#include "stm32f4xx_hal.h"
#include "string.h"

#include "gpsAPI.h"
// #include "magAPI.h"
#include "platformAPI.h"
#include "eepromAPI.h"
#include "ntripClient.h"
#include "UserConfiguration.h"
#include "Indices.h"
#include "configurationAPI.h"
#include "algorithmAPI.h"
#include "osapi.h"
#include "crc16.h"
#include "uart.h"
// Default user configuration structure
// Applied to unit upon reception of "zR" command
// Do Not remove - just add extra parameters if needed
// Change default settings  if desired
const UserConfigurationStruct gDefaultUserConfig = {
    .dataCRC             =  0,
    .dataSize            =  sizeof(UserConfigurationStruct),
    .userUartBaudRate    =  460800,
    .userPacketType      =  "e1",
    .userPacketRate      =  100,
    .lpfAccelFilterFreq  =  25,
    .lpfRateFilterFreq   =  25,
    .orientation         =  "+X+Y+Z",

    // add default parameter values here, if desired
    .profile             = 0,
    .leverArmBx          = 0.0,
    .leverArmBy          = 0.0,
    .leverArmBz          = 0.0,
    .pointOfInterestBx   = 0.0,
    .pointOfInterestBy   = 0.0,
    .pointOfInterestBz   = 0.0,
    .rotationRbvx        = 0.0,
    .rotationRbvy        = 0.0,
    .rotationRbvz        = 0.0,
    
    // ethnet
    .ethMode = ETHMODE_DHCP,
    .staticIp = {192, 168, 1, 110},
	.netmask = {255, 255, 255, 0},
	.gateway = {192, 168, 1, 1},
	.mac = {2, 0, 0, 0, 0, 0},
    
    // ntrip
    .rtkType = LocalRTK,
    .ip = "106.12.40.121", // rtk.ntrip.qxwz.com // rtk.aceinna.com
	.port = 2201,
    .mountPoint = "/RTK",
    .username = "ymj_123",
    .password = "SIGEMZOOMQ1JDJI3",
};

CCMRAM UserConfigurationStruct gUserConfiguration;
const uint8_t *pUserConfigInFlash = (uint8_t *)APP_USER_CONFIG_ADDR;
BOOL configValid = FALSE;

float* get_user_ins_para()
{
    return &(gUserConfiguration.leverArmBx);
}



/** ***************************************************************************
 * @name validateUserConfigInEeprom - validating of user configuration structure 
 *       in EEPROM
 *       changes should be made to gUserConfiguration before calling this function.
 * @brief
 *
 * @param [in] userConfigSize - pointer to variable, which initialized with the
 *                              size of user configuration structure
 * @retval error (0), no error (1)
 ******************************************************************************/
BOOL EEPROM_ValidateUserConfig(int *userConfigSize)
{
    uint64_t    crc, configCrc, size;
    uint64_t   *dataPtr =  (uint64_t*)pUserConfigInFlash;

    configCrc = dataPtr[0];         // CRC 
    size      = dataPtr[1];         // Total Number of bytes in user config structure in eeprom
    if(size != *userConfigSize){    // check if image fits into user storage in RAM
        return FALSE;
    }
    crc = CalculateCRC((uint8_t*)pUserConfigInFlash + 8, size - 8);
    if(crc == configCrc){
        return TRUE;
    }
    return FALSE;    // 
}

/** ***************************************************************************
 * @name loadUserConfigInEeprom - loading user configuration structure from 
 *       predefined flash sector
 *       changes should be made to gUserConfiguration before calling this function.
 * @brief
 *
 * @param [in] N/A
 * @retval error (0), no error (1)
 ******************************************************************************/
BOOL  EEPROM_LoadUserConfig(uint8_t *ptrToUserConfigInRam, int *userConfigSize)
{
    memcpy(ptrToUserConfigInRam, pUserConfigInFlash, *userConfigSize);
    return TRUE;
}

/** ***************************************************************************
 * @name saveUserConfigInEeprom - saving of user configuration structure un the 
 *       predefined flash sector
 *       changes should be made to gUserConfiguration before calling this function.
 * @brief
 *
 * @param [in] N/A
 * @retval error (0), no error (1)
 ******************************************************************************/
BOOL EEPROM_SaveUserConfig(uint8_t *ptrToUserConfigStruct, int userConfigStructLen)
{
    HAL_StatusTypeDef status = HAL_OK;
    FLASH_EraseInitTypeDef pEraseInit;
    uint32_t    PageError;
    uint16_t    offset   =  0;
    uint16_t    num      =  userConfigStructLen;
    uint32_t    start    =  (uint32_t)pUserConfigInFlash;
    uint32_t   *dataPtr  =  (uint32_t*)ptrToUserConfigStruct;
    uint64_t   *paramPtr =  (uint64_t*)ptrToUserConfigStruct;

    paramPtr[1] = num;  //  Total size of user config structure, including Crc and data size 
    paramPtr[0] = CalculateCRC((uint8_t*)ptrToUserConfigStruct + 8, num - 8);
    // calculate CRC over user configuration structure

    ENTER_CRITICAL();

    status = HAL_FLASH_Unlock();
	__HAL_FLASH_CLEAR_FLAG(0xffff);

	pEraseInit.Banks = FLASH_BANK_1;
	pEraseInit.Sector = FLASH_SECTOR_10;
	pEraseInit.NbSectors = 1;
	pEraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	pEraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;

	status = HAL_FLASHEx_Erase(&pEraseInit, &PageError);
	if (status != HAL_OK)
	{
		EXIT_CRITICAL();
		return FALSE;
	}

	while (num > 0)
    {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, start + offset, *dataPtr++);
        if (status != HAL_OK)
        {
            EXIT_CRITICAL();
            return FALSE;
        }
        offset += 4;
        num -= 4;
    }

    HAL_FLASH_Lock();

    EXIT_CRITICAL();

    return TRUE;
}


void userInitConfigureUnit(void)
{
    uint64_t *ptr = (uint64_t*)&gUserConfiguration;
    uint8_t *ptrUser = (uint8_t*)&gUserConfiguration.profile;
    uint8_t offset = 0;
    int size = sizeof(gUserConfiguration);

    // Validate checksum of user configuration structure
    configValid = EEPROM_ValidateUserConfig(&size);
    printf("configValid = %d\r\n", configValid);
    //printf("sizeof(gUserConfiguration) = %d\r\n", size);
    
    if (configValid == TRUE) {
        // Here we have validated User configuration image.
        // Load it from eeprom into ram on top of the default configuration
        EEPROM_LoadUserConfig((void*)&gUserConfiguration, &size);
    } else{
        RestoreDefaultUserConfig();
    }

    for (int i = USER_UART_BAUD_RATE; i <= USER_LAST_SYSTEM_PARAM; i++)
    {
        UpdateSystemParameter(i, ptr[i], TRUE);
    }

    for (int i = USER_LAST_SYSTEM_PARAM+1; i < USER_MAX_PARAM; i++)
    {
        ptrUser = ptrUser + offset;
        offset = UpdateUserParameter(i, ptrUser, TRUE);
    }
/*
    printf("gUserConfiguration.leverArmBx = %f\r\n",gUserConfiguration.leverArmBx);
    printf("gUserConfiguration.leverArmBy = %f\r\n",gUserConfiguration.leverArmBy);
    printf("gUserConfiguration.leverArmBz = %f\r\n",gUserConfiguration.leverArmBz);
    printf("gUserConfiguration.pointOfInterestBx = %f\r\n",gUserConfiguration.pointOfInterestBx);
    printf("gUserConfiguration.pointOfInterestBy = %f\r\n",gUserConfiguration.pointOfInterestBy);
    printf("gUserConfiguration.pointOfInterestBz = %f\r\n",gUserConfiguration.pointOfInterestBz);
    printf("gUserConfiguration.rotationRbvx = %f\r\n",gUserConfiguration.rotationRbvx);
    printf("gUserConfiguration.rotationRbvy = %f\r\n",gUserConfiguration.rotationRbvy);
    printf("gUserConfiguration.rotationRbvz = %f\r\n",gUserConfiguration.rotationRbvz);
*/
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
            case USER_ORIENTATION:
                result = configSetUserOrientation((uint16_t*)&data, fApply);
                break;
            case USER_CRC:
            case USER_DATA_SIZE:
                return TRUE;
        default:
            result = 0;
            break;
    }
    
    if(result == TRUE){
        ptr[number] = data;
    }

    return result;
}

/** ***************************************************************************
 * @name UpdateUserParameter - updating of user configuration parameter based of user preferences 
 * @brief
 *
 * @param [in]  number - parameter number in user configuration structure
 * @param [in]  data   - value of the parameter in little endian format
 * @retval error (0), no error (>=1)
 ******************************************************************************/
uint8_t UpdateUserParameter(uint32_t number, uint8_t* data, BOOL fApply)
{
    uint8_t dataLen = 0;

    if (number <= USER_LAST_SYSTEM_PARAM || number >= USER_MAX_PARAM )
    {
        return 0;
    }
    
    float *tmp;
    switch (number)
    {
    case USER_PROFILE:
        gUserConfiguration.profile = *(uint32_t *)data;
        dataLen = 4;
        break;
    case USER_LEVER_ARM_BX:
        tmp = (float *)data;
        gUserConfiguration.leverArmBx = *tmp;
        setLeverArm((real)gUserConfiguration.leverArmBx,
                    (real)gUserConfiguration.leverArmBy,
                    (real)gUserConfiguration.leverArmBz);
        dataLen = 4;
        break;
    case USER_LEVER_ARM_BY:
        tmp = (float *)data;
        gUserConfiguration.leverArmBy = *tmp;
        setLeverArm((real)gUserConfiguration.leverArmBx,
                    (real)gUserConfiguration.leverArmBy,
                    (real)gUserConfiguration.leverArmBz);
        dataLen = 4;
        break;
    case USER_LEVER_ARM_BZ:
        tmp = (float *)data;
        gUserConfiguration.leverArmBz = *tmp;
        setLeverArm((real)gUserConfiguration.leverArmBx,
                    (real)gUserConfiguration.leverArmBy,
                    (real)gUserConfiguration.leverArmBz);
        dataLen = 4;
        break;
    case USER_POINT_OF_INTEREST_BX:
        tmp = (float *)data;
        gUserConfiguration.pointOfInterestBx = *tmp;
        dataLen = 4;
        break;
    case USER_POINT_OF_INTEREST_BY:
        tmp = (float *)data;
        gUserConfiguration.pointOfInterestBy = *tmp;
        dataLen = 4;
        break;
    case USER_POINT_OF_INTEREST_BZ:
        tmp = (float *)data;
        gUserConfiguration.pointOfInterestBz = *tmp;
        dataLen = 4;
        break;
    case USER_ROTATION_RBVX:
        tmp = (float *)data;
        gUserConfiguration.rotationRbvx = *tmp;
        dataLen = 4;
        break;
    case USER_ROTATION_RBVY:
        tmp = (float *)data;
        gUserConfiguration.rotationRbvy = *tmp;
        dataLen = 4;
        break;
    case USER_ROTATION_RBVZ:
        tmp = (float *)data;
        gUserConfiguration.rotationRbvz = *tmp;
        dataLen = 4;
        break;

    case USER_ETHERNET_ETHMODE:
        gUserConfiguration.ethMode = *data;
        dataLen = 1;
        break;
    case USER_ETHERNET_IP:
        memcpy(gUserConfiguration.staticIp, data, 4);
        dataLen = 4;
        break;
    case USER_ETHERNET_NETMASK:
        memcpy(gUserConfiguration.netmask, data, 4);
        dataLen = 4;
        break;
    case USER_ETHERNET_GATEWAY:
        memcpy(gUserConfiguration.gateway, data, 4);
        dataLen = 4;
        break;
    case USER_ETHERNET_MAC:
        memcpy(gUserConfiguration.mac, data, 6);
        dataLen = 6;
        break;
    
    case USER_NTRIP_RTKTYPE:
        gUserConfiguration.rtkType = *data;
        dataLen = 1;
        break;
    case USER_NTRIP_IP:
        dataLen = sizeof(gUserConfiguration.ip);
        memcpy(gUserConfiguration.ip, data, dataLen);
        break;
    case USER_NTRIP_PORT:
        gUserConfiguration.port = *(uint16_t*)data;
        dataLen = 2;
        break;
    case USER_NTRIP_MOUNTPOINT:
        dataLen = sizeof(gUserConfiguration.mountPoint);
        memcpy(gUserConfiguration.mountPoint, data, dataLen);
        break;
    case USER_NTRIP_USERNAME:
        dataLen = sizeof(gUserConfiguration.username);
        memcpy(gUserConfiguration.username, data, dataLen);
        break;
    case USER_NTRIP_PASSWORD:
        dataLen = sizeof(gUserConfiguration.password);
        memcpy(gUserConfiguration.password, data, dataLen);
        break;

    default:
        break;
    }

    return dataLen;
}

BOOL UpdateEthNtripConfig(uint32_t number)
{
    if (number >= USER_ETHERNET_ETHMODE && number <= USER_ETHERNET_GATEWAY)
    {
        netif_ethernet_config_changed();
    }
    if (number >= USER_NTRIP_IP && number <= USER_NTRIP_PASSWORD)
    {
        netif_ntrip_config_changed();
    }
    return true;
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
    BOOL ret = TRUE;
    int32_t result = 0;
    
    uint64_t* temp = (uint64_t*)pld->parameter;
    if (pld->paramNum < USER_MAX_PARAM){
        // Validate parameter first
        ret = UpdateSystemParameter(pld->paramNum, *temp, FALSE);
        if (ret != TRUE){
            ret = UpdateUserParameter(pld->paramNum, pld->parameter, FALSE);
        }
        if(ret >= TRUE){
            // Apply parameter if valid
            ret = UpdateSystemParameter(pld->paramNum, *temp, TRUE);
            if (ret != TRUE){
                ret = UpdateUserParameter(pld->paramNum, pld->parameter, TRUE);
                if (ret)
                {
                    UpdateEthNtripConfig(pld->paramNum);
                }
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
    uint32_t sn0 = *(uint32_t *)(0x1FFF7A10);

    memcpy((void*)&gUserConfiguration, (void*)&gDefaultUserConfig, sizeof(UserConfigurationStruct));
    gUserConfiguration.mac[3] = (sn0 >> 16) & 0xff;
	gUserConfiguration.mac[4] = (sn0 >> 8) & 0xff;
	gUserConfiguration.mac[5] = sn0 & 0xff;

    if(!SaveUserConfig()){
        valid = FALSE;
    }
    return valid;
}

#define STDOUT_FILENO   1
#define STDERR_FILENO   2

__weak int _write(int fd, char* ptr, int len)
{
    if (fd == STDOUT_FILENO || fd == STDERR_FILENO)
    {
        //uart_write_bytes(UART_DEBUG,ptr,len,1);
        HAL_UART_Transmit(&huart_debug, (uint8_t *)ptr, len, 0xFFFF); // FOR TESTING ONLY?
        return len;
    }
    return 0;
}
