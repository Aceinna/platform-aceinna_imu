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

#include "algorithmAPI.h"
#include "gpsAPI.h"
#include "magAPI.h"
#include "platformAPI.h"
#include "userAPI.h"

#include "UserConfiguration.h"
#include "UserMessagingUart.h"
#include "eepromAPI.h"
#include "Indices.h"
#include "sae_j1939.h"

EcuConfigurationStruct    *gEcuConfigPtr;


// Default user configuration structure
// Applied to unit upon reception of "zR" command
// Do Not remove - just add extra parameters if needed
// Change default settings  if desired
const UserConfigurationStruct gDefaultUserConfig = {
    .dataCRC            =  0,
    .dataSize           =  sizeof(UserConfigurationStruct),
    .ecuAddress         = 128,
    .ecuBaudRate        = _ECU_250K,
    .ecuPacketRate      = 1,           // 100Hz
    .ecuFilterFreqAccel = 25,
    .ecuFilterFreqRate  = 25,
    .ecuPacketType = ( 
					   ACEINNA_SAE_J1939_PACKET_SLOPE_SENSOR |
                       ACEINNA_SAE_J1939_PACKET_ANGULAR_RATE |
                       ACEINNA_SAE_J1939_PACKET_ACCELERATION |
//                       ACEINNA_SAE_J1939_PACKET_RAPID_COURSE_UPDATE   |
//                       ACEINNA_SAE_J1939_PACKET_ATTITUDE              |
//                       ACEINNA_SAE_J1939_PACKET_RAPID_POSITION_UPDATE |
                         0
                       ),
    .ecuOrientation = 0,    // +X +Y +Z
    .canTermResistorEnabled = 0,
    .canBaudRateDetectEnabled = 0,
    .userBehavior  = USER_BEHAVIOR_RUN_ALGORITHM_MASK   |
                     USER_BEHAVIOR_USE_MAGS_MASK        |
                     USER_BEHAVIOR_SWAP_PITCH_ROLL      |
                     USER_BEHAVIOR_NWU_FRAME            |
                     USER_BEHAVIOR_USE_GNSS_OVER_CAN    |    // gnss input data coming over CAN bus 
                     USER_BEHAVIOR_SWAP_REQUEST_PGN     |
                     0,
    .algResetPs    = 0,
    .saveCfgPs     = 0,
    .packetTypePs  = 0,
    .packetRatePs  = 0,
    .filterPs      = 0,
    .orientationPs = 0,
    .magAlignPs    = 0,
    .userBehvPs    = 0,
    .statusPs      = 0,
//***********************************************
//  INS - related parameters
//***********************************************
    .gpsBaudRate         =  9600,
    .gpsProtocol         =  UBLOX_BINARY,
    .leverArmBx          = 0.0,
    .leverArmBy          = 0.0,
    .leverArmBz          = 0.0,
    .pointOfInterestBx   = 0.0,
    .pointOfInterestBy   = 0.0,
    .pointOfInterestBz   = 0.0,
// Below are UART-related parameters - kept compatible with other OpenIMU products
    .uartConfig.uartPacketRate          =  0,
    .uartConfig.uartBaudRate            =  115200,  
    .uartConfig.uartPacketType          =  "z1",  
    .uartConfig.uartLpfAccelFilterFreq  =  25,
    .uartConfig.uartLpfRateFilterFreq   =  25,
    .uartConfig.uartOrientation         =  "+X+Y+Z",
    .uartConfig.uartGpsBaudRate         =  0, 
    .uartConfig.uartGpsProtocol         =  0,
    .uartConfig.uartHardIron_X          =  0.0,
    .uartConfig.uartHardIron_Y          =  0.0,
    .uartConfig.uartSoftIron_Ratio      =  1.0,
    .uartConfig.uartSoftIron_Angle      =  0.0,
};

UserConfigurationStruct gUserConfiguration;
UserConfigurationStruct gTmpUserConfiguration;
UserConfigurationUartStruct* pUserUartConfig;

uint8_t UserDataBuffer[4096];
volatile char   *info;
BOOL configValid = FALSE;

void setUserMagAlignParams(magAlignUserParams_t *params)
{
    gUserConfiguration.uartConfig.uartHardIron_X      = params->hardIron_X;
    gUserConfiguration.uartConfig.uartHardIron_Y      = params->hardIron_Y;
    gUserConfiguration.uartConfig.uartSoftIron_Ratio  = params->softIron_Ratio;
    gUserConfiguration.uartConfig.uartSoftIron_Angle  = params->softIron_Angle;
}

void getUserMagAlignParams(magAlignUserParams_t *params)
{
    params->hardIron_X     = gUserConfiguration.uartConfig.uartHardIron_X;
    params->hardIron_Y     = gUserConfiguration.uartConfig.uartHardIron_Y;
    params->softIron_Ratio = gUserConfiguration.uartConfig.uartSoftIron_Ratio;
    params->softIron_Angle = gUserConfiguration.uartConfig.uartSoftIron_Angle;
}


void userInitConfigureUnit()
{
    int       size      = sizeof(gUserConfiguration);        // total size in bytes
    pUserUartConfig     = &gUserConfiguration.uartConfig;

    // sanity check for maximum size of user config structure;
    if(size >= 0x4000){
        while(1);           
    }

    if(EEPROM_IsAppStartedFirstTime()) {
        // comment next line if want to keep previously stored in EEPROM parameters
        // after rebuilding and/or reloading new application 
        LoadDefaultUserConfig(TRUE);
    }

    // Validate checksum of user configuration structure
    configValid = EEPROM_ValidateUserConfig(&size);
    
    if(configValid == TRUE){
        // Here we have validated User configuration image.
        // Load it from eeprom into ram on top of the default configuration
        EEPROM_LoadUserConfig((void*)&gUserConfiguration, &size);
    }else{
        LoadDefaultUserConfig(FALSE);
    }

    // assign new actual size
    gUserConfiguration.dataSize = sizeof(UserConfigurationStruct);
    memset(&gEcuConfig, 0, sizeof(gEcuConfig));

    LoadEcuBankSettings(&gEcuConfig);

    if(EEPROM_IsUserApplicationActive())
    {
        ApplyEcuControlSettings(&gEcuConfig);
        ApplySystemParameters(&gEcuConfig);
    }

#ifndef GPS_OVER_CAN
    SetGpsBaudRate((int) gUserConfiguration.gpsBaudRate, TRUE);
    SetGpsProtocol((int) gUserConfiguration.gpsProtocol, TRUE);
#endif
    

    info = getBuildInfo();
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

BOOL ApplyLegacyConfigParameters()
{
    BOOL fApply = FALSE;
    uint16_t orientation        = GetNewOrientation();
    uint16_t accelFiltr         = GetNewAccelFiltr();
    uint16_t rateFiltr          = GetNewRateFiltr();
    uint16_t ecuAddress         = GetNewEcuAddress();
    uint16_t ecuBaudrate        = GetNewEcuBaudrate();
    uint16_t ecuUartBaudrate    = GetNewEcuUartBaudrate();
    uint16_t ecuUartPacketType  = GetNewEcuPacketType();
    uint16_t ecuUartPacketRate  = GetNewEcuUartPacketRate();

    if(ecuUartBaudrate != 0xFFFF){
        fApply = TRUE;
        gUserConfiguration.uartConfig.uartBaudRate = platformGetBaudRate(ecuUartBaudrate);
    }

    if(ecuUartPacketType != 0xFFFF){
        uint16_t type = ((ecuUartPacketType >> 8) & 0xff) | ((ecuUartPacketType << 8) & 0xff00);
        fApply = TRUE;
        *(uint64_t *)gUserConfiguration.uartConfig.uartPacketType = type;
    }
    
    if(ecuUartPacketRate != 0xFFFF){
        fApply = TRUE;
        gUserConfiguration.uartConfig.uartPacketRate = platformGetPacketRate(ecuUartPacketRate);
    }

    if(orientation != 0xFFFF){
        fApply = TRUE;
        gUserConfiguration.ecuOrientation     = orientation;
    }
    
    if(accelFiltr != 0xFFFF){
        gUserConfiguration.ecuFilterFreqAccel =  platformGetFilterFrequencyFromCounts(accelFiltr);
        fApply = TRUE;
    }

    if(rateFiltr != 0xFFFF){
        fApply = TRUE;
        gUserConfiguration.ecuFilterFreqRate  = platformGetFilterFrequencyFromCounts(rateFiltr);;
    }

    if(ecuAddress != 0xFFFF){
        fApply = TRUE;
        gUserConfiguration.ecuAddress  = ecuAddress;
    }

    if(ecuBaudrate != 0xFFFF){
        fApply = TRUE;
        gUserConfiguration.ecuBaudRate = ecuBaudrate;
    }

    if(fApply){
        if(!SaveUserConfig()){
            return FALSE;
        }
        ResetChanges();
    }

    return TRUE;
}

BOOL LoadDefaultUserConfig(BOOL fSave)
{
    BOOL valid = TRUE;
  // Load default user configuration
    memcpy((void*)&gUserConfiguration, (void*)&gDefaultUserConfig, sizeof(UserConfigurationStruct));

    if(!fSave){
        return TRUE;
    }

    if(!SaveUserConfig()){
        valid = FALSE;
    }
    return valid;
}


/** ***************************************************************************
 * @name  user_config_save() save current configuration to EEPROM while getting 
 *        config save request
* @brief write global parameters of configuration in RAM to EEPROM
 *        
 * @param [in] 
 * @retval 1 successfuk or 0 failure
 ******************************************************************************/ 
BOOL UpdateEcuConfig(EcuConfigurationStruct  *gEcuConfigPtr, BOOL fSave)
{
    // address, baud rate, new PS, orientation and lpf
    gUserConfiguration.ecuAddress        = (uint16_t)gEcuConfigPtr->address;
    gUserConfiguration.ecuBaudRate       = (uint16_t)gEcuConfigPtr->baudRate;
  
    gUserConfiguration.algResetPs        = gEcuConfigPtr->alg_reset_ps;
    gUserConfiguration.saveCfgPs         = gEcuConfigPtr->save_cfg_ps;
    gUserConfiguration.statusPs          = gEcuConfigPtr->status_ps;
    gUserConfiguration.packetRatePs      = gEcuConfigPtr->packet_rate_ps;
    gUserConfiguration.packetTypePs      = gEcuConfigPtr->packet_type_ps;
    gUserConfiguration.filterPs          = gEcuConfigPtr->digital_filter_ps;
    gUserConfiguration.orientationPs     = gEcuConfigPtr->orientation_ps;
    gUserConfiguration.userBehvPs        = gEcuConfigPtr->user_behavior_ps;
    gUserConfiguration.magAlignPs        = gEcuConfigPtr->mag_align_ps;

    if(fSave){
        return SaveUserConfig();
    }
    
    return TRUE;
}

BOOL  SaveEcuConfig(EcuConfigurationStruct  *gEcuConfigPtr)
{
    return UpdateEcuConfig(gEcuConfigPtr, TRUE);
}

void  SaveEcuAddress(uint16_t address)
{
    gUserConfiguration.ecuAddress  = address;
    SaveUserConfig();
}


/** ***************************************************************************
 * @name is_valid_config_command() check incoming SET packets are valid or invalid 
 * @brief a general API of checking supported SET commands
 *        
 * @param [in] ident, identifier message
 *             
 * @retval ACEINNA_J1939_INVALID_IDENTIFIER, ACEINNA_J1939_CONFIG or ACEINNA_J1939_IGNORE
 ******************************************************************************/
ACEINNA_J1939_PACKET_TYPE is_valid_config_command(SAE_J1939_IDENTIFIER_FIELD *ident)
{
   uint8_t pf_val, ps_val;
   
   if (ident == NULL)
     return ACEINNA_J1939_INVALID_IDENTIFIER;
   
   pf_val = ident->pdu_format;
   ps_val = ident->pdu_specific;
   
   if (
       (pf_val == SAE_J1939_PDU_FORMAT_GLOBAL) && 
       (
       ((ps_val >= SAE_J1939_GROUP_EXTENSION_ALGORITHM_RESET) && (ps_val <= SAE_J1939_GROUP_EXTENSION_MAG_ALIGN_CMD)) ||
       (ps_val == SAE_J1939_GROUP_EXTENSION_BANK0) ||
       (ps_val == SAE_J1939_GROUP_EXTENSION_BANK1) ||
       (ps_val == gEcuConfig.user_behavior_ps) ||
       (ps_val == gEcuConfig.alg_reset_ps) ||
       (ps_val == gEcuConfig.status_ps) ||  
       (ps_val == gEcuConfig.packet_rate_ps) ||
       (ps_val == gEcuConfig.packet_type_ps) ||
       (ps_val == gEcuConfig.digital_filter_ps) || 
       (ps_val == gEcuConfig.mag_align_ps) || 
       (ps_val == gEcuConfig.orientation_ps))
    ) {
         return ACEINNA_J1939_CONFIG;
   }
   
   return ACEINNA_J1939_IGNORE;
}

uint8_t GetEcuAddress()
{
    return gUserConfiguration.ecuAddress;
}

int GetEcuBaudRate()
{
    return gUserConfiguration.ecuBaudRate;
}

void SetEcuBaudRate(_ECU_BAUD_RATE rate )
{
    gUserConfiguration.ecuBaudRate = rate;
}

void  SetEcuPacketType(uint16_t type)
{
    gUserConfiguration.ecuPacketType = type;
}

void  SetEcuPacketRate(uint16_t rate)
{
    gUserConfiguration.ecuPacketRate = rate;
}

void  SetEcuFilterFreq(EcuConfigurationStruct  *pEcuConfig)
{
   platformSelectLPFilter(RATE_SENSOR,  pEcuConfig->rate_cut_off, TRUE);
   platformSelectLPFilter(ACCEL_SENSOR, pEcuConfig->accel_cut_off, TRUE);
   gUserConfiguration.ecuFilterFreqAccel  = pEcuConfig->accel_cut_off;
   gUserConfiguration.ecuFilterFreqRate   = pEcuConfig->rate_cut_off;
}

void  SetEcuOrientation(uint16_t orien_bits)
{
    if(platformApplyOrientation(orien_bits)){
        gUserConfiguration.ecuOrientation = orien_bits;
    }
}


void  LoadEcuBankSettings(void *pConfig)
{
    EcuConfigurationStruct *pEcuConfig = (EcuConfigurationStruct *)pConfig;

    // Add/Remove/Verify payload specific codes here
    // If Payload-specific code here is zero - it will be substituted by predefined code
    // in function  initialize_j1939_config(uint16_t baudRate, uint8_t address)

    pEcuConfig->alg_reset_ps      = gUserConfiguration.algResetPs;
    pEcuConfig->save_cfg_ps       = gUserConfiguration.saveCfgPs;
    pEcuConfig->packet_type_ps    = gUserConfiguration.packetTypePs;
    pEcuConfig->packet_rate_ps    = gUserConfiguration.packetRatePs;
    pEcuConfig->digital_filter_ps = gUserConfiguration.filterPs;
    pEcuConfig->orientation_ps    = gUserConfiguration.orientationPs;
    pEcuConfig->user_behavior_ps  = gUserConfiguration.userBehvPs;
    pEcuConfig->mag_align_ps      = gUserConfiguration.magAlignPs;
    pEcuConfig->status_ps         = gUserConfiguration.statusPs;

}


BOOL CanTermResistorEnabled()
{
    return gUserConfiguration.canTermResistorEnabled != 0;
}

BOOL CanBaudRateDetectionEnabled()
{
    return gUserConfiguration.canBaudRateDetectEnabled != 0;
}

BOOL UseAlgorithm()
{
    return (gUserConfiguration.userBehavior & USER_BEHAVIOR_RUN_ALGORITHM_MASK) != 0? TRUE : FALSE;  
}

BOOL UseMags()
{
    return (gUserConfiguration.userBehavior & USER_BEHAVIOR_USE_MAGS_MASK) != 0? TRUE : FALSE;  
}

BOOL GnssInputDataOverCan()
{
    return (gUserConfiguration.userBehavior & USER_BEHAVIOR_USE_GNSS_OVER_CAN) != 0? TRUE : FALSE;  
}

BOOL SwapPitchAndroll()
{
    return (gUserConfiguration.userBehavior & USER_BEHAVIOR_SWAP_PITCH_ROLL) != 0? TRUE : FALSE;  
}

BOOL UseNWUFrame()
{
    return (gUserConfiguration.userBehavior & USER_BEHAVIOR_NWU_FRAME) != 0? TRUE : FALSE;  
}

BOOL SwapRequestPGN()
{
    return (gUserConfiguration.userBehavior & USER_BEHAVIOR_SWAP_REQUEST_PGN) != 0? TRUE : FALSE;  
}

BOOL UseAutoBaud()
{
    return (gUserConfiguration.userBehavior & USER_BEHAVIOR_USE_AUTOBAUD) != 0? TRUE : FALSE;  
}



void  ApplySystemParameters(void *pConfig)
{
    EcuConfigurationStruct *pEcuConfig = (EcuConfigurationStruct *)pConfig;

    platformSelectLPFilter(RATE_SENSOR,  pEcuConfig->rate_cut_off, TRUE);
    platformSelectLPFilter(ACCEL_SENSOR, pEcuConfig->accel_cut_off, TRUE);
    platformApplyOrientation(pEcuConfig->orien_bits);
    
    if(UseMags()){
        platformForceMagUsage();
    }
    
    UserInitConfigureUart();
    platformSetEcuBaudrate(gUserConfiguration.ecuBaudRate);
    platformSetEcuAddress(gUserConfiguration.ecuAddress);
}

void  ApplyEcuControlSettings(void *pConfig)
{
    EcuConfigurationStruct *pEcuConfig = (EcuConfigurationStruct *)pConfig;

    // Add/Remove/Verify ECU-specific parameters here
    pEcuConfig->address           = gUserConfiguration.ecuAddress;
    pEcuConfig->baudRate          = gUserConfiguration.ecuBaudRate;
    pEcuConfig->packet_rate       = gUserConfiguration.ecuPacketRate;
    pEcuConfig->accel_cut_off     = gUserConfiguration.ecuFilterFreqAccel;
    pEcuConfig->rate_cut_off      = gUserConfiguration.ecuFilterFreqRate;
    pEcuConfig->packet_type       = gUserConfiguration.ecuPacketType;
    pEcuConfig->orien_bits        = gUserConfiguration.ecuOrientation;
    pEcuConfig->user_behavior     = gUserConfiguration.userBehavior;
    pEcuConfig->baud_rate_detect_enable = UseAutoBaud();
}

void    UpdateEcuOrientationSettings(uint16_t data)
{
    gEcuConfig.orien_bits                 = data;
    gUserConfiguration.ecuOrientation     = data;
}

void    UpdateEcuAccelFilterSettings(uint16_t data)
{
    gEcuConfig.accel_cut_off              = data;
    gUserConfiguration.ecuFilterFreqAccel = data;
}

void    UpdateEcuRateFilterSettings(uint16_t data)
{
    gEcuConfig.rate_cut_off     = data;
    gUserConfiguration.ecuFilterFreqRate = data;
}

void    UpdateEcuUartBaudrate(uint64_t data)
{
    gUserConfiguration.uartConfig.uartBaudRate = data;
}

void    UpdateEcuUartPacketRate(uint64_t data)
{
    gUserConfiguration.uartConfig.uartPacketRate = data;
}

void    UpdateEcuUartPacketType(uint64_t data)
{
    *(uint64_t *)gUserConfiguration.uartConfig.uartPacketType = data;
}


