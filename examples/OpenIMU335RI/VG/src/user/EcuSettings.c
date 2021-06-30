/** ***************************************************************************
 * @file   ecu_configuration.c
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
#include "configurationAPI.h"

#include "EcuSettings.h"
#include "eepromAPI.h"
#include "Indices.h"
#include "bitAPI.h"
#include "halAPI.h"
#include "odoAPI.h"

static   void    ApplySystemParameters(EcuConfigurationStruct* const pConfig);
static   void    UpdateEcuInstanceSettings();
BOOL             OrientationToAscii(uint8_t *asciiOrien);



static EcuConfigurationStruct  gEcuConfig;
EcuConfigurationStruct         *gEcuConfigPtr = &gEcuConfig;
int32_t   ConfigSaveError = 0;

static uint16_t newAddress       = 0xFFFFU;
static uint16_t newCanBaudRate   = 0xFFFFU;
static uint16_t newUartBaudRate  = 0xFFFFU;
static uint16_t newBehavior      = 0xFFFFU;
static uint16_t newEcuPacketRate = 0xFFFFU;
static uint16_t newEcuPacketType = 0xFFFFU;
static uint16_t newAccelFilter   = 0xFFFFU;
static uint16_t newRateFilter    = 0xFFFFU;
static uint16_t newOrientation   = 0xFFFFU;

// Default user configuration structure
// Applied to unit upon reception of "zR" command
// Do Not remove - just add extra parameters if needed
// Change default settings  if desired
static ecu_settings_struct const  DefaultEcuSettings = {
    .dataCRC            =  0,
    .dataSize           =  sizeof(ecu_settings_struct),
    .ecuAddress         = 128,
    .ecuBaudRate        = ECU_BAUD_250K,
    .ecuPacketRate          = 100,  // 100Hz
    .ecuFilterFreqAccel     = 25,
    .ecuFilterFreqRate      = 25,
    .ecuPacketType = ( 
                        ACEINNA_SAE_J1939_PACKET_SLOPE_SENSOR2     |
                        ACEINNA_SAE_J1939_PACKET_ANGULAR_RATE     | 
//                        ACEINNA_SAE_J1939_PACKET_ANGULAR_RATE_HR  | 
//                        ACEINNA_SAE_J1939_PACKET_ACCELERATION_HR  |
                        ACEINNA_SAE_J1939_PACKET_ACCELERATION     |
                        0    
                     ),
    .ecuOrientation = 0,    // +X +Y +Z
    .userBehavior  = 
                    (uint16_t)(USER_BEHAVIOR_ENABLE_DYN_MOTION     |
                    USER_BEHAVIOR_SWAP_PITCH_AND_ROLL   |
                    USER_BEHAVIOR_ENABLE_AUTO_BAUD      |
                    USER_BEHAVIOR_SWAP_ACCEL_FRAME      |
                    USER_BEHAVIOR_USE_RAW_ACCEL_FOR_ALG |
                    USER_BEHAVIOR_RUN_ALGORITHM_MASK),

    .statusPs      = 0,
    .algResetPs    = 0,
    .saveCfgPs     = 0,
    .packetRatePs  = 0,
    .packetTypePs  = 0,
    .filterPs      = 0,
    .orientationPs = 0,
    .userBehvPs    = 0,
    .algoCoefOfReduceQ       = 10,
    .algoLinAccelSwitchDelay = 2000,
    .algoRateIntegrationTime = 2000,
    .addressChanged = 0,
    .masterStatusPs = 0, 
    .hwStatusPs     = 0,
    .swStatusPs     = 0,
    .algoCtrlPs     = 0,
    .hrRatePs       = 0,
    .hrAccelPs      = 0,
    .aidLvarmPs     = 0,
    .aidConfigPs    = 0,
    .dm1ConfigPs    = 0,
    .odoLeverArmX   = 0,
    .odoLeverArmY   = 0,
    .odoLeverArmZ   = 0,
// DM1 configuration related parameters
    .lamp_status    = 0x04,         // amber light on
    .flash_status   = 0xFF,         // undefined
    .SPN            = 0x07F4B3,     // 521395 - default SPN of message DM1
    .FMI1           = 12,           // Failure Mode Identifier for DTC1
    .FMI2           = 14,           // Failure Mode Identifier for DTC2
// Priorities of messages
    .ariMsgPriority  = 3,
    .accsMsgPriority = 2,
    .ssi2MsgPriority = 3,


// UART - related parameters
    .uartCfg.userUartBaudRate    =  115200,  
    .uartCfg.userPacketType      =  "",         // do not apply 
    .uartCfg.userPacketRate      =  -1,         // do not apply  
    .uartCfg.lpfAccelFilterFreq  =  -1,         // do not apply
    .uartCfg.lpfRateFilterFreq   =  -1,         // do not apply
    .uartCfg.orientation         =  "+Z+Z+Z",   // do not apply

};

static ecu_settings_struct gEcuSettings;
static uint32_t setStat = 0U;
userUartConfig_t *pUserUartConfig = &gEcuSettings.uartCfg;


/******************************************************************************
 * @brief
 * @param fSave [in] sample new data to push onto the queue
 *  
 ******************************************************************************/
static BOOL LoadDefaultEcuSettings(BOOL const fSave)
{
  // Load default user configuration
    memcpy(&gEcuSettings, &DefaultEcuSettings, sizeof(ecu_settings_struct));

    if(!fSave){
        return TRUE;
    }

    return SaveEcuSettings(TRUE, FALSE);

}

/*******************************************
 * @brief 
 * 
 * @param config ==
********************************************/
void    ApplyDm1ConfigFromEeprom(uint64_t config)
{
    gEcuSettings.lamp_status  =  config       & 0xFF;      // byte 0
    gEcuSettings.flash_status = (config >> 8) & 0xFF;      // byte 1
    gEcuSettings.SPN          = (config >> 16) & 0xFFFFFF; // byte 2 - 4 
    gEcuSettings.FMI1         = (config >> 40) & 0xFF;     // byte 5  
    gEcuSettings.FMI2         = (config >> 48) & 0xFF;     // byte6
}


/*******************************************
 * @brief 
 * 
 * @return uint64_t 
********************************************/
uint64_t PrepareDm1ConfigToEeprom()
{
    uint64_t config = 0;
    config  <<= 8;
    config   |= gEcuSettings.FMI2;
    config  <<= 8;
    config   |= gEcuSettings.FMI1;
    config  <<= 24;
    config   |= (gEcuSettings.SPN & 0x00FFFFFF);
    config  <<= 8;
    config   |= gEcuSettings.flash_status;
    config  <<= 8;
    config   |= gEcuSettings.lamp_status;

    return config;
}



/******************************************************************************
 * @brief
 *  
 ******************************************************************************/
void LoadEcuSettings()
{
    uint16_t   size         = (uint16_t)sizeof(gEcuSettings);             // total size in bytes
    BOOL const factoryMode  = EEPROM_IsFactoryMode();                 // should be always FALSE when vital system parameters are intact 
    BOOL const configLoaded = EEPROM_IsConfigLoaded(size);

    uint8_t    ecuAddr;
    uint64_t   dm1Config;

    BOOL       res;
    // Validate checksum of user configuration structure
    BOOL const configValid = EEPROM_ValidateUserConfig(&size);

    if(!configValid){
        // Load default configuration if user configuration
        // did not pass validation
        setStat += LoadDefaultEcuSettings(FALSE);
        BIT_SetInvalidConfigStatus();
    }else{
        // Here we have validated User configuration
        setStat += (uint32_t)EEPROM_LoadUserConfig((void*)&gEcuSettings, &size);
        if(configLoaded){
            res = EEPROM_InvalidateConfigSignature(size);
            if(res){
                setStat += EEPROM_SaveEcuAddress(gEcuSettings.ecuAddress);
            }
        }
    }

    res = EEPROM_GetLastSavedEcuAddress(&ecuAddr);
    
    if(res){
        gEcuSettings.ecuAddress = ecuAddr;
    }

    res = EEPROM_GetLastSavedDM1Config(&dm1Config);

    if(res){
        ApplyDm1ConfigFromEeprom(dm1Config);
    }


    // assign new actual size
    gEcuSettings.dataSize = sizeof(ecu_settings_struct);
    
    memset(&gEcuConfig, 0, sizeof(gEcuConfig));

    // Apply parameters from NV memory to ECU configuration structure 
    ApplyEcuSettings();

    BIT_SetFMICodes(gEcuConfig.FMI1, gEcuConfig.FMI2);

    if(!factoryMode)
    {
        // User Filters and orientation
        ApplySystemParameters(&gEcuConfig);
        // UART settings 
        UserInitConfigureUart();
    }

    UpdateEcuInstanceSettings();
} 

/*******************************************
 * @brief 
 * 
********************************************/
void    BackFillUartDataStructure()
{
    OrientationToAscii(pUserUartConfig->orientation);
    pUserUartConfig->lpfAccelFilterFreq = config_GetFilterFreq(ACCEL_SENSOR, 0U);
    pUserUartConfig->lpfRateFilterFreq  = config_GetFilterFreq(RATE_SENSOR, 0U);
}



/** ***************************************************************************
 * @name SaveUserConfig - saving of user configuration structure un the 
 *       predefined flash sector
 * @brief
 *
 * @param fDefault [in]
 * @return error (0), no error (1)
 ******************************************************************************/

BOOL  SaveEcuSettings(BOOL const fDefault, BOOL const fromUart)
{
    uint16_t size;
    BOOL status;

    if (!fDefault)
    {
        gEcuSettings.ecuAddress         = gEcu.addr;
        if(gEcu.newAddr != 0U){
            gEcuSettings.ecuAddress     = gEcu.newAddr;
        }
        gEcuSettings.ecuBaudRate        = gEcu.baudrate;
        gEcuSettings.ecuPacketRate      = gEcuConfigPtr->packet_rate_div == 0? 0 : 100/gEcuConfigPtr->packet_rate_div;
        gEcuSettings.ecuPacketType      = gEcuConfigPtr->packet_type;
        gEcuSettings.userBehavior       = gEcuConfigPtr->user_behavior;
        if(!fromUart){
            // populate from CAN bus side
            gEcuSettings.ecuOrientation     = gEcuConfigPtr->orien_bits;
            gEcuSettings.ecuFilterFreqAccel = gEcuConfigPtr->accel_cut_off;
            gEcuSettings.ecuFilterFreqRate  = gEcuConfigPtr->rate_cut_off;
        }else{
            // populate from config structure whatever propagated from UART commands
            gEcuSettings.ecuOrientation     = config_GetOrientation();
            gEcuSettings.ecuFilterFreqAccel = config_GetFilterFreq(ACCEL_SENSOR, 0U);
            gEcuSettings.ecuFilterFreqRate  = config_GetFilterFreq(RATE_SENSOR, 0U);
        }
        gEcuSettings.algoLinAccelSwitchDelay = gEcuConfigPtr->limitAccelSwitchDelay;
        gEcuSettings.algoRateIntegrationTime = gEcuConfigPtr->limitRateIntegrationTime;
        gEcuSettings.algoCoefOfReduceQ       = gEcuConfigPtr->coefOfReduceQ;

        gEcuSettings.algResetPs      = gEcuConfigPtr->alg_reset_ps;
        gEcuSettings.saveCfgPs       = gEcuConfigPtr->save_cfg_ps;
        gEcuSettings.packetRatePs    = gEcuConfigPtr->packet_rate_ps;
        gEcuSettings.packetTypePs    = gEcuConfigPtr->packet_type_ps;
        gEcuSettings.filterPs        = gEcuConfigPtr->digital_filter_ps;
        gEcuSettings.orientationPs   = gEcuConfigPtr->orientation_ps;
        gEcuSettings.userBehvPs      = gEcuConfigPtr->user_behavior_ps;
        gEcuSettings.masterStatusPs  = gEcuConfigPtr->master_status_ps;
        gEcuSettings.hwStatusPs      = gEcuConfigPtr->hw_status_ps;
        gEcuSettings.swStatusPs      = gEcuConfigPtr->sw_status_ps;
        gEcuSettings.algoCtrlPs      = gEcuConfigPtr->algo_control_ps;
        gEcuSettings.hrRatePs        = gEcuConfigPtr->hr_rate_ps;
        gEcuSettings.hrAccelPs       = gEcuConfigPtr->hr_accel_ps;
        // Odometer lever arm
        gEcuSettings.odoLeverArmX    = gEcuConfigPtr->odoLeverArmX;
        gEcuSettings.odoLeverArmY    = gEcuConfigPtr->odoLeverArmY;
        gEcuSettings.odoLeverArmZ    = gEcuConfigPtr->odoLeverArmZ;
        // Aiding signal configuration settings
        gEcuSettings.signalSource    = gEcuConfigPtr->signalSource;  
        gEcuSettings.aidingPF        = gEcuConfigPtr->aidingPF;  
        gEcuSettings.aidingPS        = gEcuConfigPtr->aidingPS;  
        gEcuSettings.aidingMsgRate   = gEcuConfigPtr->aidingMsgRate;  
        gEcuSettings.drivingDirPF    = gEcuConfigPtr->drivingDirPF;  
        gEcuSettings.drivingDirPS    = gEcuConfigPtr->drivingDirPS;  
        gEcuSettings.odoCfgSwitch    = gEcuConfigPtr->odoCfgSwitch;  
        // new configurable ps
        gEcuSettings.aidLvarmPs      = gEcuConfigPtr->aid_lvarm_ps;
        gEcuSettings.aidConfigPs     = gEcuConfigPtr->aid_config_ps;
        // DM1 config parameters
        gEcuSettings.dm1ConfigPs     = gEcuConfigPtr->dm1_config_ps;
        gEcuSettings.lamp_status     = gEcuConfigPtr->lamp_status;
        gEcuSettings.flash_status    = gEcuConfigPtr->flash_status;
        gEcuSettings.FMI1            = gEcuConfigPtr->FMI1;
        gEcuSettings.FMI2            = gEcuConfigPtr->FMI2;
        gEcuSettings.SPN             = gEcuConfigPtr->SPN;
        // priority of messages
        gEcuSettings.ariMsgPriority  = gEcuConfigPtr->ariPriority;
        gEcuSettings.accsMsgPriority = gEcuConfigPtr->accsPriority;
        gEcuSettings.ssi2MsgPriority = gEcuConfigPtr->ssi2Priority;

        if(!fromUart){
            BackFillUartDataStructure();
        }
    }

    config_ApplyEcuAddress((uint8_t)gEcuSettings.ecuAddress);
    config_ApplyEcuBaudrate((uint8_t)gEcuSettings.ecuBaudRate);

    size   = (uint16_t)sizeof(ecu_settings_struct);
    status = EEPROM_SaveEcuSettings((uint8_t *)&gEcuSettings, size);

    if(status == FALSE){
        if(!fDefault){
            gEcuSettings.ecuAddress   = gEcu.addr;
        }
    }
    
    EEPROM_SaveEcuAddress(gEcuSettings.ecuAddress);
    uint64_t config = PrepareDm1ConfigToEeprom();
    EEPROM_SaveDM1Config(config);

    return status;

}

/*******************************************
 * @brief 
 * 
 * @param ecuAddress ==
 * @return BOOL 
********************************************/
BOOL  SaveEcuAddress(uint8_t ecuAddress)
{
    gEcuSettings.ecuAddress = ecuAddress;
    config_ApplyEcuAddress((uint8_t)gEcuSettings.ecuAddress);
    return EEPROM_SaveEcuAddress(ecuAddress);
}

/*******************************************
 * @brief 
 * 
 * @param baudrate ==
********************************************/
extern  void UpdateEcuBaudrate(uint16_t baudrate)
{
    gEcuSettings.ecuBaudRate = baudrate;
    config_ApplyEcuBaudrate((uint8_t)gEcuSettings.ecuBaudRate);
}


 /******************************************************************************
 * @brief
 * @return address 
 ******************************************************************************/
uint8_t GetEcuAddress()
{
    return (uint8_t)gEcuSettings.ecuAddress;
}

/**********************************************
* @brief Set the Ecu Baudrate object
* 
* @param baudrate -- 
***********************************************/
void SetEcuBaudrate(uint16_t const baudrate)
{
    newCanBaudRate = baudrate;
}


/**********************************************
* @brief Set the Ecu Address object
* 
* @param address --- 
***********************************************/
void SetEcuAddress(uint16_t const address)
{
    newAddress = address;
}

/*******************************************
 * @brief Set the Ecu Behavior object
 * 
 * @param behavior ==
********************************************/
void SetEcuBehavior(uint16_t const behavior)
{
    newBehavior = behavior;
}

/*******************************************
 * @brief Set the Ecu Packet Type object
 * 
 * @param type ==
********************************************/
void SetEcuPacketType(uint16_t const type)
{
    newEcuPacketType = type;
}

/*******************************************
 * @brief Set the Ecu Packet Rate object
 * 
 * @param rate ==
********************************************/
BOOL    SetEcuPacketRate(uint16_t const rate)
{
    if(rate){
        if((rate > 100) || ((100 % rate) != 0)){    // should be even to 100
            return FALSE;
        }
    }
    newEcuPacketRate = rate;
    return TRUE;
}


/*******************************************
 * @brief Set the Ecu Orientation object
 * 
 * @param orient ==
********************************************/
void SetEcuOrientation(uint16_t const orient)
{
    newOrientation = orient;
}

/*******************************************
 * @brief Set the User Uart Baud Rate object
 * 
 * @param baudRate ==
********************************************/
void SetUserUartBaudRate(uint16_t const baudRate)
{
    newUartBaudRate = baudRate;
}


/*******************************************
 * @brief Set the Ecu Accel Filter object
 * 
 * @param filter ==
********************************************/
void SetEcuAccelFilter(uint16_t const filter)
{
    newAccelFilter = filter;
}

/*******************************************
 * @brief Set the Ecu Rate Filter object
 * 
 * @param filter ==
********************************************/
void SetEcuRateFilter(uint16_t const filter)
{
    newRateFilter = filter;
}


/*******************************************
 * @brief 
 * 
********************************************/
void UpdateEcuSettings()
{
    BOOL fSave = FALSE;
    BOOL fBaudRate = FALSE;
    BOOL fAddress  = FALSE;
    BOOL fBehavior = FALSE;
    BOOL fAccelLpf = FALSE;
    BOOL fRateLpf = FALSE;
    BOOL fOrientation = FALSE;
    BOOL fPacketRate  = FALSE;
    BOOL fPacketType   = FALSE;
    BOOL fUartBaudRate = FALSE;

    if(newCanBaudRate != 0xFFFFU){
        gEcuSettings.ecuBaudRate = newCanBaudRate & 0x03U;
        newCanBaudRate = 0xFFFFU;
        fBaudRate = TRUE;
        fSave   = TRUE; 
    }
    
    if(newUartBaudRate != 0xFFFFU){
        gEcuSettings.uartCfg.userUartBaudRate = newUartBaudRate;
        newUartBaudRate = 0xFFFFU;
        fUartBaudRate   = TRUE;
        fSave           = TRUE; 
    }

    if(newAddress != 0xFFFFU){
        gEcuSettings.ecuAddress  = newAddress & 0xFFU;
        fAddress  = TRUE;
        newAddress = 0xFFFFU;
        fSave = TRUE;
    }
    
    if(newBehavior != 0xFFFFU){
        gEcuSettings.userBehavior = newBehavior;
        fBehavior = TRUE;
        newBehavior = 0xFFFF;
        fSave = TRUE;
    }

    if(newAccelFilter != 0xFFFFU){
        gEcuSettings.ecuFilterFreqAccel = newAccelFilter;
        gEcuSettings.uartCfg.lpfAccelFilterFreq = 0xFF;   // invalidate 
        fAccelLpf = TRUE;
        newAccelFilter = 0xFFFF;
        fSave = TRUE;
    }

    if(newRateFilter != 0xFFFFU){
        gEcuSettings.ecuFilterFreqRate = newRateFilter;
        gEcuSettings.uartCfg.lpfRateFilterFreq = 0xFF;   // invalidate 
        fRateLpf = TRUE;
        newRateFilter = 0xFFFF;
        fSave = TRUE;
    }

    if(newOrientation != 0xFFFFU){
        gEcuSettings.ecuOrientation = newOrientation;
        gEcuSettings.uartCfg.orientation[0] = 0;   // invalidate 
        fOrientation = TRUE;
        newOrientation = 0xFFFF;
        fSave = TRUE;
    }

    if(newEcuPacketRate != 0xFFFFU){
        gEcuSettings.ecuPacketRate = newEcuPacketRate;
        fPacketRate  = TRUE;
        newEcuPacketRate   = 0xFFFF;
        fSave = TRUE;
    }

    if(newEcuPacketType != 0xFFFFU){
        gEcuSettings.ecuPacketType = newEcuPacketType;
        fPacketType  = TRUE;
        newEcuPacketType = 0xFFFF;
        fSave = TRUE;
    }

    if (fSave)
    {

        HW_FeedWatchdog();
        uint32_t const size = sizeof(ecu_settings_struct);
        BOOL const status = EEPROM_SaveEcuSettings((uint8_t *)&gEcuSettings, size);

        if (fAddress)
        {
            config_ApplyEcuAddress((uint8_t)gEcuSettings.ecuAddress);
            EEPROM_SaveEcuAddress(gEcuSettings.ecuAddress);
        }
        if (fBaudRate)
        {
            config_ApplyEcuBaudrate((uint8_t)gEcuSettings.ecuBaudRate);
        }
        if (fBehavior)
        {
            config_ApplyEcuUnitBehavior(gEcuSettings.userBehavior);
        }
        if (fAccelLpf)
        {
            config_SelectUserLPFilter(ACCEL_SENSOR, gEcuSettings.ecuFilterFreqAccel, TRUE);
        }
        if (fRateLpf)
        {
            config_SelectUserLPFilter(RATE_SENSOR, gEcuSettings.ecuFilterFreqRate, TRUE);
        }
        if (fOrientation)
        {
            config_ApplyCanOrientation(gEcuSettings.ecuOrientation);
        }
        if (fPacketRate)
        {
            config_ApplyCanPacketRate(gEcuSettings.ecuPacketRate);
        }
        if (fPacketType)
        {
            config_ApplyCanPacketType(gEcuSettings.ecuPacketType);
        }
        if (fUartBaudRate)
        {
            config_SetBaudRate(gEcuSettings.uartCfg.userUartBaudRate, TRUE);
        }


        if (!status)
        {
            ConfigSaveError++;
        }
    }
}



/**********************************************
* @brief Get the Ecu Baud Rate object
* 
* @return int32_t 
***********************************************/
int32_t GetEcuBaudRate()
{
    return gEcuSettings.ecuBaudRate;
}

/******************************************************************************
 * @brief
 *  
 ******************************************************************************/
void  ApplyEcuSettings()
{
    EcuConfigurationStruct* const pEcuConfig =  gEcuConfigPtr;
    
    // Add/Remove/Verify ECU-specific parameters here
    pEcuConfig->packet_rate_div   = gEcuSettings.ecuPacketRate == 0? 0 : 100U/gEcuSettings.ecuPacketRate;   //++
    pEcuConfig->accel_cut_off     = gEcuSettings.ecuFilterFreqAccel;
    pEcuConfig->rate_cut_off      = gEcuSettings.ecuFilterFreqRate;
    pEcuConfig->packet_type       = gEcuSettings.ecuPacketType;       //++
    pEcuConfig->orien_bits        = gEcuSettings.ecuOrientation;
    pEcuConfig->user_behavior     = gEcuSettings.userBehavior;
    pEcuConfig->limitAccelSwitchDelay = gEcuSettings.algoLinAccelSwitchDelay;
    pEcuConfig->limitRateIntegrationTime = gEcuSettings.algoRateIntegrationTime;
    pEcuConfig->coefOfReduceQ            = gEcuSettings.algoCoefOfReduceQ;
    
    pEcuConfig->alg_reset_ps      = gEcuSettings.algResetPs;          //++
    pEcuConfig->save_cfg_ps       = gEcuSettings.saveCfgPs;           //++
    pEcuConfig->packet_type_ps    = gEcuSettings.packetTypePs;        //++
    pEcuConfig->packet_rate_ps    = gEcuSettings.packetRatePs;        //++
    pEcuConfig->digital_filter_ps = gEcuSettings.filterPs;            //++
    pEcuConfig->orientation_ps    = gEcuSettings.orientationPs;       //++
    pEcuConfig->user_behavior_ps  = gEcuSettings.userBehvPs;          //++
    pEcuConfig->master_status_ps  = gEcuSettings.masterStatusPs;  
    pEcuConfig->hw_status_ps      = gEcuSettings.hwStatusPs;  
    pEcuConfig->sw_status_ps      = gEcuSettings.swStatusPs;  
    pEcuConfig->hr_accel_ps       = gEcuSettings.hrAccelPs;  
    pEcuConfig->hr_rate_ps        = gEcuSettings.hrRatePs;  
    pEcuConfig->algo_control_ps   = gEcuSettings.algoCtrlPs;  
    pEcuConfig->ecuBaudrate       = gEcuSettings.ecuBaudRate;
    pEcuConfig->ecuAddress        = gEcuSettings.ecuAddress;

    // Odometer lever arm
    pEcuConfig->odoLeverArmX      = gEcuSettings.odoLeverArmX;
    pEcuConfig->odoLeverArmY      = gEcuSettings.odoLeverArmY;
    pEcuConfig->odoLeverArmZ      = gEcuSettings.odoLeverArmZ;
    // Aiding signal configuration settings
    pEcuConfig->signalSource      = gEcuSettings.signalSource;
    pEcuConfig->aidingPF          = gEcuSettings.aidingPF;
    pEcuConfig->aidingPS          = gEcuSettings.aidingPS;
    pEcuConfig->aidingMsgRate     = gEcuSettings.aidingMsgRate;
    pEcuConfig->drivingDirPF      = gEcuSettings.drivingDirPF;
    pEcuConfig->drivingDirPS      = gEcuSettings.drivingDirPS;
    pEcuConfig->odoCfgSwitch      = gEcuSettings.odoCfgSwitch;
    // new configurable ps
    pEcuConfig->aid_lvarm_ps      = gEcuSettings.aidLvarmPs;
    pEcuConfig->aid_config_ps     = gEcuSettings.aidConfigPs;
    pEcuConfig->dm1_config_ps     = gEcuSettings.dm1ConfigPs;
    // DM1 config parameters
    pEcuConfig->dm1_config_ps     = gEcuSettings.dm1ConfigPs;
    pEcuConfig->lamp_status       = gEcuSettings.lamp_status;
    pEcuConfig->flash_status      = gEcuSettings.flash_status;
    pEcuConfig->FMI1              = gEcuSettings.FMI1;
    pEcuConfig->FMI2              = gEcuSettings.FMI2;
    pEcuConfig->SPN               = gEcuSettings.SPN;
   // priority of messages
    pEcuConfig->ariPriority       = gEcuSettings.ariMsgPriority;
    pEcuConfig->accsPriority      = gEcuSettings.accsMsgPriority; 
    pEcuConfig->ssi2Priority      = gEcuSettings.ssi2MsgPriority;

    // Propagate parameters to common configuration area

    config_ApplyEcuAddress(gEcuSettings.ecuAddress);
    config_ApplyEcuBaudrate(gEcuSettings.ecuBaudRate);
    config_ApplyEcuUnitBehavior(gEcuSettings.userBehavior);
    config_ApplyCanPacketRate(gEcuSettings.ecuPacketRate);
    config_ApplyCanPacketType(gEcuSettings.ecuPacketType);
    config_SelectUserLPFilter(ACCEL_SENSOR, gEcuSettings.ecuFilterFreqAccel, TRUE);
    config_SelectUserLPFilter(RATE_SENSOR, gEcuSettings.ecuFilterFreqRate, TRUE);
    config_ApplyCanOrientation(gEcuSettings.ecuOrientation);
    OdoUpdateConfig(gEcuConfigPtr->aidingMsgRate, gEcuConfigPtr->signalSource ,gEcuConfigPtr->odoCfgSwitch);
    OdoUpdateLeverArmConfig(&(gEcuConfigPtr->odoLeverArmX));
    ecu_set_address(gEcuSettings.ecuAddress);
    ecu_set_baudrate(gEcuSettings.ecuBaudRate);

}

 /******************************************************************************
 * @brief
 * @return detect enabled 
 ******************************************************************************/
BOOL CanBaudRateDetectionEnabled()
{
    if((gEcuConfigPtr->user_behavior & (uint16_t)USER_BEHAVIOR_ENABLE_AUTO_BAUD) != 0U){
        return TRUE;
    }
    return FALSE;

}

 /******************************************************************************
 * @brief
 * @return use algo 
 ******************************************************************************/
BOOL UseAlgorithm()
{
   if((gEcuConfigPtr->user_behavior & (uint16_t)USER_BEHAVIOR_RUN_ALGORITHM_MASK) != 0U){
       return TRUE;
   };  

   return FALSE;
}

 /******************************************************************************
 * @brief
 * @return send rates 
 ******************************************************************************/
BOOL SendRawRates()
{
    if((gEcuConfigPtr->user_behavior & (uint16_t)USER_BEHAVIOR_SEND_RAW_RATES) != 0U){  
        return TRUE;
    }
   return FALSE;

}

 /******************************************************************************
 * @brief
 * @return swap pich 
 ******************************************************************************/
BOOL SwapPitchAndRoll()
{
    if((gEcuConfigPtr->user_behavior & (uint16_t)USER_BEHAVIOR_SWAP_PITCH_AND_ROLL) != 0U){
        return TRUE;
    }
   return FALSE;
}

 /******************************************************************************
 * @brief
 * @return swap Frame 
 ******************************************************************************/
BOOL SwapAccelFrame()
{
    if ((gEcuConfigPtr->user_behavior & (uint16_t)USER_BEHAVIOR_SWAP_ACCEL_FRAME) != 0U) { 
        return TRUE;
    }
    
    return FALSE;
}

 /******************************************************************************
 * @brief
 * @return swap bytes 
 ******************************************************************************/
BOOL SwapBytesInRequest()
{
    if ((gEcuConfigPtr->user_behavior & (uint16_t)USER_BEHAVIOR_SWAP_BYTES_IN_REQUEST) != 0U) { 
        return TRUE;
    }
    
    return FALSE;
}


/******************************************************************************
 * @brief
 * @param pConfig [in] 
 *  
 ******************************************************************************/
static void  ApplySystemParameters(EcuConfigurationStruct* const pConfig)
{
    setStat += (uint32_t)config_SelectUserLPFilter(RATE_SENSOR,  pConfig->rate_cut_off, TRUE);
    setStat += (uint32_t)config_SelectUserLPFilter(ACCEL_SENSOR, pConfig->accel_cut_off, TRUE);
    setStat += (uint32_t)config_ApplyOrientation(pConfig->orien_bits, TRUE);
}


/*******************************************
 * @brief 
 * 
********************************************/
static void UpdateEcuInstanceSettings()
{
    ecu_set_address(config_GetEcuAddress());
    ecu_set_baudrate(config_GetEcuBaudRate());
}

 /******************************************************************************
 * @brief
 * @return divider 
 ******************************************************************************/
int32_t GetCANPacketRateDivider()
{
    return gEcuConfigPtr->packet_rate_div;
}

 /******************************************************************************
 * @brief
 * @return detectMode 
 ******************************************************************************/
BOOL  GetAlgorithmLinAccelDetectMode()
{
    if((gEcuConfig.user_behavior & (uint16_t)USER_BEHAVIOR_USE_RAW_ACCEL_FOR_ALG) != 0U){
        return TRUE;
    }
    
    return FALSE;

}

 /******************************************************************************
 * @brief
 * @return predictMode 
 ******************************************************************************/
BOOL  GetAlgorithmAccelPredictMode()
{
    if((gEcuConfig.user_behavior & (uint16_t)USER_BEHAVIOR_USE_RAW_RATE_TO_PREDICT_ACCEL) != 0U){
        return TRUE;
    }
    
    return FALSE;

}

 /******************************************************************************
 * @brief
 * @return time 
 ******************************************************************************/
float32_t  GetAlgorithmCoefOfReduceQ()
{
    // 0.0001 to 1 (1 to  10000)
    return (float32_t)gEcuConfig.coefOfReduceQ/10000.0F;
}

 /******************************************************************************
 * @brief
 * @return time 
 ******************************************************************************/
float32_t  GetAlgorithmAccelSwitchDelay()
{
    // 0.01 to 10 (100 to 10000)
    return (float32_t)gEcuConfig.limitAccelSwitchDelay/1000.0F;
}     

 /******************************************************************************
 * @brief
 * @return time 
 ******************************************************************************/
float32_t  GetAlgorithmRateIntegrationTime()      
{
    // 0.01 to 10 (100 to 10000)
    return (float32_t)gEcuConfig.limitRateIntegrationTime/1000.0F;
}     


