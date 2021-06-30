/** ***************************************************************************
 * @file   UARTMessages.c
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
#include "calibrationAPI.h"
#include "sensorsAPI.h"
#include "appVersion.h"

#include "CommonMessages.h"
#include "indices.h"

#include "ekfAPI.h"
#include "halAPI.h"
#include "partNumber.h"
#include "aceinna_sae_j1939.h"

extern BOOL odoUpdate;
/******************************************************************************
 * @name  FillPingPacketPayload - API call ro prepare user output packet
 * @brief general handler
 * @param [in] payload pointer to put user data to
 * @param [in/out] number of bytes in user payload
 * @retval N/A
 ******************************************************************************/
BOOL Fill_PingPacketPayload(uint8_t *payload, uint8_t *payloadLen)
{
    int len; 
    uint8_t *model           = cal_GetUnitVersion();
    unsigned int serialNum   = cal_GetUnitSerialNum();
    len = snprintf((char*)payload, 250, "%s %s %s SN:%u", model, PART_NUMBER_STRING, APP_VERSION_STRING, serialNum );
    *payloadLen = len;
    return TRUE;
}

/******************************************************************************
 * @name FillVersionPacketPayload - API call ro prepare user output packet
 * @brief general handler
 * @param [in] payload pointer to put user data to
 * @param [in/out] number of bytes in user payload
 * @retval N/A
 ******************************************************************************/
BOOL Fill_VersionPacketPayload(uint8_t *payload, uint8_t *payloadLen)
{
   int len = snprintf((char*)payload, 250, "%s %s", APP_NAME_STRING, APP_VERSION_STRING);
   *payloadLen = len;
    return TRUE;
}

/******************************************************************************
 * @name FillTestPacketPayload - API call ro prepare user output packet
 * @brief general handler
 * @param [in] payload pointer to put user data to
 * @param [in/out] number of bytes in user payload
 * @retval N/A
 ******************************************************************************/
BOOL Fill_zTPacketPayload(uint8_t *payload, uint8_t *payloadLen)
{
    static uint32_t _testVal = 0;
    test_payload_t* pld = (test_payload_t*)payload;
    pld->counter = _testVal++;
    *payloadLen  = sizeof(test_payload_t);
    return TRUE;
}

/******************************************************************************
 * @name FillTestPacketPayloadt - API call ro prepare user output packet
 * @brief general handler
 * @param [in] payload pointer to put user data to
 * @param [in/out] number of bytes in user payload
 * @retval N/A
 ******************************************************************************/
BOOL Fill_z1PacketPayload(uint8_t *payload, uint8_t *payloadLen)
{
    uint64_t tstamp;
    double accels[NUM_AXIS];
    double rates[NUM_AXIS];
    
    data1_payload_t *pld = (data1_payload_t *)payload;  

//  tstamp = platformGetDacqTimeStamp();          // time stamp of last sensor sample in microseconds from system start
    tstamp = TIMER_GetCurrTimeStamp();            // current time stamp in microseconds from system start
//  tstamp /= 1000;                                 // convert to miliseconds 
//  timer  = getSystemTime();                       // OS timer value (tick defined in FreeRTOSConfig.h)
    pld->timer = tstamp;
    sens_GetAccelData_mPerSecSq(accels);
    sens_GetRateData_degPerSec(rates);

    for (int i = 0; i < NUM_AXIS; i++){
        pld->accel_mpss[i] = (float)accels[i];
        pld->rate_dps[i] = (float)rates[i];
        pld->mag_G[i] = 0.0F;
    }

    *payloadLen = sizeof(data1_payload_t);
    return TRUE;
}


/******************************************************************************
 * @name FillTestPacketPayloadt - API call ro prepare user output packet
 * @brief general handler
 * @param [in] payload pointer to put user data to
 * @param [in/out] number of bytes in user payload
 * @retval N/A
 ******************************************************************************/
BOOL Fill_a1PacketPayload(uint8_t *payload, uint8_t *payloadLen)
{           
    angle1_payload_t* pld = (angle1_payload_t*)payload;

    // Variables used to hold the EKF values
    real EulerAngles[NUM_AXIS];
    real CorrRates_B[NUM_AXIS];
    double accels[NUM_AXIS];
    // Diagnostic flags
    uint8_t OperMode, LinAccelSwitch, TurnSwitch;

    EKF_GetAttitude_EA(EulerAngles);
    EKF_GetCorrectedAngRates(CorrRates_B);
    sens_GetAccelData_mPerSecSq(accels);
    EKF_GetOperationalMode(&OperMode);
    EKF_GetOperationalSwitches(&LinAccelSwitch, &TurnSwitch);

    pld->itow           = TIMER_GetCurrTimeStamp()/1000;
    pld->dblItow        = 1.0e-3 * pld->itow;
    pld->roll           = (float)EulerAngles[ROLL];
    pld->pitch          = (float)EulerAngles[PITCH];
    pld->ekfOpMode      = OperMode;
    pld->accelLinSwitch = LinAccelSwitch;
    pld->turnSwitch     = TurnSwitch;

    for(int i = 0; i < NUM_AXIS; i++){
        pld->corrRates[i] = (float)CorrRates_B[i];
        pld->accels[i]    = (float)accels[i];
    }

    *payloadLen  = sizeof(angle1_payload_t);
    
    return TRUE;
}

/******************************************************************************
 * @name FillTestPacketPayloadt - API call ro prepare user output packet
 * @brief general handler
 * @param [in] payload pointer to put user data to
 * @param [in/out] number of bytes in user payload
 * @retval N/A
 ******************************************************************************/
BOOL Fill_a2PacketPayload(uint8_t *payload, uint8_t *payloadLen)
{           
    angle2_payload_t* pld = (angle2_payload_t*)payload;

    // Variables used to hold the EKF values
    real EulerAngles[NUM_AXIS];
    real CorrRates_B[NUM_AXIS];
    double accels[NUM_AXIS];

    EKF_GetAttitude_EA(EulerAngles);
    EKF_GetCorrectedAngRates(CorrRates_B);
    sens_GetAccelData_mPerSecSq(accels);

    pld->itow           = TIMER_GetCurrTimeStamp()/1000;
    pld->dblItow        = 1.0e-3 * pld->itow;
    pld->roll           = (float)EulerAngles[ROLL];
    pld->pitch          = (float)EulerAngles[PITCH];
    pld->yaw            = (float)EulerAngles[YAW];

    for(int i = 0; i < NUM_AXIS; i++){
        pld->corrRates[i] = (float)CorrRates_B[i];
        pld->accels[i]    = (float)accels[i];
    }

    *payloadLen  = sizeof(angle2_payload_t);
    
    return TRUE;
}

/******************************************************************************
 * @name Fills1PacketPayloadt - API call ro prepare user output packet
 * @brief general handler
 * @param [in] payload pointer to put user data to
 * @param [in/out] number of bytes in user payload
 * @retval N/A
 ******************************************************************************/
BOOL Fill_s1PacketPayload(uint8_t *payload, uint8_t *payloadLen)
{           
    double accels[NUM_AXIS];
    double mags[NUM_AXIS];
    double rates[NUM_AXIS];
    double temp;
    scaled1_payload_t *pld = (scaled1_payload_t *)payload;  
    *payloadLen = sizeof(scaled1_payload_t);

    sens_GetAccelData_mPerSecSq(accels);
    sens_GetRateData_degPerSec(rates);
    sens_GetMagData_G(mags);
    sens_GetBoardTempData(&temp);

    pld->tstmp   = TIMER_GetCurrTimeStamp()/1000;   // miliseconds
    pld->dbTstmp = (double)pld->tstmp/1000;         // seconds

    for (int i = 0; i < NUM_AXIS; i++){
        pld->accel_g[i]  = (float)accels[i];
        pld->rate_dps[i] = (float)rates[i];
        pld->mag_G[i]    = (float)mags[i];
    }
    
    pld->temp_C = (float)temp;

    return TRUE;
}

/******************************************************************************
 * @name Fille1PacketPayloadt - API call ro prepare user output packet - EKF data 1
 * @brief general handler
 * @param [in] payload pointer to put user data to
 * @param [in/out] number of bytes in user payload
 * @retval N/A
 ******************************************************************************/
BOOL Fill_e1PacketPayload(uint8_t *payload, uint8_t *payloadLen)
{           
    // Variables used to hold the EKF values
    uint8_t opMode, linAccelSw, turnSw;
    double data[NUM_AXIS];
    real EulerAngles[NUM_AXIS];
    
    ekf1_payload_t *pld = (ekf1_payload_t *)payload;  

    *payloadLen  = sizeof(ekf1_payload_t);
    pld->tstmp   = TIMER_GetCurrTimeStamp()/1000;             // milliseconds
    pld->dbTstmp = (double)TIMER_GetCurrTimeStamp()/1000000;  // seconds

    EKF_GetAttitude_EA(EulerAngles);
    pld->roll    = (float)EulerAngles[ROLL];
    pld->pitch   = (float)EulerAngles[PITCH];
    pld->yaw     = (float)EulerAngles[YAW];

    sens_GetAccelData_g(data);
    for(int i = 0; i < NUM_AXIS; i++){
        pld->accels_g[i] = (float)data[i];
    }

    sens_GetRateData_degPerSec(data);
    for(int i = 0; i < NUM_AXIS; i++){
        pld->rates_dps[i] = data[i];
    }

    for(int i = 0; i < NUM_AXIS; i++){
        pld->mags[i] = 0.0F;
    }

    float rateBias[NUM_AXIS];
    EKF_GetEstimatedAngRateBias(rateBias);
    for(int i = 0; i < NUM_AXIS; i++){
        pld->rateBias[i] = (float)rateBias[i];
    }

    EKF_GetOperationalMode(&opMode);
    EKF_GetOperationalSwitches(&linAccelSw, &turnSw);
    pld->opMode         = opMode;
    pld->accelLinSwitch = linAccelSw;
    pld->turnSwitch     = turnSw;
    return TRUE;
}

/******************************************************************************
 * @name Fille2PacketPayloadt - API call ro prepare user output packet
 * @brief general handler
 * @param [in] payload pointer to put user data to
 * @param [in/out] number of bytes in user payload
 * @retval N/A
 ******************************************************************************/
BOOL Fill_e2PacketPayload(uint8_t *payload, uint8_t *payloadLen)
{           
    float  fData[3];
    real   rData[3];
    double dData[3];

    ekf2_payload_t *pld = (ekf2_payload_t *)payload;  
    
    *payloadLen  = sizeof(ekf2_payload_t);
    pld->tstmp   = TIMER_GetCurrTimeStamp()/1000;             // milliseconds
    pld->dbTstmp = (double)TIMER_GetCurrTimeStamp()/1000000;  // seconds

    EKF_GetAttitude_EA(rData);
    pld->roll  = (float)rData[ROLL];
    pld->pitch = (float)rData[PITCH];
    pld->yaw   = (float)rData[YAW];

    sens_GetAccelData_g(dData);
    for(int i = 0; i < NUM_AXIS; i++){
        pld->accels_g[i] = (float)dData[i]; 
    }

    EKF_GetEstimatedAccelBias(fData);
    for(int i = 0; i < NUM_AXIS; i++){
        pld->accelBias[i] = fData[i]; 
    }

    sens_GetRateData_degPerSec(dData);
    for(int i = 0; i < NUM_AXIS; i++){
        pld->rates_dps[i] = (float)dData[i]; 
    }

    EKF_GetEstimatedAngRateBias(fData);
    for(int i = 0; i < NUM_AXIS; i++){
        pld->rateBias[i] = fData[i]; 
    }


    EKF_GetEstimatedVelocity(fData);
    for(int i = 0; i < NUM_AXIS; i++){
        pld->velocity[i] = fData[i]; 
    }

    for(int i = 0; i < NUM_AXIS; i++){
        pld->mags[i] = 0.0F; 
    }

    EKF_GetEstimatedLLA(dData);
    for(int i = 0; i < NUM_AXIS; i++){
        pld->pos[i] = dData[i]; 
    }

    uint8_t opMode, linAccelSw, turnSw;
    EKF_GetOperationalMode(&opMode);
    EKF_GetOperationalSwitches(&linAccelSw, &turnSw);

    pld->opMode         = opMode;
    pld->accelLinSwitch = linAccelSw;
    pld->turnSwitch     = turnSw;

    return TRUE;
}

/******************************************************************************
 * @name Filld1PacketPayload - API call ro prepare user output packet - Aiding Signal Information 1
 * @brief general handler
 * @param [in] payload pointer to put user data to
 * @param [in/out] number of bytes in user payload
 * @retval N/A
 ******************************************************************************/
BOOL Fill_d1PacketPayload(uint8_t *payload, uint8_t *payloadLen)
{           
    // Variables used to hold the EKF values
    double data[NUM_AXIS];
    real EulerAngles[NUM_AXIS];
    
    aid1_payload_t *pld = (aid1_payload_t *)payload;  

    *payloadLen  = sizeof(aid1_payload_t);

    odoDataStruct_t* odo = GetOdometerPtr();
    pld->update = odoUpdate;
    odoUpdate = FALSE;
    pld->v = odo->v;

    EKF_GetAttitude_EA(EulerAngles);
    pld->roll    = (float)EulerAngles[ROLL];
    pld->pitch   = (float)EulerAngles[PITCH];

    sens_GetAccelData_g(data);
    for(int i = 0; i < NUM_AXIS; i++){
        pld->accels[i] = (float)data[i];
    }

    sens_GetRateData_degPerSec(data);
    for(int i = 0; i < NUM_AXIS; i++){
        pld->rates[i] = data[i];
    }

    pld->itow           = TIMER_GetCurrTimeStamp()/1000;
    pld->dblItow        = 1.0e-3 * pld->itow;
    return TRUE;
}

/******************************************************************************
 * @name Filld2PacketPayload - API call ro prepare user output packet - Aiding Signal Information 2
 * @brief general handler
 * @param [in] payload pointer to put user data to
 * @param [in/out] number of bytes in user payload
 * @retval N/A
 ******************************************************************************/
BOOL Fill_d2PacketPayload(uint8_t *payload, uint8_t *payloadLen)
{           
    // Variables used to hold the EKF values
    double data[NUM_AXIS];
    real EulerAngles[NUM_AXIS];
    
    aid2_payload_t *pld = (aid2_payload_t *)payload;  

    *payloadLen  = sizeof(aid2_payload_t);

    odoDataStruct_t* odo = GetOdometerPtr();
    pld->update = odoUpdate;
    odoUpdate = FALSE;
    memcpy(pld->vehAccel,odo->vehAccel,sizeof(real)*3);

    EKF_GetAttitude_EA(EulerAngles);
    pld->roll    = (float)EulerAngles[ROLL];
    pld->pitch   = (float)EulerAngles[PITCH];

    sens_GetAccelData_g(data);
    for(int i = 0; i < NUM_AXIS; i++){
        pld->accels[i] = (float)data[i];
    }

    sens_GetRateData_degPerSec(data);
    for(int i = 0; i < NUM_AXIS; i++){
        pld->rates[i] = data[i];
    }

    pld->itow           = TIMER_GetCurrTimeStamp()/1000;
    pld->dblItow        = 1.0e-3 * pld->itow;
    return TRUE;
}
extern EcuConfigurationStruct *gEcuConfigPtr;
/******************************************************************************
 * @name Filld3PacketPayload - API call ro prepare user output packet - Aiding Signal Information 3
 * @brief general handler
 * @param [in] payload pointer to put user data to
 * @param [in/out] number of bytes in user payload
 * @retval N/A
 ******************************************************************************/
BOOL Fill_d3PacketPayload(uint8_t *payload, uint8_t *payloadLen)
{           
    aid3_payload_t *pld = (aid3_payload_t *)payload;  

    *payloadLen  = sizeof(aid3_payload_t);

    OdoCfgStruct_t*odoCfg = GetOdometerCfgPtr();
    pld->aidingSigSrcType = odoCfg->signalSource;
    pld->aidingSigPGN = ((gEcuConfigPtr->aidingPF << 8) | (gEcuConfigPtr->aidingPS));
    pld->odoCfgSwitch = odoCfg->odoCfgSwitch;
    pld->msgRate = odoCfg->msgRate;
    pld->transSigPGN = ((gEcuConfigPtr->drivingDirPF << 8)|(gEcuConfigPtr->drivingDirPS));
    memcpy(pld->leverarmB, odoCfg->leverArmB, 3 * sizeof(real));
    return TRUE;
}