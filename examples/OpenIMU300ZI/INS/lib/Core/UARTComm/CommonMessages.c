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
#include "platformAPI.h"
#include "sensorsAPI.h"
#include "appVersion.h"
#include "ucb_packet_struct.h"
#include "magAPI.h"
#include "MagAlign.h"

#include "CommonMessages.h"
#include "algorithm.h"

#include "EKF_Algorithm.h"


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
    uint8_t *model           = (uint8_t*)unitVersionString();
    uint8_t *rev             = (uint8_t*)platformBuildInfo();
    unsigned int serialNum   = unitSerialNumber();
    len = snprintf((char*)payload, 250, "%s %s SN:%u", model, rev, serialNum );
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
   int len = snprintf((char*)payload, 250, "%s", APP_VERSION_STRING );
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
    double mags[NUM_AXIS];
    double rates[NUM_AXIS];
    
    data1_payload_t *pld = (data1_payload_t *)payload;  

//  tstamp = platformGetDacqTimeStamp();          // time stamp of last sensor sample in microseconds from system start
    tstamp = platformGetCurrTimeStamp();            // current time stamp in microseconds from system start
//  tstamp /= 1000;                                 // convert to miliseconds 
//  timer  = getSystemTime();                       // OS timer value (tick defined in FreeRTOSConfig.h)
    pld->timer = tstamp;
    GetAccelData_mPerSecSq(accels);
    GetRateData_degPerSec(rates);
    GetMagData_G(mags);

    for (int i = 0; i < NUM_AXIS; i++){
        pld->accel_mpss[i] = (float)accels[i];
        pld->rate_dps[i] = (float)rates[i];
        pld->mag_G[i] = (float)mags[i];
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
    GetAccelData_mPerSecSq(accels);
    EKF_GetOperationalMode(&OperMode);
    EKF_GetOperationalSwitches(&LinAccelSwitch, &TurnSwitch);

    pld->itow           = getAlgorithmITOW();
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
    GetAccelData_mPerSecSq(accels);

    pld->itow           = getAlgorithmITOW();
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

    GetAccelData_mPerSecSq(accels);
    GetRateData_degPerSec(rates);
    GetMagData_G(mags);
    GetBoardTempData(&temp);

    pld->tstmp   = platformGetIMUCounter();
    pld->dbTstmp = (double)pld->tstmp * 1.0e-3; // seconds

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
    pld->tstmp   = platformGetIMUCounter();
    pld->dbTstmp = platformGetSolutionTstampAsDouble() * 0.000001;  // seconds

    EKF_GetAttitude_EA(EulerAngles);
    pld->roll    = (float)EulerAngles[ROLL];
    pld->pitch   = (float)EulerAngles[PITCH];
    pld->yaw     = (float)EulerAngles[YAW];

    GetAccelData_g(data);
    for(int i = 0; i < NUM_AXIS; i++){
        pld->accels_g[i] = (float)data[i];
    }

    GetRateData_degPerSec(data);
    for(int i = 0; i < NUM_AXIS; i++){
        pld->rates_dps[i] = data[i];
    }

    GetMagData_G(data);
    for(int i = 0; i < NUM_AXIS; i++){
        pld->mags[i] = data[i];
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
    pld->tstmp   = platformGetIMUCounter();
    pld->dbTstmp = platformGetSolutionTstampAsDouble() * 0.000001;  // seconds

    EKF_GetAttitude_EA(rData);
    pld->roll  = (float)rData[ROLL];
    pld->pitch = (float)rData[PITCH];
    pld->yaw   = (float)rData[YAW];

    GetAccelData_g(dData);
    for(int i = 0; i < NUM_AXIS; i++){
        pld->accels_g[i] = (float)dData[i]; 
    }

    EKF_GetEstimatedAccelBias(fData);
    for(int i = 0; i < NUM_AXIS; i++){
        pld->accelBias[i] = fData[i]; 
    }

    GetRateData_degPerSec(dData);
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

    GetMagData_G(dData);
    for(int i = 0; i < NUM_AXIS; i++){
        pld->mags[i] = (float)dData[i]; 
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

BOOL    Fill_MagAlignResponsePayload(int8_t state, UcbPacketStruct *ptrUcbPacket)
{
    int8_t estimatedMagAlignVals[8] = {0};
    int8_t magAlignVals[8] = {0};
    BOOL valid = TRUE;

    switch (state - 1)
    {
        //
        uint8_t len;

    case 0: // Return the Mag-Align status
        //uint8_t *model = (uint8_t*)unitVersionString();
        //uint8_t *rev   = (uint8_t*)platformBuildInfo();
        //unsigned int serialNum = unitSerialNumber();
        //len = snprintf((char*)ptrUcbPacket->payload, 250, "%s %s SN:%u", model, rev, serialNum );
        //ptrUcbPacket->payloadLength = len;
        if (gMagAlign.state == MAG_ALIGN_STATUS_START_CAL_WITH_AUTOEND)
        {
            // Start (auto end)
            len = snprintf((char *)ptrUcbPacket->payload, 250, "%c", (char)0x1);
        }
        else if (gMagAlign.state == MAG_ALIGN_STATUS_START_CAL_WITHOUT_AUTOEND)
        {
            // Start (manual end)
            len = snprintf((char *)ptrUcbPacket->payload, 250, "%c", (char)0x2);
        }
        else
        {
            // Start (manual end)
            len = snprintf((char *)ptrUcbPacket->payload, 250, "%c", (char)0x0);
        }

        ptrUcbPacket->payloadLength = len;
        break;

    case 1: // Start mag-align w/ autoend
    case 2: // Start mag-align w/ autoend
    case 3: // Stop mag-align w/ autoend
    case 4: // Accept results
    case 5: // Accept results and write to EEPROM
    case 6: // Abort Mag-Align or reject results
    case 8: // Restore default mag-align values
    case 9: // Restore default mag-align values and save in EEPROM
        len = snprintf((char *)ptrUcbPacket->payload, 250, "%c", (char)state - 1);
        ptrUcbPacket->payloadLength = len;
        break;

    case 7: // Return stored mag-align values
#if 0
                    // Test values:
                    gMagAlign.estParams.hardIronBias[X_AXIS] =  0.1;
                    gMagAlign.estParams.hardIronBias[Y_AXIS] = -0.2;
                    gMagAlign.estParams.softIronScaleRatio   = 0.98;
                    gMagAlign.estParams.softIronAngle        = -270.0 * DEG_TO_RAD;
#endif

        // Bias can be +/- 8.0 [g] (full scale of sensor)
        //   SF = 2^15 / maxVal = 2^15 / 8.0 = 4096
        magAlignVals[0] = (char)(((int16_t)(gMagAlign.hardIronBias[X_AXIS] * (float)4096.0) >> 8) & 0xFF);
        magAlignVals[1] = (char)(((int16_t)(gMagAlign.hardIronBias[X_AXIS] * (float)4096.0) >> 0) & 0xFF);
        magAlignVals[2] = (char)(((int16_t)(gMagAlign.hardIronBias[Y_AXIS] * (float)4096.0) >> 8) & 0xFF);
        magAlignVals[3] = (char)(((int16_t)(gMagAlign.hardIronBias[Y_AXIS] * (float)4096.0) >> 0) & 0xFF);

        // Ratio can be 0 --> 1
        //   SF = (2^16-1) / maxVal = (2^16-1) / 1.0 = 65535
        magAlignVals[4] = (char)(((int16_t)(gMagAlign.softIronScaleRatio * (float)65535.0) >> 8) & 0xFF);
        magAlignVals[5] = (char)(((int16_t)(gMagAlign.softIronScaleRatio * (float)65535.0) >> 0) & 0xFF);

        //   SF = 2^15 / maxVal = 2^15 / pi = 10430.37835047045
        magAlignVals[6] = (char)(((int16_t)(gMagAlign.softIronAngle * (float)10430.37835047046) >> 8) & 0xFF);
        magAlignVals[7] = (char)(((int16_t)(gMagAlign.softIronAngle * (float)10430.37835047046) >> 0) & 0xFF);

        // Bias can be +/- 8.0 [g] (full scale of sensor)
        //   SF = 2^15 / maxVal = 2^15 / 8.0 = 4096
        estimatedMagAlignVals[0] = (char)(((int16_t)(gMagAlign.estParams.hardIronBias[X_AXIS] * (float)4096.0) >> 8) & 0xFF);
        estimatedMagAlignVals[1] = (char)(((int16_t)(gMagAlign.estParams.hardIronBias[X_AXIS] * (float)4096.0) >> 0) & 0xFF);
        estimatedMagAlignVals[2] = (char)(((int16_t)(gMagAlign.estParams.hardIronBias[Y_AXIS] * (float)4096.0) >> 8) & 0xFF);
        estimatedMagAlignVals[3] = (char)(((int16_t)(gMagAlign.estParams.hardIronBias[Y_AXIS] * (float)4096.0) >> 0) & 0xFF);

        // Ratio can be 0 --> 1
        //   SF = (2^16-1) / maxVal = (2^16-1) / 1.0 = 65535
        estimatedMagAlignVals[4] = (char)(((int16_t)(gMagAlign.estParams.softIronScaleRatio * (float)65535.0) >> 8) & 0xFF);
        estimatedMagAlignVals[5] = (char)(((int16_t)(gMagAlign.estParams.softIronScaleRatio * (float)65535.0) >> 0) & 0xFF);

        // Angle can be +/- pi (in radians)
        //   Correct for angles that exceed +/-180
        if (gMagAlign.estParams.softIronAngle > PI)
        {
            gMagAlign.estParams.softIronAngle = (float)PI - gMagAlign.estParams.softIronAngle;
        }
        else if (gMagAlign.estParams.softIronAngle < -PI)
        {
            gMagAlign.estParams.softIronAngle = TWO_PI + gMagAlign.estParams.softIronAngle;
        }

        //   SF = 2^15 / maxVal = 2^15 / pi = 10430.37835047045
        estimatedMagAlignVals[6] = (char)(((int16_t)(gMagAlign.estParams.softIronAngle * (float)10430.37835047046) >> 8) & 0xFF);
        estimatedMagAlignVals[7] = (char)(((int16_t)(gMagAlign.estParams.softIronAngle * (float)10430.37835047046) >> 0) & 0xFF);

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

        len = snprintf((char *)ptrUcbPacket->payload, 250, "%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c", (char)magAlignVals[0],
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
                       (char)estimatedMagAlignVals[7]);
        
        ptrUcbPacket->payloadLength = len;
        break;

    case 10: // Load user computed mag-align values
        break;

    default:
        valid = FALSE;
        break;
    }
    return valid;
}
