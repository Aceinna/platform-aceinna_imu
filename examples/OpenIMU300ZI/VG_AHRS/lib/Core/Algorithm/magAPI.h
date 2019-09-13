/** ***************************************************************************
 * @file magAPI.h API functions for Magnitometer functionality
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
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

#ifndef _MAG_API_H
#define _MAG_API_H

#include <stdint.h>
#include "GlobalConstants.h"

/** ***************************************************************************
 * @name InitMagAlignParams
 * @brief initialize the parameters for magnetic heading calculation
 * taskDataAcquisition.c and HardSoftIronCalibration()
 * Trace:
 * [SDD_INIT_EXT_MAG_CONFIG <-- SRC_INIT_MAGALIGN_RESULT]
 * @param N/A
 * @retval None
 ******************************************************************************/
void InitMagAlignParams(void);

/** ****************************************************************************
 * @name MagAlign API call to collect data for Hard/Soft Iron calibration
 * @brief called every frame in taskDataAcquisition.c
 *
 * Trace:
 *
 * @param N/A
 * @retval always returns 1
 ******************************************************************************/
uint8_t MagAlign( void );


/** ***************************************************************************
 * @name TaskDataAcquisition() CALLBACK main loop
 * @brief Get the sensor data at the specified frequency (based on the
 *        configuration of the accelerometer rate-sensor). Process and provide
 *        information to the user via the UART or SPI.
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void TaskWorldMagneticModel(void const *argument);


/*****************************************************************************
 * @name  SetMagAlignState 
 * @brief Sets current state of mag calibration
 * @param [in] state currend calibration state
 * @retval N/A
 ******************************************************************************/
void SetMagAlignState(int state);

/*****************************************************************************
 * @name  GetMagAlignState 
 * @brief Returns current state of mag calibration
 * @param [out ] state currend calibration state
 * @retval N/A
 ******************************************************************************/
int GetMagAlignState(void);

/*****************************************************************************
 * @name  GetMagAlignEstimatedParams 
 * @brief Returns current state of mag calibration and estimated values of the parameters
 * @param [out ] state currend calibration state
 * @param [in  ] structure to fill estimated values
 * @retval N/A
 ******************************************************************************/
uint8_t GetMagAlignEstimatedParams(real *params);

#pragma pack(1)
// example of user payload structure
typedef struct {
    uint8_t   parameter[8];
} magAlignCmdPayload;
#pragma pack()

typedef struct{
    real  hardIron_X;
    real  hardIron_Y;
    real  softIron_Ratio;
    real  softIron_Angle;
}magAlignUserParams_t;

/*****
 * 
 * 
 ****/
uint8_t ProcessMagAlignCmds(magAlignCmdPayload*  pld, uint8_t *payloadLen);

/*****
 * 
 * 
 ****/
void getUserMagAlignParams(magAlignUserParams_t *params);

/*****
 * 
 * 
 ****/
void setUserMagAlignParams(magAlignUserParams_t *params);

#endif
