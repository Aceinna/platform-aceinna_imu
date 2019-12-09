/** ******************************************************************************
 * @file algorithmAPI.h API functions for Interfacing with lgorithm
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

#ifndef _ALGORITHM_API_H
#define _ALGORITHM_API_H

#include <stdint.h>
#include "GlobalConstants.h"
#include "algorithm.h"

/******************************************************************************
 * @name Initialize_AlgorithmStruct
 * @brief initializes the values in the AlgorithmStruct
 *
 * @retval N/A
 *****************************************************************************/
void InitializeAlgorithmStruct(uint8_t callingFreq);

/******************************************************************************
 * @brief Get algorithm status.
 * Refer to the declaration of struct ALGO_STATUS_BITS to see the details about
 * algorithm status.
 * @param [in] algoStatus: to store the returned values.
 * @param [out] algoStatus
 * @retval None.
******************************************************************************/
void GetAlgoStatus(AlgoStatus *algoStatus);

void setAlgorithmStateStabilizeSystem();

uint16_t getAlgorithmCounter();

/******************************************************************************
 * @brief Set the calling frequency of the algorithm.
 * @param [in]
 * @param [out]
 * @retval None.
******************************************************************************/
void setAlgorithmExeFreq(int freq);

void enableMagInAlgorithm(BOOL enable);

BOOL magUsedInAlgorithm();

void enableGpsInAlgorithm(BOOL enable);

BOOL gpsUsedInAlgorithm();

void   SetAlgorithmUseDgps(BOOL d);

void updateAlgorithmTimings(int corr, uint32_t tmrVal );

uint32_t getAlgorithmTimer();

uint16_t getAlgorithmCounter();
uint16_t getAlgorithmFrequency();
uint32_t getAlgorithmITOW();


/******************************************************************************
 * @name setLeverArm
 * @brief Set the position of the antenna relative to the IMU in the vehicle
 * body frame. Lever arm will be substracted from the GNSS measurement to get
 * the position of the IMU.
 * @param [in] leverArmBx/y/z: lever arm in the vehicle body frame, in unti of meters.
 * @param [out]
 * @retval None.
 *****************************************************************************/
void setLeverArm( real leverArmBx, real leverArmBy, real leverArmBz );

/******************************************************************************
 * @name setPointOfInterest
 * @brief Set the position of the point of interest relative to the IMU in the
 * vehicle body frame. This value will be added to the position of the IMU to get
 * the absolute position of the point of interest.
 * @param [in] poiBx/y/z: position of the point of interest in the vehicle
 *                        body frame, in unti of meters.
 * @param [out]
 * @retval None.
 *****************************************************************************/
void setPointOfInterest( real poiBx, real poiBy, real poiBz );

#endif
