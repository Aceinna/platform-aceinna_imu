/** ***************************************************************************
 * @file sensorsI.h API functions for Magnitometer functionality
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

#ifndef _SENSORS_API_H
#define _SENSORS_API_H

#include <stdint.h>



typedef struct {
    // Timer output counter
    uint32_t timerCntr, dTimerCntr;

    // Algorithm states
    double accel_g[3];
    double rate_radPerSec[3];
    double rate_degPerSec[3];
//  double mag_G[3];
    double temp_C;
} IMUDataStruct;
/** ****************************************************************************
 * @name GetAccelData_g
 * @brief Get scaled accelerometer data in G
 * @param [in] data - pointer to external data structure
 * @retval N/A
 ******************************************************************************/
void  GetAccelData_g(float *data);
void GetAccelData_g_AsDouble(double *data);

/** ****************************************************************************
 * @name GetAccelData_mPerSecSq
 * @brief Get scaled accelerometer data in m/s/s
 * @param [in] data - pointer to external data structure
 * @retval N/A
 ******************************************************************************/
void  GetAccelData_mPerSecSq(float *data);

/** ****************************************************************************
 * @name GetRateData_radPerSec
 * @brief Get scaled rate sesnors data in rad/ses
 * @param [in] data - pointer to external data structure
 * @retval N/A
 ******************************************************************************/
void  GetRateData_radPerSec(float *data);
void  GetRateData_radPerSec_AsDouble(double *data);

/** ****************************************************************************
 * @name GetRateData_degPerSec
 * @brief Get scaled rate sesnors data in deg/ses
 * @param [in] data - pointer to external data structure
 * @retval N/A
 ******************************************************************************/
void  GetRateData_degPerSec(float *data);
void  GetRateData_degPerSec_AsDouble(double *data);

/** ****************************************************************************
 * @name GetMagData_G
 * @brief Get scaled magnetometer data in Gauss
 * @param [in] data - pointer to external data structure
 * @retval N/A
 ******************************************************************************/
void  GetMagData_G(float *data);
void  GetMagData_G_AsDouble(double *data);


/** ****************************************************************************
 * @name GetBoardTempData
 * @brief Get Board Temperature 
 * @param [in] temp - pointer to external data structure
 * @retval N/A
 ******************************************************************************/
void  GetBoardTempData(float *temp); 
void  GetBoardTempData_AsDouble(double *data);

/** ****************************************************************************
 * @name GetUnitTemp
 * @brief Get Board Temperature 
 * @param [in] N/A
 * @retval unit temperature in deg C
 ******************************************************************************/
float GetUnitTemp();

/** ****************************************************************************
 * @name FillRawSensorsData
 * @brief Prepares raw sensors data for calibration 
 * @retval N/A
 ******************************************************************************/
void FillRawSensorsData();


/** ****************************************************************************
 * @name CombineSensorsData
 * @brief Combines sensors data after calibration 
 * @retval N/A
 ******************************************************************************/
void CombineSensorsData();

/** ****************************************************************************
 * @name initSensorsData
 * @brief Initialized sensors data structures 
 * @retval N/A
 ******************************************************************************/
void InitSensorsData();


/** ****************************************************************************
 * @name SapmleSensorsData
 * @brief Performs sampling of sensors data 
 * @retval N/A
 ******************************************************************************/
void SampleSensorsData();


/** ****************************************************************************
 * @name ActivateSensors 
 * @brief Performs activation of selected sensors 
 * @retval N/A
 ******************************************************************************/
uint8_t  ActivateSensors();

/** ****************************************************************************
 * @name InitSensors 
 * @brief Performs initialization of selected sensors 
 * @retval N/A
 ******************************************************************************/
uint8_t  InitSensors();


#endif
