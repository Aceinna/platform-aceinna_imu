/** ***************************************************************************
 * @file   taskDataAcquisition.h
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * sensor data acquisition task runs at 100Hz, gets the data for each sensor
 * and applies available calibration
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

#ifndef _TASK_DATA_ACQUISITION_H_
#define _TASK_DATA_ACQUISITION_H_

#include "stdint.h"
#include "GlobalConstants.h"

extern void TaskDataAcquisition(void const *argument);
void PrepareToNewDacqTick();
void TaskDataAcquisition_Init(void);
void GetSensorsData(void);
void ProcessSensorsData(void);
void EnterMainAlgLoop(void);
void DataAquisitionStart(void);
BOOL isOneHundredHertzFlag(void);
extern uint8_t gpsUpdateCpy;
extern uint8_t gnss_signal_flag;
extern uint8_t g_next_time_available;//0:not available 1:available 2:uncertain

#endif