/** ***************************************************************************
 * @file   UserAlgorithm.h
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

#ifndef _USER_ALGORITHM_H_
#define _USER_ALGORITHM_H_

#include <stdint.h>
#include "GlobalConstants.h"

 // IMU data structure
typedef struct {
    // Timer output counter
    uint32_t timerCntr, dTimerCntr;

    // Algorithm states
    double accel_g[3];
    double rate_radPerSec[3];
    double rate_degPerSec[3];
    double mag_G[3];
    double temp_C;
} IMUDataStruct;

extern IMUDataStruct gIMU;

#endif /* _USER_ALGORITHM_H_ */

