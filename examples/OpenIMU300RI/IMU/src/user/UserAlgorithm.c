/** ***************************************************************************
 * @file   UserAlgorithm.c
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

#include <stddef.h>

#include "algorithmAPI.h"
#include "gpsAPI.h"
#include "platformAPI.h"
#include "userAPI.h"

#include "Indices.h"
#include "GlobalConstants.h"

#include "algorithm.h"
#include "EKF_Algorithm.h"
#include "BITStatus.h"
#include "UserConfiguration.h"
#include "bsp.h"
#include "debug.h"

#include "MagAlign.h"
#include "sae_j1939.h"

#define DEBUG_FREQ_0HZ  0
#define DEBUG_FREQ_1HZ  1
#define DEBUG_FREQ_2HZ  2
#define DEBUG_FREQ_5HZ  5


//
void _Algorithm(int dacqRate, uint8_t algoType);

// Initialize GPS algorithm variables
void InitUserAlgorithm()
{
    // place required initialization here
}

void *RunUserNavAlgorithm(double *accels, double *rates, double *mags, gpsDataStruct_t *gps, uint16_t dacqRate)
{

    // So far the returned value from this function is unused by external functions.  The
    //   NULL pointer is returned instead of a data structure.
    return NULL;
}

