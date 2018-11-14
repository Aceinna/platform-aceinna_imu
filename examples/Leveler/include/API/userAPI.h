/** ******************************************************************************
 * @file userAPI.h API functions for Interfacing with user algorithm
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

#ifndef _USER_API_H
#define _USER_API_H

// Some common constants used in the user algorithm logic
#define  ZERO_HZ         0
#define  ONE_HZ          1
#define  TWO_HZ          2
#define  FOUR_HZ         4
#define  FIVE_HZ         5
#define  TEN_HZ          10
#define  TWENTY_HZ       20
#define  TWENTY_FIVE_HZ  25
#define  FIFTY_HZ        50

#define  NUM_AXIS  3

#include <stdint.h>
#include "gpsAPI.h"

void  inertialAndPositionDataProcessing(uint16_t dacqRate);

void *RunUserNavAlgorithm(double *accels, double *rates, double* mags, gpsDataStruct_t *gps, uint16_t dacqRate);
void  WriteResultsIntoOutputStream(void *results) ;
void  InitUserDataStructures();
void  InitUserFilters();       
void  InitUserAlgorithm();     
void  initUserDataProcessingEngine();
void  userInitConfigureUnit();

#endif
