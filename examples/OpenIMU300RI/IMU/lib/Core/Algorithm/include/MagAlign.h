/******************************************************************************
 * File:   MagAlign.h
 *******************************************************************************/
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

#ifndef MAGALIGN_H
#define MAGALIGN_H

#include "GlobalConstants.h"

/* new mag-align progress status   */
#define MAG_ALIGN_STATUS_LEVEL_START  			0x6
#define MAG_ALIGN_STATUS_LEVEL_END    			0xF
#define MAG_ALIGN_STATUS_SAVE2EEPROM  			0xE
#define MAG_ALIGN_STATUS_START_CAL_WITHOUT_AUTOEND   0x9
#define MAG_ALIGN_STATUS_START_CAL_WITH_AUTOEND      0xC
#define MAG_ALIGN_STATUS_TERMINATION                 0xB
#define MAG_ALIGN_STATUS_COMPLETE                    0xD
#define MAG_ALIGN_STATUS_ACCEPTED                    0xA
#define MAG_ALIGN_STATUS_IDLE                        0x0

#define ONE_DEGREE_Q29             9370165

typedef struct {
    real            hardIronBias[2];
    real            softIronAngle;
    real            softIronScaleRatio;
} ComputedMagneticParams;

typedef struct {
    real            hardIronBias[2];
    real            softIronAngle;
    real            softIronScaleRatio;
    real            SF[4];

    ComputedMagneticParams estParams;

    volatile int    state;
} MagAlignStruct;

extern MagAlignStruct gMagAlign;

#endif /* MAGALIGN_H */
