/******************************************************************************
 * @file TimingVars.h
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



#include <stdint.h>  // uint8_t, etc.


#ifndef TIMINGVARS_H
#define TIMINGVARS_H

#include "GlobalConstants.h"

//
void TimingVars_Increment(void);
float TimingVars_GetTime(void);

void  TimingVars_SetTMin(float tMin);
float TimingVars_GetTMin(void);

void TimingVars_DisplayTimerVars(signed long timeStep);

#ifdef DISPLAY_DIAGNOSTIC_MSG
  #ifdef INS_OFFLINE
    #include <iostream>     // std::cin, std::cout
    #include <fstream>      // std::ifstream
    #include <string>
    #include <sstream>
    void TimingVars_DiagnosticMsg(std::string msg);
  #else
    #include "debug.h"
    void TimingVars_DiagnosticMsg(char *msg);
  #endif
#endif


uint32_t TimingVars_GetTimeStep(void);

typedef struct {
    uint8_t  dacqFrequency;
    uint32_t secondCntr;
    uint8_t  tenHertzCntr;
    int8_t   subFrameCntr;
    uint16_t basicFrameCounter;

    // toggles between 0 and 1 at 200 Hz (currently used in firmware)
    int oneHundredHertzFlag;

    //
    float time;
    float tMin;
} TimingVars;

extern TimingVars timer;

void Initialize_Timing(void);

#endif /* TIMINGVARS_H */

