/*
 * File:   TimingVars.h
 * Author: joemotyka
 *
 * Created on April 9, 2016, 9:39 PM
 */


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
#include <iostream>     // std::cin, std::cout
#include <fstream>      // std::ifstream
#include <string>
#include <sstream>
void TimingVars_DiagnosticMsg(std::string msg);
#endif

uint32_t TimingVars_GetTimeStep(void);

typedef struct {
    uint8_t  odr;

    uint32_t secondCntr;
    uint8_t tenHertzCntr;
    int8_t subFrameCntr;
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

