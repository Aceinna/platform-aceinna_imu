/*
 * File:   UserAlgorithm.h
 * Author: joemotyka
 *
 * Created on June 28, 2018, 12:23 AM
 */

#ifndef _USER_ALGORITHM_H_
#define _USER_ALGORITHM_H_

#include "GlobalConstants.h"

// Leveler related functions
void Leveler_GetAttitude_EA(real *EulerAngles);
void Leveler_SetExeFreq(uint16_t freq);
void Leveler_InitializeDataStruct(void);

// Leveler data structure
typedef struct {
    // Leveler calling-frequency and derived 
    uint16_t callingFreq;
    real dt;

    // Timer output counter
    uint32_t timerCntr, dTimerCntr;

    // Algorithm states
    real measuredEulerAngles_BinN[2];
} LevelerDataStruct;

extern LevelerDataStruct gLeveler;

#endif /* _USER_ALGORITHM_H_ */

