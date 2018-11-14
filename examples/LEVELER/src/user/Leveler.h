/*
 * File:   UserAlgorithm.h
 * Author: joemotyka
 *
 * Created on June 28, 2018, 12:23 AM
 */

#ifndef _COMPASS_H_
#define _COMPASS_H_

//#include "GlobalConstants.h"
#include "Indices.h"

#include "gpsAPI.h"   // for gpsDataStruct_t

// Leveler related functions
void Leveler_GetAttitude_EA(real *EulerAngles);
void Leveler_SetExeFreq(uint16_t freq);
void Leveler_InitializeAlgorithmStruct(uint16_t callingFreq);

void Leveler_Algorithm(void);
void Leveler_SetInputStruct(double *accels, double *rates, double *mags, gpsDataStruct_t *gps);
void Leveler_SetOutputStruct(void);

// Compass input data structure
typedef struct {
    // Sensor readings
    double accel_B[NUM_AXIS];
    double angRate_B[NUM_AXIS];
    double magField_B[NUM_AXIS];
} LevelerInputDataStruct;


// Compass output data structure
typedef struct {
    // Angles converted to degrees
    double measuredEulerAngles_BinN[2];
} LevelerOutputDataStruct;


// Leveler data structure
typedef struct {
    // Leveler calling-frequency and derived 
    uint16_t callingFreq;
    real dt;

    // Timer output counter
    uint32_t timerCntr, dTimerCntr;

    // Algorithm states
    real measuredEulerAngles_BinN[2];

    // Input/output data items
    LevelerInputDataStruct  input;
    LevelerOutputDataStruct output;
} LevelerDataStruct;

extern LevelerDataStruct gLeveler;

#endif /* _COMPASS_H_ */
