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

// Compass related functions
void Compass_GetAttitude_EA(real *EulerAngles);
void Compass_SetExeFreq(uint16_t freq);
void Compass_InitializeAlgorithmStruct(uint16_t callingFreq);

void Compass_Algorithm(void);
void Compass_SetInputStruct(double *accels, double *rates, double *mags, gpsDataStruct_t *gps);
void Compass_SetOutputStruct(void);

// Compass input data structure
typedef struct {
    // Sensor readings
    double accel_B[NUM_AXIS];
    double angRate_B[NUM_AXIS];
    double magField_B[NUM_AXIS];
} CompassInputDataStruct;


// Compass output data structure
typedef struct {
    // Angles converted to degrees
    double measuredEulerAngles_BinN[NUM_AXIS];
} CompassOutputDataStruct;


// Compass data structure
typedef struct {
    // Compass calling-frequency and derived 
    uint16_t callingFreq;
    real dt;

    // Timer output counter
    uint32_t timerCntr, dTimerCntr;

    // Algorithm states
    real measuredEulerAngles_BinN[NUM_AXIS];

    // Input/output data items
    CompassInputDataStruct  input;
    CompassOutputDataStruct output;
} CompassDataStruct;

extern CompassDataStruct gCompass;

#endif /* _COMPASS_H_ */
