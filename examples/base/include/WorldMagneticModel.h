/*
* File:   WorldMagneticModel.h
* Author: joemotyka
*
* Created on May 16, 2016, 8:22 PM
*/

#ifndef WORLDMAGNETICMODEL_H
#define WORLDMAGNETICMODEL_H

#include "GlobalConstants.h"

#include <stdint.h>

// at 1000 mph at the equator this needs to be run once every couple of seconds
// at general aviation speed this needs to be run once every couple of minutes
void WMM_Initialize(void);

void WMM_GetMagVector( float    Lat,
                       float    Lon,
                       float    AltEllipsoid,
                       uint16_t Month,
                       uint16_t Day,
                       uint16_t Year,
                       float*   B,
                       float*   wmmDecl); // X - N, Y - E, Z - D


extern void TaskWorldMagneticModel(void);

//void WorldMagneticModel(void);

typedef struct {
    uint32_t timeOfLastSoln;
    int validSoln;

    float decl_rad;
} WorldMagModelStruct;

extern  WorldMagModelStruct  gWorldMagModel;

#endif /* MAGALIGN_H */
