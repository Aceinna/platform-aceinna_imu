/*
 * File:   UserAlgorithm.h
 * Author: joemotyka
 *
 * Created on June 28, 2018, 12:23 AM
 */

#ifndef _USER_ALGORITHM_H_
#define _USER_ALGORITHM_H_

#include <stdint.h>
#include "GlobalConstants.h"
#include "sensorsAPI.h"
 // IMU data structure
// typedef struct {
//     // Timer output counter
//     uint32_t timerCntr, dTimerCntr;

//     // Algorithm states
//     double accel_g[3];
//     double rate_radPerSec[3];
//     double rate_degPerSec[3];
// //  double mag_G[3];
//     double temp_C;
// } IMUDataStruct;

extern IMUDataStruct gIMU;

#endif /* _USER_ALGORITHM_H_ */

