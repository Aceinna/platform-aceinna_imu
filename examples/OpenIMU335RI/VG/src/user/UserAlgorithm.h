/*
 * File:   UserAlgorithm.h
 * Author: joemotyka
 *
 * Created on June 28, 2018, 12:23 AM
 */

#ifndef USER_ALGORITHM_H
#define USER_ALGORITHM_H

#include <stdint.h>

 // IMU data structure
typedef struct {
    // Timer output counter
//    uint32_t timerCntr, dTimerCntr;

    // Algorithm states
    float64_t accel_g[3];
    float64_t rate_radPerSec[3];
    float64_t rate_degPerSec[3];
    float64_t mag_G[3];
    float64_t temp_C;
} IMUDataStruct;

extern IMUDataStruct gIMU;

#endif /* _USER_ALGORITHM_H_ */

