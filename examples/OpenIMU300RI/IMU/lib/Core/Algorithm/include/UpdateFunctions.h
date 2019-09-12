/*
 * File:   UpdateFunctions.h
 * Author: joemotyka
 *
 * Created on May 8, 2016, 12:35 AM
 */

#ifndef UPDATEFUNCTIONS_H
#define UPDATEFUNCTIONS_H

#include <stdint.h>    // for uint8_t, ...
#include "GlobalConstants.h"

// Make these #defines
#define  ROWS_IN_P  16
#define  COLS_IN_P  16

#define  ROWS_IN_H  3
#define  COLS_IN_H  16

#define  ROWS_IN_R  3
#define  COLS_IN_R  ROWS_IN_R

#define  ROWS_IN_K  16

void EKF_UpdateStage(void);

// Functions to split the INS update across multiple iterations, so the update can
// complete in the required 10 ms
void Update_Pos(void);
void Update_Vel(void);
void Update_Att(void);

void ComputeSystemInnovation_Att(void);
void ComputeSystemInnovation_Pos(void);
void ComputeSystemInnovation_Vel(void);


uint8_t _GenerateObservationJacobian_AHRS(void);

void _GenerateObservationCovariance_AHRS(void);
void _GenerateObservationCovariance_INS(void);


#endif /* UPDATEFUNCTIONS_H */

