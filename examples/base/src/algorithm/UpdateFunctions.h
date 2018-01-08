/*
 * File:   UpdateFunctions.h
 * Author: joemotyka
 *
 * Created on May 8, 2016, 12:35 AM
 */

#ifndef UPDATEFUNCTIONS_H
#define UPDATEFUNCTIONS_H

#include "UpdateMatrixSizing.h"
#include <stdint.h>    // for uint8_t, ...
#include "GlobalConstants.h"

// Make these #defines
#define  ROWS_IN_P  16
#define  COLS_IN_P  16

#ifdef EULER_ANGLE_SOLN
#define  ROWS_IN_H  3
#define  ROWS_IN_R  3
#else
#define  ROWS_IN_H  4
#define  ROWS_IN_R  4
#endif

#define  COLS_IN_H  16

#define  COLS_IN_R  ROWS_IN_R

void EKF_UpdateStage(void);

void Update_AHRS_Q(void);
void Update_AHRS_EA(void);
void Update_VG(void);
void Update_MagOnly(void);

void ComputeSystemInnovation_AHRS_EA(void);
void ComputeSystemInnovation_AHRS_Q(void);

void ComputeSystemInnovation_INS(void);
void ComputeSystemInnovation_MagOnly(void);

uint8_t _GenerateObservationJacobian_AHRS_EA(void);
void GenerateObservationJacobian_INS(void);
void GenerateObservationJacobian_MagOnly(void);

void _GenerateObservationCovariance_AHRS_EA(void);
void _GenerateObservationCovariance_AHRS_Q(void);
void GenerateObservationCovariance_INS(void);
void GenerateObservationCovariance_MagOnly(void);


#endif /* UPDATEFUNCTIONS_H */

