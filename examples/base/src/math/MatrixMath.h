/** ***************************************************************************
* @file matrixMath.h real precision Linear algebra calculation functions
* @author
* @date   September, 2008
* @copyright (c) 2013, 2014 All Rights Reserved.
* @section LICENSE
* THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
* KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
* PARTICULAR PURPOSE.
* @brief
*****************************************************************************/

#ifndef MATRIXMATH_H
#define MATRIXMATH_H

#include <stdint.h>  // uint8_t, etc.
#include "GlobalConstants.h"  // for real

// #defines for the 4x4 matrix inverse (to make debugging easier)
#define A11 *(A+0*4+0)
#define A12 *(A+0*4+1)
#define A13 *(A+0*4+2)
#define A14 *(A+0*4+3)

#define A21 *(A+1*4+0)
#define A22 *(A+1*4+1)
#define A23 *(A+1*4+2)
#define A24 *(A+1*4+3)

#define A31 *(A+2*4+0)
#define A32 *(A+2*4+1)
#define A33 *(A+2*4+2)
#define A34 *(A+2*4+3)

#define A41 *(A+3*4+0)
#define A42 *(A+3*4+1)
#define A43 *(A+3*4+2)
#define A44 *(A+3*4+3)

// Function declarations
uint8_t AxBTranspose(real *A, real *B, uint8_t rowsInA, uint8_t colsInA, uint8_t rowsInB, real *C);
uint8_t AxB(real *A, real *B, uint8_t rowsInA, uint8_t colsInA, uint8_t colsInB, real *C);
uint8_t AxV(real *A, real *V, uint8_t rowsInA, uint8_t colsInA, real *C);
uint8_t APlusB(real *A, real *B, uint8_t rowsInA, uint8_t colsInA, real *C);
uint8_t AMinusB(real *A, real *B, uint8_t rowsInA, uint8_t colsInA, real *C);
uint8_t AxScalar(real *A, real c, uint8_t rowsInA, uint8_t colsInA, real *C);
real DotProduct(real *V1, real *V2, uint8_t rowsInA);
void ForceMatrixSymmetry(real *A, uint8_t rowsInA, uint8_t colsInA);
void ForceMatrixSymmetry_avg(real *A, uint8_t rowsInA, uint8_t colsInA);
void LimitMatrixValues(real* A, real limit, uint8_t rowsInA, uint8_t colsInA);
void LimitValuesAndForceMatrixSymmetry_avg(real* A, real limit, uint8_t rowsInA, uint8_t colsInA);
void LimitValuesAndForceMatrixSymmetry_noAvg(real* A, real limit, uint8_t rowsInA, uint8_t colsInA);

uint8_t matrixInverse_2x2(real *A, real *AInverse);
uint8_t matrixInverse_3x3(real* A, real* AInverse);
int MatrixInverse_4x4(real* A, real* AInverse);

// RLE implementation
#endif /* MATRIXMATH_H */
