/** ***************************************************************************
* @file matrixMath.c real precision Linear algebra calculation functions
* @author
* @date   September, 2008
* @copyright (c) 2013, 2014 All Rights Reserved.
* @section LICENSE
* THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
* KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
* PARTICULAR PURPOSE.
* @details
* NOTE these count on the matrices being defined per the inputs.
* They use pointer math instead of array indexing so you can't pass in a
* 7x3 matrix and attempt to calculate using the first two columns
*****************************************************************************/

#include "MatrixMath.h"

/** ****************************************************************************
* @name: ATimesBTranspose  A*B'
* @brief Compute A x BTranspose (Only the B matrix is provided to the function,
*   the inverse is performed automatically).  Tested June 25, 2014.
* TRACE:
* @param [in] A matrix
* @param [in] B matrix
* @param [in]  A[rowsInA][]
* @param [in]  A[][colsInA]
* @param [in]  B[rowsInB]
* @param [out] C[][]
* @retval always 1
******************************************************************************/
uint8_t AxBTranspose( real *A,
                      real *B,
                      uint8_t rowsInA,
                      uint8_t colsInA,
                      uint8_t rowsInB,
                      real *C )
{
    uint8_t rowNum, colNum, multIndex;

    // Compute A*transpose( B )
    for (rowNum = 0; rowNum < rowsInA; rowNum++) {
        for (colNum = 0; colNum < rowsInB; colNum++) {
            *(C + rowNum*rowsInB + colNum) = 0.0;
            for (multIndex = 0; multIndex < colsInA; multIndex++) {
                *(C + rowNum*rowsInB + colNum) = *(C + rowNum*rowsInB + colNum) +
                    *(A + rowNum*colsInA + multIndex) * *(B + colNum*colsInA + multIndex);
            }
        }
    }
    return 1;
}


/** ****************************************************************************
* @name: ATimesB  A * B
* @brief Compute A * B Tested June 25, 2014.
* TRACE:
* @param [in] A matrix
* @param [in] B matrix
* @param [in]  A[rowsInA][]
* @param [in]  A[][colsInA]
* @param [in]  B[rowsInB]
* @param [out] C[][]
* @retval always 1
******************************************************************************/
uint8_t AxB( real *A,
             real *B,
             uint8_t rowsInA,
             uint8_t colsInA,
             uint8_t colsInB,
             real  *C )
{
    uint8_t rowNum, colNum, multIndex;

    // Compute A * B
    for (rowNum = 0; rowNum < rowsInA; rowNum++) {
        for (colNum = 0; colNum < colsInB; colNum++) {
            *(C + rowNum*colsInB + colNum) = 0.0;
            for (multIndex = 0; multIndex < colsInA; multIndex++) {
                *(C + rowNum*colsInB + colNum) = *(C + rowNum*colsInB + colNum) +
                    *(A + rowNum*colsInA + multIndex) * *(B + colNum + colsInB*multIndex);
            }
        }
    }
    return 1;
}


/** ****************************************************************************
* @name: ATimesV  A * V multiply a matrix by a vector
* @brief Compute A * V Tested June 25, 2014. Assumes the vector is the right
*        size
* TRACE:
* @param [in] A matrix
* @param [in] V vector
* @param [in]  A[rowsInA][]
* @param [in]  A[][colsInA]
* @param [out] C[][]
* @retval always 1
******************************************************************************/
uint8_t AxV( real *A,
             real *V,
             uint8_t rowsInA,
             uint8_t colsInA,
             real  *C )
{
    uint8_t rowNum, multIndex;

    for (rowNum = 0; rowNum < rowsInA; rowNum++) {
        *(C + rowNum) = 0.0;
        for (multIndex = 0; multIndex < colsInA; multIndex++) {
            *(C + rowNum) = *(C + rowNum) + *(A + rowNum*colsInA + multIndex) * *(V + multIndex);
        }
    }

    return 1;
}


/** ****************************************************************************
* @name: ATimesBTranspose  V1 dot  V2
* @brief Compute A' * B Tested June 25, 2014. Assumes the vectors are the same
*        size
* TRACE:
* @param [in] V1 vector
* @param [in] V2 vector
* @param [in]  [size]
* @retval the calculated dot product
******************************************************************************/
real DotProduct(real *V1,
    real *V2,
    uint8_t size)
{
    real VOut = 0.0;
    uint8_t rowNum = 0;

    for (rowNum = 0; rowNum < size; rowNum++) {
        VOut = VOut + *(V1 + rowNum) * *(V2 + rowNum);
    }

    return VOut;
}


/** ****************************************************************************
* @name: APlusB  A + B
* @brief Compute A + B Tested June 25, 2014. Assumes the matrices are the same
*        size
* TRACE:
* @param [in] A matrix
* @param [in] B matrix
* @param [in]  A[rowsInA][]
* @param [in]  A[][colsInA]
* @retval always 1
******************************************************************************/
uint8_t APlusB( real *A,
                real *B,
                uint8_t rowsInA,
                uint8_t colsInA,
                real *C )
{
    uint8_t rowNum = 0, colNum = 0;

    for (rowNum = 0; rowNum < rowsInA; rowNum++) {
        for (colNum = 0; colNum < colsInA; colNum++) {
            *(C + rowNum*colsInA + colNum) = *(A + rowNum*colsInA + colNum) +
                *(B + rowNum*colsInA + colNum);
        }
    }

    return 1;
}


/** ****************************************************************************
* @name: APlusB  A * c
* @brief Compute C = A * c where c is a scaler
* TRACE:
* @param [in] A matrix
* @param [in] c scaler
* @param [in]  A[rowsInA][]
* @param [in]  A[][colsInA]
* @param [out]  C[][] result
* @retval always 1
******************************************************************************/
uint8_t AxScalar( real *A,
                  real c,
                  uint8_t rowsInA,
                  uint8_t colsInA,
                  real *C )
{
    uint8_t rowNum = 0, colNum = 0;

    for (rowNum = 0; rowNum < rowsInA; rowNum++) {
        for (colNum = 0; colNum < colsInA; colNum++) {
            *(C + rowNum*colsInA + colNum) = c * *(A + rowNum*colsInA + colNum);
        }
    }

    return 1;
}

/** ****************************************************************************
* @name: AMinusB  A - B
* @brief Compute C = A - B Tested June 25, 2014.
* TRACE:
* @param [in] A matrix
* @param [in] B matrix
* @param [in]  A[rowsInA][]
* @param [in]  A[][colsInA]
* @param [out]  C[][] result
* @retval always 1
******************************************************************************/
uint8_t AMinusB( real *A,
                 real *B,
                 uint8_t rowsInA,
                 uint8_t colsInA,
                 real *C )
{
    uint8_t rowNum = 0, colNum = 0;

    for (rowNum = 0; rowNum < rowsInA; rowNum++) {
        for (colNum = 0; colNum < colsInA; colNum++) {
            *(C + rowNum*colsInA + colNum) = *(A + rowNum*colsInA + colNum) -
                *(B + rowNum*colsInA + colNum);
        }
    }
    return 1;
}


/** ****************************************************************************
* @name: matrixInverse_2x2  A
* @brief Compute A = 1/A Tested June 25, 2014. Assumes the matrix is invertible.)
* TRACE:
* @param [in] A matrix
* @param [out]  A[][] result
* @retval always 1
******************************************************************************/
uint8_t matrixInverse_2x2( real *A,
                           real *AInverse )
{
    real determinant = 1.0, multiplier = 1.0;

    determinant = *(A + 0) * *(A + 3) - *(A + 1) * *(A + 2);
    multiplier  = (real)1.0 / determinant;

    *(AInverse + 0) =  multiplier * *(A + 3);
    *(AInverse + 1) = -multiplier * *(A + 1);
    *(AInverse + 2) = -multiplier * *(A + 2);
    *(AInverse + 3) =  multiplier * *(A + 0);

    return 1;
}


/** ****************************************************************************
 * @name: EnsureSymmetricMatrix  A
 * @brief Compute A = 1/A Tested June 25, 2014.
 * TRACE:
 * @param [in/out] A matrix
 * @param [in]  A[rowsInA][]
 * @param [in]  A[][colsInA]
 * @retval always 1
 **************************************************************************** **/
void ForceMatrixSymmetry( real *A, uint8_t rowsInA, uint8_t colsInA )
{
    uint8_t rowNum, colNum;

    for (rowNum = 0; rowNum < rowsInA; rowNum++) {
        for (colNum = 0; colNum < rowNum; colNum++) {
            *(A + colNum*colsInA + rowNum) = *(A + rowNum*colsInA + colNum);
        }
    }
}

void ForceMatrixSymmetry_avg( real *A, uint8_t rowsInA, uint8_t colsInA )
{
    uint8_t rowNum, colNum;

    for (rowNum = 0; rowNum < rowsInA; rowNum++) {
        for (colNum = 0; colNum < rowNum; colNum++) {
            *(A + rowNum*colsInA + colNum) = (real)0.5 * ( *(A + rowNum*colsInA + colNum) +
                                                           *(A + colNum*colsInA + rowNum) );
            *(A + colNum*colsInA + rowNum) = *(A + rowNum*colsInA + colNum);
        }
    }
}

void LimitValuesAndForceMatrixSymmetry_avg(real* A, real limit, uint8_t rowsInA, uint8_t colsInA)
{
    uint8_t rowNum, colNum;

    for (rowNum = 0; rowNum < rowsInA; rowNum++) {
        for (colNum = 0; colNum < rowNum; colNum++) {
            *(A + rowNum*colsInA + colNum) = (real)0.5 * ( *(A + rowNum*colsInA + colNum) +
                                                           *(A + colNum*colsInA + rowNum) );

            if (*(A + rowNum*colsInA + colNum) > limit) {
                *(A + rowNum*colsInA + colNum) = limit;
            } else if (*(A + rowNum*colsInA + colNum) < -limit) {
                *(A + rowNum*colsInA + colNum) = -limit;
            }

            *(A + colNum*colsInA + rowNum) = *(A + rowNum*colsInA + colNum);
        }
    }
}

void LimitValuesAndForceMatrixSymmetry_noAvg(real* A, real limit, uint8_t rowsInA, uint8_t colsInA)
{
    uint8_t rowNum, colNum;

    for (rowNum = 0; rowNum < rowsInA; rowNum++) {
        for (colNum = 0; colNum < rowNum; colNum++) {
            if (*(A + rowNum*colsInA + colNum) > limit) {
                *(A + rowNum*colsInA + colNum) = limit;
            } else if (*(A + rowNum*colsInA + colNum) < -limit) {
                *(A + rowNum*colsInA + colNum) = -limit;
            }

            *(A + colNum*colsInA + rowNum) = *(A + rowNum*colsInA + colNum);
        }
    }
}


/** ****************************************************************************
* @name: matrixInverse_3x3  A
* @brief Compute A = 1/A Tested Sep 19, 2014 (Matlab). (Assumes the matrix is
*         invertible.)
* DKH 09.19.14 ported 440 code with change to input to allow indexing to ease
*              debuging
* TRACE:
* @param [in] A matrix input 3x3 real precision matrix
* @param [out]  AInverse - 1/A
* @retval always 1
******************************************************************************/
uint8_t matrixInverse_3x3( real* A,
                           real* AInverse )
{
    real temp[3];
    real detInv;

    temp[0] =  *(A + 8) * *(A + 4) - *(A + 7) * *(A + 5);
    temp[1] = -*(A + 8) * *(A + 1) + *(A + 7) * *(A + 2);
    temp[2] =  *(A + 5) * *(A + 1) - *(A + 4) * *(A + 0*3 + 2);
    detInv = (real)1.0 / ( *(A + 0*3 + 0) * temp[0] + *(A + 1*3 + 0) * temp[1] + *(A + 2*3 + 0) * temp[2] );

    *(AInverse + 0*3 + 0) = temp[0] * detInv;
    *(AInverse + 0*3 + 1) = temp[1] * detInv;
    *(AInverse + 0*3 + 2) = temp[2] * detInv;

    temp[0] = -*(A + 2*3 + 2) * *(A + 1*3 + 0) + *(A + 2*3 + 0) * *(A + 1*3 + 2);
    temp[1] =  *(A + 2*3 + 2) * *(A + 0*3 + 0) - *(A + 2*3 + 0) * *(A + 0*3 + 2);
    temp[2] = -*(A + 1*3 + 2) * *(A + 0*3 + 0) + *(A + 1*3 + 0) * *(A + 0*3 + 2);
    *(AInverse + 1*3 + 0) = temp[0] * detInv;
    *(AInverse + 1*3 + 1) = temp[1] * detInv;
    *(AInverse + 1*3 + 2) = temp[2] * detInv;

    temp[0] =  *(A + 2*3 + 1) * *(A + 1*3 + 0) - *(A + 2*3 + 0) * *(A + 1*3 + 1);
    temp[1] = -*(A + 2*3 + 1) * *(A + 0*3 + 0) + *(A + 2*3 + 0) * *(A + 0*3 + 1);
    temp[2] =  *(A + 1*3 + 1) * *(A + 0*3 + 0) - *(A + 1*3 + 0) * *(A + 0*3 + 1);
    *(AInverse + 2*3 + 0) = temp[0] * detInv;
    *(AInverse + 2*3 + 1) = temp[1] * detInv;
    *(AInverse + 2*3 + 2) = temp[2] * detInv;

    return 1;
}


// previous function was incorrect
// rewritten and retested on May 22, 2016
void LimitMatrixValues(real* A, real limit, uint8_t rowsInA, uint8_t colsInA)
{
    real tmp;
    uint8_t rowNum, colNum;

    for (rowNum = 0; rowNum < rowsInA; rowNum++) {
        for (colNum = 0; colNum < colsInA; colNum++) {
            tmp = *(A + rowNum*colsInA + colNum);

            if (tmp > limit) {
                *(A + rowNum*colsInA + colNum) = limit;
            } else if (tmp < -limit) {
                *(A + rowNum*colsInA + colNum) = -limit;
            }
        }
    }
}


/* *****
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
***** */

/** ****************************************************************************
* @name: matrixInverse_3x3  A
* @brief Compute A = 1/A Tested Sep 19, 2014 (Matlab). (Assumes the matrix is
*         invertible.)
* DKH 09.19.14 ported 440 code with change to input to allow indexing to ease
*              debuging
* TRACE:
* @param [in] A matrix input 3x3 real precision matrix
* @param [out]  AInverse - 1/A
* @retval always 1
******************************************************************************/
/* *****
int MatrixInverse_4x4(real* A, real* AInverse)
{
    real inv[4][4], det;

    real tmp[36];

    tmp[35] = A21 * A32;
    tmp[28] = A21 * A33;
    tmp[27] = A21 * A34;
    tmp[32] = A21 * A42;
    tmp[25] = A21 * A43;
    tmp[22] = A21 * A44;

    tmp[34] = A22 * A31;
    tmp[17] = A22 * A33;
    tmp[14] = A22 * A34;
    tmp[33] = A22 * A41;
    tmp[10] = A22 * A43;
    tmp[9]  = A22 * A44;

    tmp[29] = A23 * A31;
    tmp[16] = A23 * A32;
    tmp[13] = A23 * A34;
    tmp[24] = A23 * A41;
    tmp[11] = A23 * A42;
    tmp[6]  = A23 * A44;

    tmp[26] = A24 * A31;
    tmp[15] = A24 * A32;
    tmp[12] = A24 * A33;
    tmp[23] = A24 * A41;
    tmp[8]  = A24 * A42;
    tmp[7]  = A24 * A43;

    tmp[30] = A31 * A42;
    tmp[21] = A31 * A43;
    tmp[18] = A31 * A44;

    tmp[31] = A32 * A41;
    tmp[4]  = A32 * A43;
    tmp[3]  = A32 * A44;

    tmp[20] = A33 * A41;
    tmp[5]  = A33 * A42;
    tmp[0]  = A33 * A44;

    tmp[19] = A34 * A41;
    tmp[2]  = A34 * A42;
    tmp[1]  = A34 * A43;

    real temp[18];
    temp[0] =  tmp[0] - tmp[1];
    temp[1] =  tmp[2] - tmp[3];
    temp[2] =  tmp[4] - tmp[5];
    temp[3] =  tmp[6] - tmp[7];
    temp[4] =  tmp[8] - tmp[9];
    temp[5] =  tmp[10] - tmp[11];
    temp[6] =  tmp[12] - tmp[13];
    temp[7] =  tmp[14] - tmp[15];
    temp[8] =  tmp[16] - tmp[17];
    temp[9] =  tmp[18] - tmp[19];
    temp[10] = tmp[20] - tmp[21];
    temp[11] = tmp[22] - tmp[23];
    temp[12] = tmp[24] - tmp[25];
    temp[13] = tmp[26] - tmp[27];
    temp[14] = tmp[28] - tmp[29];
    temp[15] = tmp[30] - tmp[31];
    temp[16] = tmp[32] - tmp[33];
    temp[17] = tmp[34] - tmp[35];


    // First Row
    inv[0][0] =  (A22 * temp[0] + A23 * temp[1] + A24 * temp[2]);
    inv[0][1] = -(A12 * temp[0] + A13 * temp[1] + A14 * temp[2]);
    inv[0][2] =  (A12 * temp[3] + A13 * temp[4] + A14 * temp[5]);
    inv[0][3] =  (A12 * temp[6] + A13 * temp[7] + A14 * temp[8]);

    // Second Row
    inv[1][0] = -(A21 * temp[0] - A23 * temp[9] - A24 * temp[10]);
    inv[1][1] =  (A11 * temp[0] - A13 * temp[9] - A14 * temp[10]);
    inv[1][2] = -(A11 * temp[3] - A13 * temp[11] - A14 * temp[12]);
    inv[1][3] = -(A11 * temp[6] - A13 * temp[13] - A14 * temp[14]);

    // Third Row
    inv[2][0] = -(A21 * temp[1] + A22 * temp[9] - A24 * temp[15]);
    inv[2][1] =  (A11 * temp[1] + A12 * temp[9] - A14 * temp[15]);
    inv[2][2] = -(A11 * temp[4] + A12 * temp[11] - A14 * temp[16]);
    inv[2][3] = -(A11 * temp[7] + A12 * temp[13] - A14 * temp[17]);

    // Fourth row
    inv[3][0] = -(A21 * temp[2] + A22 * temp[10] + A23 * temp[15]);
    inv[3][1] =  (A11 * temp[2] + A12 * temp[10] + A13 * temp[15]);
    inv[3][2] = -(A11 * temp[5] + A12 * temp[12] + A13 * temp[16]);
    inv[3][3] = -(A11 * temp[8] + A12 * temp[14] + A13 * temp[17]);

    // compute the determinant (if "too small", declare the solution invalid) <-- problem with scaling here
    det = A11 * inv[0][0] + A12 * inv[1][0] + A13 * inv[2][0] + A14 * inv[3][0];

    BOOL retVal;
    if (det < 1e-8) {
        retVal = false;
    } else {
        retVal = true;
    }

    det = (real)1.0 / det;

    //
    *(AInverse + 0 * 4 + 0) = inv[0][0] * det;
    *(AInverse + 0 * 4 + 1) = inv[0][1] * det;
    *(AInverse + 0 * 4 + 2) = inv[0][2] * det;
    *(AInverse + 0 * 4 + 3) = inv[0][3] * det;

    *(AInverse + 1 * 4 + 0) = inv[1][0] * det;
    *(AInverse + 1 * 4 + 1) = inv[1][1] * det;
    *(AInverse + 1 * 4 + 2) = inv[1][2] * det;
    *(AInverse + 1 * 4 + 3) = inv[1][3] * det;

    *(AInverse + 2 * 4 + 0) = inv[2][0] * det;
    *(AInverse + 2 * 4 + 1) = inv[2][1] * det;
    *(AInverse + 2 * 4 + 2) = inv[2][2] * det;
    *(AInverse + 2 * 4 + 3) = inv[2][3] * det;

    *(AInverse + 3 * 4 + 0) = inv[3][0] * det;
    *(AInverse + 3 * 4 + 1) = inv[3][1] * det;
    *(AInverse + 3 * 4 + 2) = inv[3][2] * det;
    *(AInverse + 3 * 4 + 3) = inv[3][3] * det;

    return retVal;
}
***** */

