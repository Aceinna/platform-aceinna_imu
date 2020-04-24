/*
* File:   VectorMath.h
* Author: joemotyka
*
* Created on May 7, 2016, 12:50 AM
*/

#ifndef VECTORMATH_H
#define VECTORMATH_H

#include "GlobalConstants.h"

// Declare the function definitions for real
void VectorNormalize(real *vectorIn, real *vectorOut);
real VectorMag(real *vectorIn);
void VectorCrossProduct(real *vect1, real *vect2, real *vectOut);
real VectorDotProduct(real *vect1, real *vect2);

/******************************************************************************
* @brief Compute the variance of a vector
* @param [in] a     pointer to the vector.
* @param [in] am    mean value of the vector.
* @param [in] n     number of elements in the vector.
* @param [out]
* @retval           variance of the vector
******************************************************************************/
real vecVar(real *a, real am, int n);

/******************************************************************************
* @brief vector coross product
* @param [in]   a   3D vector
* @param [in]   b   3D vector
* @param [out]  c   c = axb
* return:
******************************************************************************/
void cross(real* a, real* b, real* axb);

#endif /* VECTORMATH_H */
