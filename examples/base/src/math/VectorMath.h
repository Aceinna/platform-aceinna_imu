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

#endif /* VECTORMATH_H */
