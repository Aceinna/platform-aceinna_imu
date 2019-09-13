/* 
 * File:   VectorMath.cpp
 * Author: joemotyka
 * 
 * Created on May 7, 2016, 12:50 AM
 */

#include "VectorMath.h"

#include <math.h>

// Function definitions for reals
void VectorNormalize( real *vectorIn, real *vectorOut )
{
    real vectorMag;
    real multiplier;

    // sqrt( v1^2 + v2^2 + v3^2 )
    vectorMag = VectorMag( vectorIn );
    multiplier = (real)1.0 / vectorMag;

    vectorOut[0] = multiplier * vectorIn[0];
    vectorOut[1] = multiplier * vectorIn[1];
    vectorOut[2] = multiplier * vectorIn[2];
}


// Components must be less than 1239850262 (Q27) ~ 9.23 (dec) in order to use this function without
//   overflow.  Else, need to represent the output in another q-format.
real VectorMag( real *vectorIn )
{
    real temp[3];

    //
    temp[0] = vectorIn[0] * vectorIn[0];
    temp[1] = vectorIn[1] * vectorIn[1];
    temp[2] = vectorIn[2] * vectorIn[2];

    // sqrt( v1^2 + v2^2 + v3^2 )
    return( (real)(sqrtf( temp[0] + temp[1] + temp[2] )) );
}


void VectorCrossProduct( real *vect1, real *vect2, real *vectOut )
{
    // |   i       j       k   |
    // | v1[0]   v1[1]   v1[2] | = i*( v1[1]*v2[2] - v1[2]*v2[1] ) - j*( v1[0]*v2[2] - v1[2]*v2[0] ) + k*( v1[0]*v2[1] - v1[1]*v2[0] )
    // | v2[0]   v2[1]   v2[2] |

    vectOut[0] = vect1[1] * vect2[2] - vect1[2] * vect2[1];   // v1[1]*v2[2] - v1[2]*v2[1]
    vectOut[1] = vect1[2] * vect2[0] - vect1[0] * vect2[2];   // v1[2]*v2[0] - v1[0]*v2[2]
    vectOut[2] = vect1[0] * vect2[1] - vect1[1] * vect2[0];   // v1[0]*v2[1] - v1[1]*v2[0]
}

real VectorDotProduct( real *vect1, real *vect2 )
{
    real result;

    result = vect1[0] * vect2[0] + 
             vect1[1] * vect2[1] + 
             vect1[2] * vect2[2];

    return( result );
}

real vecVar(real *a, real am, int n)
{
    int i;
    real sum = 0.0;
    for (i = 0; i < n; i++)
    {
        real d = a[i] - am;
        sum += d * d;
    }
    return sum / n;
}

void cross(real* a, real* b, real* axb)
{
    axb[0] = a[1] * b[2] - a[2] * b[1];
    axb[1] = a[2] * b[0] - a[0] * b[2];
    axb[2] = a[0] * b[1] - a[1] * b[0];
}
