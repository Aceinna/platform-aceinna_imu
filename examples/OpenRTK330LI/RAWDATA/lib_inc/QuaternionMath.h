/* 
 * File:   QuaternionMath.h
 * Author: joemotyka
 *
 * Created on May 7, 2016, 5:03 PM
 */

#ifndef QUATERNIONMATH_H
#define QUATERNIONMATH_H

#include "GlobalConstants.h"

BOOL EulerAnglesToQuaternion(real* EulerAngles, real* Quaternion);
BOOL EulerAnglesToR321(real* EulerAngles, real* R321);
BOOL QuatNormalize(real *Quat);
BOOL QuaternionToEulerAngles( real* EulerAngles, real* Quaternion );
BOOL QuaternionToR321(real* Quaternion, real* R321);

#endif /* QUATERNIONMATH_H */

