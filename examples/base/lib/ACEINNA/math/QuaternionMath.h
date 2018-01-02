/* 
 * File:   QuaternionMath.h
 * Author: joemotyka
 *
 * Created on May 7, 2016, 5:03 PM
 */

#ifndef QUATERNIONMATH_H
#define QUATERNIONMATH_H

#include "GlobalConstants.h"

int EulerAnglesToQuaternion(real* EulerAngles, real* Quaternion);
int EulerAnglesToR321(real* EulerAngles, real* R321);
int QuatNormalize(real *Quat);
int QuaternionToEulerAngles( real* EulerAngles, real* Quaternion );
int QuaternionToR321(real* Quaternion, real* R321);

#endif /* QUATERNIONMATH_H */

