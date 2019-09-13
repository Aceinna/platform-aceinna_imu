/*
 * File:   QuaternionMath.cpp
 * Author: joemotyka
 *
 * Created on May 7, 2016, 5:03 PM
 */

#include "QuaternionMath.h"
#ifdef DISPLAY_DIAGNOSTIC_MSG
#include "TimingVars.h"
#endif

#include "Indices.h"
#include <math.h>

#if FAST_MATH
#include "arm_math.h"
#include "FastInvTrigFuncs.h"
#endif

BOOL EulerAnglesToQuaternion( real* EulerAngles, real* Quaternion )
{
    real sinThetaXOver2, cosThetaXOver2;  // roll
    real sinThetaYOver2, cosThetaYOver2;  // pitch
    real sinThetaZOver2, cosThetaZOver2;  // yaw

    // Divide the angle by two (the angles used in computing the quaternion
    //   are theta/2)
    real ThetaXOver2 = (real)0.5 * EulerAngles[ROLL];
    real ThetaYOver2 = (real)0.5 * EulerAngles[PITCH];
    real ThetaZOver2 = (real)0.5 * EulerAngles[YAW];

    // Precompute sin/cos values used in the expressions below
#if !FAST_MATH
    sinThetaXOver2 = sin( ThetaXOver2 );
    cosThetaXOver2 = cos( ThetaXOver2 );
    sinThetaYOver2 = sin( ThetaYOver2 );
    cosThetaYOver2 = cos( ThetaYOver2 );
    sinThetaZOver2 = sin( ThetaZOver2 );
    cosThetaZOver2 = cos( ThetaZOver2 );
#else
    sinThetaXOver2 = arm_sin_f32( ThetaXOver2 );
    cosThetaXOver2 = arm_cos_f32( ThetaXOver2 );
    sinThetaYOver2 = arm_sin_f32( ThetaYOver2 );
    cosThetaYOver2 = arm_cos_f32( ThetaYOver2 );
    sinThetaZOver2 = arm_sin_f32( ThetaZOver2 );
    cosThetaZOver2 = arm_cos_f32( ThetaZOver2 );
#endif

    // q0 = SIN( ThetaX/2 ) * SIN( ThetaY/2 ) * SIN( ThetaZ/2 ) + COS( ThetaX/2 ) * COS( ThetaY/2 ) * COS( ThetaZ/2 )
    Quaternion[Q0] = sinThetaXOver2 * sinThetaYOver2 * sinThetaZOver2 +
                     cosThetaXOver2 * cosThetaYOver2 * cosThetaZOver2;

    // q1 = SIN( ThetaX/2 ) * COS( ThetaY/2 ) * COS( ThetaZ/2 ) - COS( ThetaX/2 ) * SIN( ThetaY/2 ) * SIN( ThetaZ/2 )
    Quaternion[Q1] = sinThetaXOver2 * cosThetaYOver2 * cosThetaZOver2 -
                     cosThetaXOver2 * sinThetaYOver2 * sinThetaZOver2;

    // q2 = SIN( ThetaX/2 ) * COS( ThetaY/2 ) * SIN( ThetaZ/2 ) + COS( ThetaX/2 ) * SIN( ThetaY/2 ) * COS( ThetaZ/2 )
    Quaternion[Q2] = sinThetaXOver2 * cosThetaYOver2 * sinThetaZOver2 +
                     cosThetaXOver2 * sinThetaYOver2 * cosThetaZOver2;

    // q3 = COS( ThetaX/2 ) * COS( ThetaY/2 ) * SIN( ThetaZ/2 ) - SIN( ThetaX/2 ) * SIN( ThetaY/2 ) * COS( ThetaZ/2 )
    Quaternion[Q3] = cosThetaXOver2 * cosThetaYOver2 * sinThetaZOver2 -
                     sinThetaXOver2 * sinThetaYOver2 * cosThetaZOver2;

    QuatNormalize( Quaternion );

    return 1;
}

BOOL EulerAnglesToR321(real* EulerAngles, real* R321)
{
    real sinThetaX, cosThetaX;
    real sinThetaY, cosThetaY;
    real sinThetaZ, cosThetaZ;

    // Precompute sin/cos values used in the expressions below
#if !FAST_MATH
    sinThetaX = sin(EulerAngles[ROLL]);
    cosThetaX = cos(EulerAngles[ROLL]);
    sinThetaY = sin(EulerAngles[PITCH]);
    cosThetaY = cos(EulerAngles[PITCH]);
    sinThetaZ = sin(EulerAngles[YAW]);
    cosThetaZ = cos(EulerAngles[YAW]);
#else
    sinThetaX = arm_sin_f32(EulerAngles[ROLL]);
    cosThetaX = arm_cos_f32(EulerAngles[ROLL]);
    sinThetaY = arm_sin_f32(EulerAngles[PITCH]);
    cosThetaY = arm_cos_f32(EulerAngles[PITCH]);
    sinThetaZ = arm_sin_f32(EulerAngles[YAW]);
    cosThetaZ = arm_cos_f32(EulerAngles[YAW]);
#endif

    *(R321 + 3*X_AXIS+X_AXIS) = cosThetaZ*cosThetaY;
    *(R321 + 3*X_AXIS+Y_AXIS) = cosThetaZ*sinThetaY*sinThetaX - sinThetaZ*cosThetaX;
    *(R321 + 3*X_AXIS+Z_AXIS) = cosThetaZ*sinThetaY*cosThetaX + sinThetaZ*sinThetaX;

    *(R321 + 3*Y_AXIS+X_AXIS) = sinThetaZ*cosThetaY;
    *(R321 + 3*Y_AXIS+Y_AXIS) = sinThetaZ*sinThetaY*sinThetaX + cosThetaZ*cosThetaX;
    *(R321 + 3*Y_AXIS+Z_AXIS) = sinThetaZ*sinThetaY*cosThetaX - cosThetaZ*sinThetaX;

    *(R321 + 3*Z_AXIS+X_AXIS) = -sinThetaY;
    *(R321 + 3*Z_AXIS+Y_AXIS) = cosThetaY*sinThetaX;
    *(R321 + 3*Z_AXIS+Z_AXIS) = cosThetaY*cosThetaX;

    return 1;
}


BOOL QuatNormalize( real *Quat )
{
    real QuatSquared[4];
    real QuatMag;
    real temp;

    // Square the components of the quaternion (0 <= sSquared <= 1)
    QuatSquared[Q0] = Quat[Q0] * Quat[Q0];
    QuatSquared[Q1] = Quat[Q1] * Quat[Q1];
    QuatSquared[Q2] = Quat[Q2] * Quat[Q2];
    QuatSquared[Q3] = Quat[Q3] * Quat[Q3];

    // Find the RSS of the quaternion: sqrt( q1^2 + q2^2 + q3^2 + q4^2 )
    QuatMag = QuatSquared[Q0] +
              QuatSquared[Q1] +
              QuatSquared[Q2] +
              QuatSquared[Q3];
    QuatMag = sqrtf( QuatMag );

    // Normalize the quaternion
    temp = 1 / QuatMag;
    Quat[Q0] = Quat[Q0] * temp;
    Quat[Q1] = Quat[Q1] * temp;
    Quat[Q2] = Quat[Q2] * temp;
    Quat[Q3] = Quat[Q3] * temp;

#ifdef FORCE_Q0_POSITIVE
    // Force Q0 to be positive
    if (Quat[Q0] < 0.0) {
//#ifdef DISPLAY_DIAGNOSTIC_MSG
//        TimingVars_DiagnosticMsg("QuatNormalize: flip sign on q");
//#endif

        // Flip signs on all quaternion elements
        Quat[Q0] = -Quat[Q0];
        Quat[Q1] = -Quat[Q1];
        Quat[Q2] = -Quat[Q2];
        Quat[Q3] = -Quat[Q3];
    }
#endif

    return 1;
}


BOOL QuaternionToEulerAngles( real* EulerAngles,
                              real* Quaternion )
{
    real R[3][3];

    real q0Sq, q0q1, q0q2, q0q3;
    real q1Sq, q1q2, q1q3;
    real q2Sq, q2q3;
    real q3Sq;

    // Compute values used repeatedly in the function
    q0Sq = Quaternion[Q0] * Quaternion[Q0];
    q0q1 = Quaternion[Q0] * Quaternion[Q1];
    q0q2 = Quaternion[Q0] * Quaternion[Q2];
    q0q3 = Quaternion[Q0] * Quaternion[Q3];

    q1Sq = Quaternion[Q1] * Quaternion[Q1];
    q1q2 = Quaternion[Q1] * Quaternion[Q2];
    q1q3 = Quaternion[Q1] * Quaternion[Q3];

    q2Sq = Quaternion[Q2] * Quaternion[Q2];
    q2q3 = Quaternion[Q2] * Quaternion[Q3];

    q3Sq = Quaternion[Q3] * Quaternion[Q3];

    // Form the direction cosine matrix (DCM) from q
    R[0][0] = q0Sq + q1Sq - q2Sq - q3Sq;
//    R[0][1] = 2.0 * (q1q2 - q0q3);
//    R[0][2] = 2.0 * (q1q3 + q0q2);

    R[1][0] = (real)2.0 * (q1q2 + q0q3);
//    R[1][1] = q0Sq - q1Sq + q2Sq - q3Sq;
//    R[1][2] = 2.0 * (q2q3 - q0q1);

    R[2][0] = (real)2.0 * (q1q3 - q0q2);
    R[2][1] = (real)2.0 * (q2q3 + q0q1);
    R[2][2] = q0Sq - q1Sq - q2Sq + q3Sq;

    // Calculate the euler angles from the DCM
    // prevent NaN due to numerical error, dxg, 20180802
    if (R[2][0] > 1.0)
    {
        R[2][0] = 1.0;
    }
    if (R[2][0] < -1.0)
    {
        R[2][0] = -1.0;
    }
#if !FAST_MATH
    EulerAngles[ROLL]  = atan2( R[2][1], R[2][2] );
    EulerAngles[PITCH] = -asin( R[2][0] );
    EulerAngles[YAW]   = atan2( R[1][0], R[0][0] );
#else
    EulerAngles[ROLL]  = fatan2_rad( R[2][1], R[2][2] );
    EulerAngles[PITCH] = -fasin_rad( R[2][0] );
    EulerAngles[YAW]   = fatan2_rad( R[1][0], R[0][0] );
#endif

    // What do do in the case that pitch = 90 degrees???  Indeterminate roll and yaw...
    return 1;
}


BOOL QuaternionToR321(real* Quaternion, real* R321)
{
    real q0Sq, q0q1, q0q2, q0q3;
    real q1Sq, q1q2, q1q3;
    real q2Sq, q2q3;
    real q3Sq;

    // Compute values used repeatedly in the function
    q0Sq = Quaternion[Q0] * Quaternion[Q0];
    q0q1 = Quaternion[Q0] * Quaternion[Q1];
    q0q2 = Quaternion[Q0] * Quaternion[Q2];
    q0q3 = Quaternion[Q0] * Quaternion[Q3];

    q1Sq = Quaternion[Q1] * Quaternion[Q1];
    q1q2 = Quaternion[Q1] * Quaternion[Q2];
    q1q3 = Quaternion[Q1] * Quaternion[Q3];

    q2Sq = Quaternion[Q2] * Quaternion[Q2];
    q2q3 = Quaternion[Q2] * Quaternion[Q3];

    q3Sq = Quaternion[Q3] * Quaternion[Q3];

    //Form the direction cosine matrix (DCM) from q
    *(R321 + 0) = q0Sq + q1Sq - q2Sq - q3Sq;
    *(R321 + 1) = (real)2.0 * (q1q2 - q0q3);
    *(R321 + 2) = (real)2.0 * (q1q3 + q0q2);

    *(R321 + 3) = (real)2.0 * (q1q2 + q0q3);
    *(R321 + 4) = q0Sq - q1Sq + q2Sq - q3Sq;
    *(R321 + 5) = (real)2.0 * (q2q3 - q0q1);

    *(R321 + 6) = (real)2.0 * (q1q3 - q0q2);
    *(R321 + 7) = (real)2.0 * (q2q3 + q0q1);
    *(R321 + 8) = q0Sq - q1Sq - q2Sq + q3Sq;

    //
    return 1;
}


