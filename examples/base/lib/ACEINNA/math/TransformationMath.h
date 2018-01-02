/*
 * File:   TransformationMath.h
 * Author: joemotyka
 *
 * Created on May 8, 2016, 12:35 AM
 */

#ifndef TRANSFORMATIONMATH_H
#define TRANSFORMATIONMATH_H

#include "GlobalConstants.h"
#include <stdint.h>

#define  E_MAJOR     6.378137e+6             // semi major axis a
#define  E_MINOR     6.356752314245216e+6    // semi minor axis b
#define  E_ECC       0.08181919084255        // first eccentricity
#define  E_ECC_SQ    0.006694379990130       // (first eccentricity)^2
#define  E_MAJOR_SQ  4.068063159076900e+013  // E_MAJOR^2
#define  E_MINOR_SQ  4.040829998466191e+013  // E_MINOR2
#define  EP_SQ       4.284131151324081e+004  // (E_MAJOR2 - E_MINOR2)/E_MINOR

#define  E_MINOR_OVER_MAJOR_SQ  0.993305620009870   // E_MAJOR_SQ / E_MINOR_SQ
#define  E_MAJOR_OVER_MINOR_SQ  1.006739496742265   // E_MINOR_SQ / E_MAJOR_SQ

#define  E_MAJOR_OVER_MINOR  1.003364089820971   // E_MAJOR / E_MINOR

#define  E_ECC_SQxE_MAJOR  42697.67270710779

int      FieldVectorsToEulerAngles( real* GravityVector,
                                real* MagFieldVector,
                                uint8_t usePredFlag,
                                real* EulerAngles );

//
int      LLA_To_R_EinN( double* llaRad, real* R_EinN);
int      LLA_To_R_NinE( double* llaRad, real* R_NinE);
int      LLA_To_Base( double* llaRad, double* rECEF_Init,
                  real* dr_N,
                  real* R_NinE, double* rECEF );
int      LLA_To_ECEF( double* lla_Rad, double* ecef_m);
int      PosNED_To_PosECEF( real*  r_N, double* rECEF_Init,
                        real* R_NinE, double* rECEF );
int      ECEF_To_LLA( double* llaDeg, double* ecef_m );

int      VelECEF_To_VelNED( double* LLA, real* VelECEF, real* VelNED );

#endif /* TRANSFORMATIONMATH_H */



