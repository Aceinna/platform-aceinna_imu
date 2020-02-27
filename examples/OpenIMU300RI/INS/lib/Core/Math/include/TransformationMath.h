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

 //=============================================================================
 /******************************************************************************
 * @brief Compute unit gravity vector in the body frame from accel measurement.
 * Accelerometer measurement = -gravity when there is no linear acceleration.
 *   If acceleromter measurement is [0; 0; -1], the unit gravity vector should
 *   be [0; 0; 1].* TRACE:
 * @param [in] accel    accelerometer measurement.
 * @param [out] unitGravityVector unit gravity vector in the body frame.
 * @retval
 ******************************************************************************/
void UnitGravity(real *accel, real *unitGravityVector);

/******************************************************************************
* @brief Compute pitch and roll angle from unit gravity vector in the body frame.
* @param [in] unitGravityVector    unit gravity vector in the body frame.
* @param [out] eulerAngles Euler angles in order [roll pitch yaw].
*   roll is put in eulerAngle[0], and pitch in eulerAngle[1].
* @retval
******************************************************************************/
void UnitGravityToEulerAngles(real *unitGravityVector, real* eulerAngles);

/******************************************************************************
* @brief Compute yaw angle from unit gravity vector and magnetic measurement.
* The unit gravity vector in the body frame is used to project the magnetic
* measurement onto a perpendicular frame. The projected x and y component of
* the magnetic measurement in this perpendicular frame are used to compute the
* yaw angle.
* TRACE:
* @param [in] unitGravityVector unit gravity vector in the body frame.
* @param [in] magFieldVector    magnetic vector in the body frame.
* @retval yaw angle in [rad].
******************************************************************************/
real UnitGravityAndMagToYaw(real *unitGravityVector, real *magFieldVector);

/******************************************************************************
* @brief Compute yaw angle from pitch, roll and magnetic measurement.
* The pitch and roll angles are used to project the magneticmeasurement onto a
* perpendicular frame. The projected x and y component of the magnetic measurement
* in this perpendicular frame are used to compute the yaw angle.
* TRACE:
* @param [in] roll  roll angle in [rad].
* @param [in] pitch pitch angle in [rad].
* @param [in] magFieldVector    magnetic vector in the body frame.
* @retval yaw angle in [rad].
******************************************************************************/
real RollPitchAndMagToYaw(real roll, real pitch, real *magFieldVector);

/******************************************************************************
* @brief Limit angle error to be [-180, 180].
* TRACE:
* @param [in] aErr in [deg].
* @retval aErr within [-180, 180]deg.
******************************************************************************/
real AngleErrDeg(real aErr);

/******************************************************************************
* @brief Limit angle error to be [-PI, PI].
* TRACE:
* @param [in] aErr in [rad].
* @retval aErr within [-2*PI, 2*PI].
******************************************************************************/
real AngleErrRad(real aErr);
//
BOOL LLA_To_R_EinN( double* llaRad, real* R_EinN);
BOOL LLA_To_R_NinE( double* llaRad, real* R_NinE);
BOOL ECEF_To_Base( double* rECEF_Init,
                  double* rECEF ,
                  real* R_NinE, 
                  real* dr_N);
BOOL LLA_To_ECEF( double* lla_Rad, double* ecef_m);
BOOL PosNED_To_PosECEF( real*  r_N, double* rECEF_Init,
                        real* R_NinE, double* rECEF );
BOOL ECEF_To_LLA( double* llaDeg, double* ecef_m );

BOOL VelECEF_To_VelNED( double* LLA, real* VelECEF, real* VelNED );


void printMtx(float *a, int m, int n);
void printVec(float *v, int n);

#endif /* TRANSFORMATIONMATH_H */



