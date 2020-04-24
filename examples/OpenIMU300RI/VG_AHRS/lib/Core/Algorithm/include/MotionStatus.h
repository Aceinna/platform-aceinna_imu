/*****************************************************************************
 * @file    MotionStatus.h
 * @brief   Calculate sensor stats, and detect motion status using IMU/ODO/GNSS
 * @author  Dong Xiaoguang
 * @version 1.0
 * @date    20190801
 *****************************************************************************/

#ifndef MOTION_STATUS_H_INCLUDED
#define MOTION_STATUS_H_INCLUDED

#include <stdint.h>
#include "GlobalConstants.h"
#include "algorithm.h"


typedef struct
{
    BOOL bValid;        // tell if stats are valid
    BOOL bStaticIMU;    // Static period detected by IMU
    BOOL accelErrLimit; // accelErr is set to max/min limit
    real lpfAccel[3];   // [m/s/s], low-pass filtered accel
    real accelNorm;     // [m/s/s], magnitude of current accel
    real accelMean[3];  // [m/s/s], average of past n accel samples
    real accelVar[3];   // [m/s/s]^2, variance of past n accel samples
    real accelErr[3];   // [m/s/s], estimated accel error
    real lpfGyro[3];    // [rad/s], low-pass filtered gyro
    real gyroMean[3];   // [rad/s], average of past n gyro samples
    real gyroVar[3];    // [rad/s]^2, variance of past n gyro samples
} ImuStatsStruct;

/******************************************************************************
 * @brief Calculate IMU data stats, and detect zero velocity.
 * TRACE:
 * @param [in]  gyro        [rad/s]
 * @param [in]  accel       [m/s/s]
 * @param [in]  reset       TRUE to reset this process
 * @param [Out] imuStats    results
 * @retval None.
******************************************************************************/
void MotionStatusImu(real *gyro, real *accel, ImuStatsStruct *imuStats, BOOL reset);

/******************************************************************************
 * @brief Using gyro propagation to estimate accel error.
 * g_dot = -cross(w, g), g is gravity and w is angular rate.
 * TRACE:
 * @param [in] accel            Input accel, m/s/s.
 * @param [in] w                Input angular rate, rad/s.
 * @param [in] dt               Sampling interval, sec.
 * @param [in] staticDelay      A Counter. When static period detected, delay [staticDelay] samples before
 *                              lowering accel error. [staticDelay] is also used to reset initial accel that
 *                              is propagated using gyro to estimate future accel.
 * @param [out] gAccelStats     A struct for results storage.
 * @retval None.
******************************************************************************/
void EstimateAccelError(real *accel, real *w, real dt, uint32_t staticDelay, ImuStatsStruct *gAccelStats);

/******************************************************************************
 * @brief Detect motion according to the difference between measured accel magnitude and 1g.
 * Set gAlgorithm.linAccelSwitch to be True if being static for a while.
 * This result is no longer used in current algorithm.
 * TRACE:
 * @param [in] accelNorm Input accel magnitude, g.
 * @param [in] iReset   Reset the procedure.
 * @retval Always true.
******************************************************************************/
BOOL DetectMotionFromAccel(real accelNorm, int iReset);

/******************************************************************************
 * @brief Detect zero velocity using GNSS speed.
 * TRACE:
 * @param [in]  vNED        NED velocity measured by GNSS   [m/s]
 * @param [in]  gnssValid   Indicate if GNSS measurement is valid.
 *                          If valid, vNED will be used to detect zero velocity.
 *                          If not, detection will be reset and FALSE is always
 *                          returned.
 * @retval TRUE if static, otherwise FALSE.
******************************************************************************/
BOOL DetectStaticGnssVelocity(double *vNED, real threshold, BOOL gnssValid);

/******************************************************************************
 * @brief Detect zero velocity using odometer data.
 * TRACE:
 * @param [in]  odo velocity measured by odometer   [m/s]
 * @retval TRUE if static, otherwise FALSE.
******************************************************************************/
BOOL DetectStaticOdo(real odo);

#endif // MOTION_STATUS_H_INCLUDED

