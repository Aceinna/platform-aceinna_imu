/*
* File:   SensorNoiseParameters.h
* Author: joemotyka
*
* Created on April 10, 2016, 11:37 PM
*/

#ifndef _SENSOR_NOISE_PARAMETERS_H_
#define _SENSOR_NOISE_PARAMETERS_H_

#include "GlobalConstants.h"

// IMU spec
#define RW_ODR      100.0               /* [Hz], The data sampling rate when calculate ARW and VRW.
                                         * ARW = sigma * sqrt(dt) = sigma * sqrt(1/ODR)
                                         */
#define ARW_300ZA   8.73e-5             /* [rad/sqrt(s)], gyro angular random walk, sampled at 100Hz
                                         * 0.3deg/sqrt(Hr) = 0.3 / 60 * D2R = 8.72664625997165e-05rad/sqrt(s)
                                         */
#define BIW_300ZA   2.91e-5             /* [rad/s], gyro bias instability
                                         * 6.0deg/Hr = 6.0 / 3600 * D2R = 2.90888208665722e-05rad/s
                                         */
#define MAX_BW      8.73e-3             /* [rad/s], max possible gyro bias
                                         * 0.5deg/s = 0.5 * D2R = 0.00872664625997165rad/s
                                         */
#define VRW_300ZA   1.0e-3              /* [m/s/sqrt(s)], accel velocity random walk, sampled at 100Hz
                                         * 0.06m/s/sqrt(Hr) = 0.06 / 60 = 0.001m/s/sqrt(s)
                                         */
#define BIA_300ZA   10.0e-6 * GRAVITY   /* [m/s/s], accel bias instability
                                         * 10ug = 10.0e-6g * GRAVITY
                                         */
#define MAX_BA      3.0e-3 * GRAVITY    /* [m/s/s], max possible accel bias
                                         * 3mg = 3.0e-3g * GRAVITY
                                         */

// GNSS spec
#define R_VALS_GPS_POS_X                5.0
#define R_VALS_GPS_POS_Y                5.0
#define R_VALS_GPS_POS_Z                7.5

#define R_VALS_GPS_VEL_X                0.025
#define R_VALS_GPS_VEL_Y                0.025
#define R_VALS_GPS_VEL_Z                0.025


#endif /* _SENSOR_NOISE_PARAMETERS_H_ */