/*
* File:   SensorNoiseParameters.h
* Author: joemotyka
*
* Created on April 10, 2016, 11:37 PM
*/

#ifndef _SENSOR_NOISE_PARAMETERS_H_
#define _SENSOR_NOISE_PARAMETERS_H_

#include "GlobalConstants.h"

    // Add 25% uncertainty to the noise parameter
#define SENSOR_NOISE_RATE_STD_DEV       (1.25 * 1e-1 * D2R)
#define SENSOR_NOISE_ACCEL_STD_DEV      1.25 * 7.0e-4 * GRAVITY
#define SENSOR_NOISE_MAG_STD_DEV        0.00  // todo tm20160603 - ?check this value ?

#define R_VALS_GPS_TRACK                0.1

#define R_VALS_GPS_POS_X                5.0
#define R_VALS_GPS_POS_Y                5.0
#define R_VALS_GPS_POS_Z                7.5

#define R_VALS_GPS_VEL_X                0.025
#define R_VALS_GPS_VEL_Y                0.025
#define R_VALS_GPS_VEL_Z                0.025


#endif /* _SENSOR_NOISE_PARAMETERS_H_ */