/*
 * File:   Limits.h
 * Author: joemotyka
 *
 * Created on April 10, 2016, 11:37 PM
 */

#ifndef _ALGORITHM_LIMITS_H_
#define _ALGORITHM_LIMITS_H_

#include "GlobalConstants.h"

// Declare the limits
#define LIMIT_P                     5000.0

#define LIMIT_ANG_ERROR_ROLL        0.017453292519943    // ONE_DEGREE_IN_RAD
#define LIMIT_ANG_ERROR_PITCH       0.017453292519943    // ONE_DEGREE_IN_RAD
#define LIMIT_ANG_ERROR_YAW         0.043633231299858    //(2.5 * ONE_DEGREE_IN_RAD)
#define LIMIT_YAW_RATE_SQ           0.001903858873667   // ( 2.5 [ deg/sec ] * DEG_TO_RAD )^2

#define LIMIT_BIAS_RATE_UPDATE_AHRS      5.0e-3
#define LIMIT_BIAS_RATE_UPDATE_INS       5.0e-4

#define LIMIT_GPS_VELOCITY_SQ       1.0

#define LIMIT_OBS_JACOBIAN_DENOM    1e-3;

// The following times are compared against ITOW (units in [msec])
#define LIMIT_MAX_GPS_DROP_TIME                     3    // 3 [sec]
#define LIMIT_MAX_REST_TIME_BEFORE_DROP_TO_AHRS     60000 // 60000 [ msec ] = 60 [ sec ]
#define LIMIT_DECL_EXPIRATION_TIME                  60000  // 60,000 [ counts ] = 10 [ min ]

// The following is compared against a counter (in units of the calling frequency of the EKF)
#define LIMIT_FREE_INTEGRATION_CNTR                 60   // 60 [ sec ]

#define LIMIT_QUASI_STATIC_STARTUP_RATE             0.087266462599716    // (5.0 * ONE_DEGREE_IN_RAD)

#endif /* _ALGORITHM_LIMITS_H_ */
