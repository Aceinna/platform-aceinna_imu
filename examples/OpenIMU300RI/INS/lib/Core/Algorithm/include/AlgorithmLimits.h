/***************************************************************************
 * File:   AlgorithmLimits.h
 ***************************************************************************/
/*******************************************************************************
Copyright 2018 ACEINNA, INC

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*******************************************************************************/


#ifndef _ALGORITHM_LIMITS_H_
#define _ALGORITHM_LIMITS_H_

#include "GlobalConstants.h"

#define  INIT_P_Q    1.0e-5;
#define  INIT_P_WB   1.0e-5;
#define  INIT_P_INS  1.0e-3;

// Declare the limits
#define LIMIT_P                     500.0

#define LIMIT_ANG_ERROR_ROLL        0.017453292519943    // ONE_DEGREE_IN_RAD
#define LIMIT_ANG_ERROR_PITCH       0.017453292519943    // ONE_DEGREE_IN_RAD
#define LIMIT_ANG_ERROR_YAW         0.043633231299858    //(2.5 * ONE_DEGREE_IN_RAD)
#define LIMIT_YAW_RATE_SQ           0.001903858873667   // ( 2.5 [ deg/sec ] * DEG_TO_RAD )^2

#define LIMIT_BIAS_RATE_UPDATE_AHRS      5.0e-3
#define LIMIT_BIAS_RATE_UPDATE_INS       5.0e-4

#define LIMIT_MIN_GPS_VELOCITY_HEADING  0.45        //0.45 m/s ~= 1.0 mph
#define RELIABLE_GPS_VELOCITY_HEADING   1.0         // velocity of 1.0m/s should provide reliable GNSS heading

#define LIMIT_OBS_JACOBIAN_DENOM    1e-3;

// The following times are compared against ITOW (units in [msec])
#define LIMIT_MAX_GPS_DROP_TIME                     300      // [sec]
#define LIMIT_RELIABLE_DR_TIME                      10      // [sec]
#define LIMIT_MAX_REST_TIME_BEFORE_DROP_TO_AHRS     60000   // 60000 [ msec ] = 60 [ sec ]
#define LIMIT_MAX_REST_TIME_BEFORE_HEADING_INVALID  120000  // 120sec, heading drifts much slower than pos
#define LIMIT_DECL_EXPIRATION_TIME                  60000  // 60,000 [ counts ] = 10 [ min ]

// The following is compared against a counter (in units of the calling frequency of the EKF)
#define LIMIT_FREE_INTEGRATION_CNTR                 60   // 60 [ sec ]

#define LIMIT_QUASI_STATIC_STARTUP_RATE             0.087266462599716    // (5.0 * ONE_DEGREE_IN_RAD)

#endif /* _ALGORITHM_LIMITS_H_ */
