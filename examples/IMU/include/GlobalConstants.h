/*******************************************************************************
 * File:   GlobalConstants.h
 *******************************************************************************/
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


#ifndef GLOBALCONSTANTS_H
#define GLOBALCONSTANTS_H

#ifndef INS_OFFLINE
    typedef int BOOL;

    #ifndef true
    #define true  1
    #define false 0
    #endif
#else
    typedef bool BOOL;
#endif

#ifndef TRUE
#define TRUE  1
#define FALSE 0
#endif

#define  TWO_POW_16  65536UL

// Constants
#define D2R        ( 0.017453292519943 ) ///< ( PI/180.0 ) = 0.017453292519943
#define DEG2RAD(d) ( (d) * D2R )
#define R2D         57.29577951308232
#define RAD2DEG     ( (r) * R2D )
#define SIGMA        1.0e-8
#define KNOT2MPSEC   5.144444444e-1
#define SQUARE(x) ((x)*(x))

#define  RAD_TO_DEG     57.29577951308232
#define  DEG_TO_RAD     0.017453292519943
#define  g_TO_M_SEC_SQ  9.80655

// physical constants
#define  GRAVITY            9.80665
#define  PI                 3.1415926535897932385
#define  TWO_PI             6.28318530717959

// Specify constants used to limit variables in the algorithm
#define ONE_DEGREE_IN_RAD     (0.017453292519943)
#define TWO_DEGREES_IN_RAD    (0.034906585039887)
#define TWO_PT_FIVE_DEGREES_IN_RAD    (0.043633231299858)
#define THREE_DEGREES_IN_RAD  (0.052359877559830)
#define FIVE_DEGREES_IN_RAD   (0.087266462599716)
#define SIX_DEGREES_IN_RAD    (0.104719755119660)
#define TEN_DEGREES_IN_RAD    (0.17453292519943)
#define TWENTY_DEGREES_IN_RAD (0.349065850398866)
#define THREE_HUNDRED_EIGHTY_DEGREES_IN_RAD  (6.632251157578453)

#define ONE_OVER_PI        0.318309886183791
#define ONE_OVER_TWO_PI    0.159154943091895

#define MIN_TO_MILLISECONDS 60000.0

//  10.0 degs = 93701651
//   5.0 degs = 46850825 5 deg -> rad[Q29]
//   4.0 degs = 37480660
//   3.0 degs = 28110495
//   2.5 degs = 23425413   <-- AHRS440 Yaw limit
//   2.0 degs   = 18740330
//   1.0 degs   = 9370165   <-- AHRS440 Roll/Pitch limit
//   0.5 degs   = 4685082
//   0.375 degs = 3513553
//   0.25  degs = 2342541
//   0.1   degs =  937017
// Roll/pitch/yaw limit values (converted to radians and expressed in Q29 format)
//#define TWO_POINT_FIVE_DEGREE_Q29 23425413
//#define TWO_DEGREES_Q29           18740330
#define ONE_DEGREE_Q29             9370165
//#define ONE_HALF_DEGREE_Q29        4685082
//#define THREE_EIGTHS_DEGREE_Q29    3513553
//#define ONE_QUARTER_DEGREE_Q29     2342541

/// Specify the data acquisition task rate of the system in Hz. Due to the way data is collected,
/// this is different than the sampling rate of the sensors.  Note: must be 100 or 200.
#define  DACQ_100_HZ        100
#define  DACQ_200_HZ        200
#define  DACQ_RATE_INVALID  0

/// Specify the algorithm execution frequency in Hz.
/// So far only 100 and 200 
#define  FREQ_100_HZ        100
#define  FREQ_200_HZ        200
#define  FREQ_INVALID       0

// Choices for user communication interface
#define UART_COMM       0
#define SPI_COMM        1
#define CAN_BUS         2
#define UNVALID_COMM    100 

// Choices for sensors range
#define _200_DPS_RANGE   0
#define _400_DPS_RANGE   1
#define _1000_DPS_RANGE  2

// Choices for user system type
#define IMU_6DOF_SYS      0     //         IMU 6 degrees of freedom - Only accelerometers and gyros
#define IMU_9DOF_SYS      1     // default IMU 9 degrees of freedom - Accelerometers, gyros, magnetometer
#define UNAIDED_VG_SYS    2
#define UNAIDED_AHRS_SYS  3
#define AIDED_VG_SYS      4
#define AIDED_AHRS_SYS    5  
#define INS_SYS           6

// Choices for GPS protocol type
typedef enum{
    AUTODETECT              = -1,
	UBLOX_BINARY            =  0,
	NOVATEL_BINARY          =  1,
	NOVATEL_ASCII           =  2,
	NMEA_TEXT               =  3,
    DEFAULT_SEARCH_PROTOCOL =  NMEA_TEXT, // 3
	SIRF_BINARY             =  4,
	INIT_SEARCH_PROTOCOL    =  SIRF_BINARY, ///< 4 max value, goes through each until we hit AUTODETECT
	UNKNOWN                 = 0xFF
} enumGPSProtocol;



//
#define  IMU   0
#define  AHRS  1
#define  VG    2
#define  INS   3

//#define USE_DOUBLE
#ifdef USE_DOUBLE
#define  real  double
#else
#define real  float
#endif

// Force Q0 positive by flipping the sign on all quaternion-elements when q0 < 0.  This
//   seems to reduce the errors in the system although (in theory) it shouldn't affect
//   the result.
#define FORCE_Q0_POSITIVE

#ifdef INS_OFFLINE
    #define MAX_NUM_OF_LINES  1500000
    #define DISPLAY_DIAGNOSTIC_MSG
#endif

// Profiling during algorithm development - todo tm20160608 - remove when done
//#define RUN_PROFILING
#ifdef RUN_PROFILING
#include <stdint.h>
extern uint32_t gEkfElapsedTime;
extern uint32_t gEkfMaxTime;
extern float gEkfAvgTime;
#endif

#endif /* GLOBALCONSTANTS_H */


