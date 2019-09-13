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

#define BMI_RS 1
#define MAXIM_RS 2

#ifndef NULL
#define NULL 0
#endif

#define APPLY     1
#define REMOVE    0

typedef unsigned char bool;
typedef unsigned char BOOL;

#ifndef true
#define true  1
#define false 0
#endif

#ifndef TRUE
#define TRUE  1
#define FALSE 0
#endif

#define  TWO_POW_16  65536UL

// Constants
#define  RAD_TO_DEG     57.29577951308232
#define  DEG_TO_RAD     0.017453292519943
#define D2R        ( 0.017453292519943 ) ///< ( PI/180.0 ) = 0.017453292519943
#define R2D         57.29577951308232

#define DEG2RAD(d)  ( (d) * D2R )
#define RAD2DEG(r)  ( (r) * R2D )

#define SIGMA        1.0e-8
#define KNOT2MPSEC   5.144444444e-1
#define SQUARE(x) ((x)*(x))

#define ROLL_INCIDENCE_LIMIT  0x1000
#define PITCH_INCIDENCE_LIMIT 0x1000
#define HARD_IRON_LIMIT       8192 // 0.25 G
#define SOFT_IRON_LIMIT       6554 // 20%
/// hard and soft iron resolution (2^16 / 2)
#define IRON_SCALE            32768


// For fast inverse trigonometric functions
#define  TAN15DEG  0.26794919243F
#define  TAN30DEG  0.57735026919F

// The following is the acceleration due to gravity at the calibration location
#define  GRAVITY            9.80665
#define  ACCEL_DUE_TO_GRAV  9.794259
#define  g_TO_M_SEC_SQ  9.80655

#define  MAX_ITOW  604800000    // Second in a week (assuming exactly 24 hours in a day)

// PI and related values
#define  TWO_PI        6.283185307179586
#define  PI            3.141592653589793
#define  PI_OVER_TWO   1.570796326794897
#define  PI_OVER_FOUR  0.785398163397448
#define  PI_OVER_SIX   0.523598775598299

#define  ONE_OVER_PI      0.318309886183791
#define  ONE_OVER_TWO_PI  0.159154943091895

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

#define  FAST_MATH  0

#define MIN_TO_MILLISECONDS 60000.0

/// Specify the data acquisition task rate of the system in Hz. Due to the way data is collected,
/// this is different than the sampling rate of the sensors.  Note: must be 100 or 200.
#define  DACQ_100_HZ        100
#define  DACQ_200_HZ        200
#define  DACQ_RATE_INVALID  0

/// Specify the algorithm execution frequency in Hz.
/// So far only 100 and 200 
#define  FREQ_50_HZ         50
#define  FREQ_100_HZ        100
#define  FREQ_200_HZ        200
#define  FREQ_INVALID       0

// Choices for user communication interface
#define UART_COMM       0
#define SPI_COMM        1
#define CAN_BUS         2
//#define UNVALID_COMM    100 

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


// Algorithm specifiers
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

// some value limits
#define INT16_LIMIT 32765
#define INT12_LIMIT 2045

#define USER_PACKET_OK      0
#define UNKNOWN_USER_PACKET 1
#define USER_PACKET_ERROR   2

#define USER_OK      0x00
#define USER_NAK     0x80
#define USER_INVALID 0x81

#define MAXUINT32 4294967295 	///< max unsigned 32 bit int=> ((2^32)-1)
#define MAXUINT16      65535    ///< max unsigned 16 bit int=> ((2^16)-1)
#define MAXINT16     ( 32767)   ///< max signed 16 bit int=> ((2^15)-1)
#define MININT16     (-32768)   ///< max negative signed 16 bit int=> (-(2^15))
#define MAXINT32     ( 0x7fffffff)   ///< max signed 32 bit int
#define MININT32     ( 0x80000000)   ///< max negative signed 32 bit int


#endif /* GLOBALCONSTANTS_H */

