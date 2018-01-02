/*
 * File:   GlobalConstants.h
 * Author: joemotyka
 *
 * Created on April 10, 2016, 1:05 AM
 */

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

// physical constants
#define  GRAVITY            9.80665
#define  PI                 3.1415926535897932385
#define  TWO_PI             6.28318530717959

#define ONE_DEGREE_IN_RAD     (0.017453292519943)  //  1 degree -> radians
#define TWO_DEGREES_IN_RAD    (0.034906585039887)  //  2 degree -> radians
#define THREE_DEGREES_IN_RAD  (0.052359877559830)  //  3 degree -> radians
#define FIVE_DEGREES_IN_RAD   (0.087266462599716)  //  5 degree -> radians
#define TEN_DEGREES_IN_RAD    (0.17453292519943)
#define TWENTY_DEGREES_IN_RAD (0.349065850398866)
#define THREE_HUNDRED_EIGHTY_DEGREES_IN_RAD  (6.632251157578453)  // 380 degrees -> radians
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
#define TWO_POINT_FIVE_DEGREE_Q29 23425413
#define TWO_DEGREES_Q29           18740330
#define ONE_DEGREE_Q29             9370165
#define ONE_HALF_DEGREE_Q29        4685082
#define THREE_EIGTHS_DEGREE_Q29    3513553
#define ONE_QUARTER_DEGREE_Q29     2342541

///   where Dt = 1/200 [ sec ] = 0.005 = 5368709 (Q30)
///         ( Dt / 2 ) = 2684355 (Q30)
#define  DT_OVER_TWO_Q30  2684355
#define  DT_Q30           5368709

// Initialize the time parameters used in the EKF
#define DT              0.01     // (1.0 / 100.0)
#define DT_OVER_TWO     0.005    // (0.5 * DT)
#define DT_SQUARED      0.0001   // (DT * DT)

/// Specify the output data rate of the system in Hz. Due to the way data is collected,
/// this is different than the ODR of the sensors.  Note: must be 100 or 200.
#define  ODR_100_HZ        100
#define  ODR_200_HZ        200

//
#define  IMU   0
#define  AHRS  1
#define  VG    2
#define  INS   3

#ifdef INS_OFFLINE
//#define USE_DOUBLE
#endif

#ifdef USE_DOUBLE
#define  real  double
#else
#define real  float
#endif

// Force Q0 positive to minimize the errors
#define FORCE_Q0_POSITIVE

// Leave this in to use euler angles in the update stage, else use quaternions
#define EULER_ANGLE_SOLN

// one stop shop to configure parameters for the simulation
#define DRIVE_TEST     // used to select TST value (0.5 or 10.0 deg/sec)

// Add a identifier that tells the compiler which dataset is being simulated (used
//   to specify the proper mag-align parameters)
//#define MAX_DPS_400
//#define DRIVE_TEST_20160723
//#define DRIVE_TEST_20160508
//#define DRIVE_TEST_20161029
//#define FLIGHT_TEST_20160512

// #define MAX_DPS_400  // May need to use this for the pc-based simulation
#define DRIVE_TEST_20161118
//#define FLIGHT_TEST_

#define MAX_NUM_OF_LINES  1000000

#ifdef INS_OFFLINE
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

#define  MAXIM_RS  0
#define  BMI_RS    1

#endif /* GLOBALCONSTANTS_H */


