/** ***************************************************************************
 * @file   dmu.h  Configuration and calibration data structure
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
#ifndef DMU_H
#define DMU_H
#include "GlobalConstants.h"
#include <stdint.h>

// These are in IQMath.h
typedef float iq27;
typedef float iq30;
typedef float iq23;
typedef float iq29;

#define IQ27(x) (x)


#ifndef NULL
#define NULL 0
#endif

// bias for gyro, accel and mag
#define APPLY     1
#define REMOVE    0


enum eGainOrder {
     ACCEL_GAIN       = 0,
     GYRO_GAIN        = 1,
     MAG_GAIN         = 2,
     TEMP_GAIN        = 3,
     NUM_GAIN_ENTRIES = 4
} ;

#define MAXUINT32 4294967295 	///< max unsigned 32 bit int=> ((2^32)-1)
#define MAXUINT16      65535    ///< max unsigned 16 bit int=> ((2^16)-1)
#define MAXINT16     ( 32767)   ///< max signed 16 bit int=> ((2^15)-1)
#define MININT16     (-32768)   ///< max negative signed 16 bit int=> (-(2^15))

/// Conversion functions to go from XBOW to 380 formats
//#define CONVERT_XBOW_TO_380(x) (((x) >> 16) + (int32_t) MININT16) // going to signed
#define CONVERT_XBOW_TO_380(x) ( (int32_t)( (int32_t)( (x) >> 16 ) +  MININT16 ) ) ///< going to signed int 32
#define CONVERT_380_TO_XBOW(x) (( (uint32_t)( (x) - MININT16 ) ) << 16 ) ///< going to unsigned int 32

// The ODR of the Rate-Sensor is set to 10 kHz so the latency in the data
//   is minimal (this was added to comply with the updated method of
//   operation of the Maxim rate-sensor).  The ODR of the accelerometer
//   is set to 800 Hz to minimize aliasing and enable filtering using a
//   digital 50 Hz LPF.
#define  RATE_SENSOR_ODR  10000 ///< ASAP
#define  ACCEL_ODR        400
#define  TIM5_OUTPUT_DATA_RATE   800

extern void DataAquisitionStart(void);
extern void DataAquisitionStop(void);
void        InitDataAcquisitionTimer(uint32_t outputDataRate);

#define kick_dog()

#endif /* DMU_H */