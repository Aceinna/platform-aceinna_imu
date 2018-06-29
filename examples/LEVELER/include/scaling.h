/** ***************************************************************************
 * @file   scaling.h
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * macros to define various power of 2, fixed point and constant scalsing
 ******************************************************************************/
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

#ifndef SCALING_H
#define SCALING_H


///  pre-computed values
#define SCALE_BY_8(value)				( (value) * 8.0 )
#define SCALE_BY_2POW16_OVER_2PI(value)	( (value) * 10430.37835047045 ) // 65536 / (2.0 * PI)
#define SCALE_BY_2POW16_OVER_7PI(value) ( (value) * 2980.108100134415 ) // 65536 / (7.0 * PI)
#define SCALE_BY_2POW16_OVER_2(value)   ( (value) * 32768.0 ) // 65536 / 2
#define SCALE_BY_2POW16_OVER_16(value)  ( (value) * 4096.0 ) // 5536 / 16
#define SCALE_BY_2POW16_OVER_20(value)  ( (value) * 3276.8 ) // 65536 / 20
#define SCALE_BY_2POW16_OVER_64(value)  ( (value) * 1024.0 ) // 65536 / 64
#define SCALE_BY_2POW16_OVER_128(value) ( (value) * 512.0 ) // 65536 / 128
#define SCALE_BY_2POW16_OVER_200(value) ( (value) * 327.68 ) // 65536 / 200
#define SCALE_BY_2POW16_OVER_512(value) ( (value) * 128.0 ) // 65536 / 512

#define SCALE_BY_2POW32_OVER_2PI(value)    ( (value) * 683565287.23678304) // 4294967296 / 2 * PI
#define SCALE_BY_2POW16_OVER_2POW14(value) ( (value) * 4.0 ) // 65536 / 16384

#define TWO_OVER_TWO_POW16_q30       32768
#define TWENTY_OVER_TWO_POW16_q30    327680
#define TWOPI_OVER_MAXINT16_q30      205894

#define TWO_POW16_OVER_7PI_q19    1562434916   // floor( 2^16/( 7*pi ) * 2^19 )
#define TWO_POW16_OVER_20_q19     1717986918   // floor( 2^16/20 * 2^19 )
#define TWO_POW16_OVER_2_q15      1073741824   // floor( 2^16/2 * 2^15 )
#define TWO_POW16_OVER_200_q22    1374389534   // floor( 2^16/200 * 2^22 )
//#define TWO_POW16_OVER_20_q19    1717986918    // floor( 2^16/20 * 2^19 )
#define TWO_POW16_TIMES_100_OVER_7PI_q12 298011 // floor( 2^16/( 7*pi ) * 2^12 )

#define TWO_POW16_OVER_128_q21 1073741824   // floor( 2^16/200 * 2^22 )
#define TWO_POW16_OVER_512_q23 128

#define TWO_POW16_OVER_2PI_q17 1367130551
#define TWO_POW19_OVER_7PI_q16 23841

#define MAXUINT16_OVER_2PI 10430.21919552736
#define DEGREES_TO_RADS        0.017453292519943
#define RADS_TO_DEGREES       57.29577951308232
#define ITAR_RATE_LIMIT        7.15585 // 410 dps * DEGREES_TO_RADS

#define MAXINT16_OVER_2PI   5215.030020292134 //( MAXINT16 / TWOPI)
#define MAXUINT16_OVER_512   127.9980468750000 // ( MAXUINT16 / 512.0)
#define MAXUINT16_OVER_2   32768.0 //( MAXUINT16 / 2.0)

// For magCal()
#define MAXINT16_OVER_2PI_q18  1367088830

#define MAXINT32_20BIT_OVER131072M  8 // 2^20/(2^17)

/// INT32_TO_MISALIGN_SCALING = 1/2^( 32 - 5 ) as per the definition of misalign
///    in DMU Serial Interface Spec
#define INT32_TO_MISALIGN_SCALING 7.450580596923828e-09

/// BOARD_TEMP_SCALE_FACTOR = 1/256 = 0.00390625 from the TMP102 datasheet
///  ( shift 4-bits due to buffer and 4-bits for scaling)
#define BOARD_TEMP_SCALE_FACTOR 0.00390625
#define BOARD_TEMP_SCALE_FACTOR_q30  419430

/// GYRO_TEMP_SCALE_FACTOR = 1/256 = 0.00390625 from the Maxim21000 datasheet
#define MAXIM21000_TEMP_SCALE_FACTOR  0.00390625
#define BMI160_TEMP_SCALE_FACTOR  0.001953125
#define BMI160_TEMP_OFFSET        23.0
#endif

