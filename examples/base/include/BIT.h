 /** ***************************************************************************
 * @file BIT.h DMU380 header file for sensor filtering, mapping, and scaling
 * @brief Sensor data manipulation functions and collection from
 *       internal and external ADC. Analog hardware and logic for Built-In-Test
 *       functions.
 * @Author Darren Liccardo
 * @date   Oct. 2005
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * @brief Sensor data, ordering, and filtering is based on the
 *        productConfiguration.architecture field.
 * 06.19.2006 DRA bug 1216: changed lower limit of pcbTemp for
 *				  architectures 3 and 4 to 0. was 0xFFFF. reported
 *				  by BEELINE
 * 07.26.2006 DRA bug 1234: changed lower and upper limits on 1.9V
 *				  bus to 1.8V and 2V. (for architectures 3 and 4)
 * 07.01.2014 DKH Ported to DMU380 hardware
 *****************************************************************************/
#ifndef _BIT_H_
#define _BIT_H_

#include <stdint.h>
#include "dmu.h"

// the mapping of low-pass temperature filter coefficients to inertial sensors
// for each architecture
static const int TEMP_FILTER[4][6] = {
//AX,AY,AZ,RX,RY,RZ
{ 1,  1,  1, 12, 12, 12},   //STANDARD
{12, 12, 12, 14, 14, 14},   //INTERNATIONAL
{ 4,  4,  4,  1,  1,  1},   //VG325
{ 4,  4,  4,  1,  1,  1}};  //VG327

// the mapping of temperature sensor scale factor to temp sensors for each
// architecture used in converting temperatures to deg C
static const double TEMP_SCALE[4][6] = {
//temp for AX,AY,AZ,RX,RY,RZ
{-3.956341e-7, -3.956341e-7, -3.956341e-7, -3.956341e-7, -3.956341e-7, -3.956341e-7},   //STANDARD
{ 4.656613e-7,  4.656613e-7,  4.656613e-7,  4.656613e-7,  4.656613e-7,  4.656613e-7},   //INTERNATIONAL
{ 4.852657e-5,  4.852657e-5,  4.852657e-5,  4.852657e-5,  4.852657e-5,  4.852657e-5},   //VG325
{ 4.852657e-5,  4.852657e-5,  4.852657e-5,  4.852657e-5,  4.852657e-5,  4.852657e-5}};  //VG327


// the mapping of temperature sensor bias to temp sensors for each
// architecture used in converting temperatures to deg C
static const long TEMP_BIAS[4][6] = {
//temp for AX,AY,AZ,RX,RY,RZ
{401021096, 401021096, 401021096, 401021096, 401021096, 401021096},   //STANDARD
{107374182, 107374182, 107374182, 107374182, 107374182, 107374182},   //INTERNATIONAL
{  5621025,   5621025,   5621025,   5621025,   5621025,   5621025},   //VG325
{  5621025,   5621025,   5621025,   5621025,   5621025,   5621025}};  //VG327

// Defines the limit above which the overrange bit is set
static const iq27 SENSOR_RANGE_Q27[3][2] = {
/// ACCELS (g), RATES (rad/sec)
{IQ27(4.0), IQ27( 3.50)},     //low range (200 dps)
{IQ27(8.0), IQ27( 6.98)},     //high range (400 dps)
{IQ27(8.0), IQ27(10.47)} };   //high range (1000 dps)

// Define the hard-limits on accelerometer and rate-sensor output.  The sensors will put out higher
//   values but the 380 will not report values higher than the limits.
static const int32_t SENSOR_LIMIT_q27[3][2] = {
/// ACCELS (g), RATES (rad/sec)
{ 536870912,   527071785},   //low range (4.0 g & 225 deg/sec)
{1342177280,   995580039},   //high range (10.0 g & 425 deg/sec)
{1342177280,  1464088293} }; //high range (10.0 g & 625 deg/sec)

#define OVER_RANGE_COUNT_LIMIT      10
#define QUASI_STATIC_OVER_RANGE_RATE 0.0349  ///<(rad/sec)  set at 2 deg/sec
#define QUASI_STATIC_STARTUP_RATE    0.06981 ///< (rad/sec) set at 4 deg/sec

#define PCBTEMP_BIAS 6554          //bias counts on BIT PCB temperature
#define PCBTEMP_SCALE   0.00762939 //scale factor on BIT PCB temperature

/// functions defined in bitManager.c
void handleBITandStatus(void);
void handleOverRange(void);
void setMagAlignOutOfBoundsBIT(void);
void setLevelingOutofBoundBIT(void);
uint16_t getTopERROR(void);
// may write new versions of these
void lowPass(uint32_t *out, uint32_t input, int resolution, int shiftCoef);
//void lowPass(uint16_t *out, uint32_t input, int resolution, int shiftCoef);
void lowPass2Pole(int32_t *out, int32_t *lastInput, int32_t input, int resolution, int shiftCoef, int shiftCoefLast);

#endif // _BIT_H_

