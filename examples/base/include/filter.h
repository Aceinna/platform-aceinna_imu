/** ***************************************************************************
 * @file filter.h iir, fir, moving average, debounce digital filters
 * @Author
 * @date   September, 2008
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *****************************************************************************/
#ifndef _FILTER_H_
#define _FILTER_H_
#include <stdint.h>

/** ***************************************************************************
 * @name Butterworth_Q27_Filter
 * implementation of a direct form 2nd order Butterworth IIR Filter
 * y = filtered data + delayed data
 * x = raw data (scaledSensors) + delay data
 *
 * Second-order low-pass Butterworth filter
 * Difference equation:
 * b1*y(k) = g*( a1*x(k) + a2*x(k-1) + a3*x(k-2) ) - (b2*y(k-1) + b3*y(k-2) )
 * Second order 50 Hz LP Buttterworth filter coefficients based on a 200 hz
 * sampling rate
 * a = [a1 a2 a3] = [1 2 1]
 * b = [b1 b2 b3] = [1 -1.561018075800718 0.641351538057563]
 * Q27 b = [b1 b2 b3] = [134217728  -209516300    86080746]
 * g = 0.020083365564211
 * Q27 g =  2695544
 * @author Douglas K. Hiranaka
 * Memsic
 * @date April 22, 2014
 * @rev 1.0 (4/22/14):  Orignal
 * 4/22/14 DKH. Original
 * @param [in] coefficients - data struct with the order of the filter and the
 *        taps Q27 x *2^27 to scale to Q27
 * @param [in]  x - pointer to the raw data + delay samples
 * @param [out] y - filtered and delayed filter data Q27 y >> 27 scaled back
 *                  input raw count scaling
 * @retval always 0 - uses a return value to force system to wait for fcn
 *         complete before continuing.
 *****************************************************************************/

/// Digital Low pass filter types
enum eFilterType {
      UNFILTERED          = 0x00,
      FIR_40HZ_LPF        = 0x03,    // Bartlett
      FIR_20HZ_LPF        = 0x04,
      FIR_10HZ_LPF        = 0x05,
      FIR_05HZ_LPF        = 0x06,
      IIR_50HZ_LPF        = 0x30, // Butterworth
      IIR_20HZ_LPF        = 0x40,
      IIR_10HZ_LPF        = 0x50,
      IIR_05HZ_LPF        = 0x60,
      IIR_02HZ_LPF        = 0x70,
      IIR_25HZ_LPF        = 0x80,
      IIR_40HZ_LPF        = 0x90
};


typedef struct {
	int32_t  *a; // numerator coeffiecients
	int32_t  *b; // denominator coefficients
	int32_t  g;    // gain coefficient
	uint32_t N;    // order of the filter
} butterworth_fixed;
// @brief the implementation needs delay buffers:
//	int32_t  x[NUM_SENSOR_READINGS][3]; // raw + delay data
//	int32_t  y[NUM_SENSOR_READINGS][3]; // filtered + delay data

// push a value on the raw data (x) queue
void Butterworth_Q27_PushSample(int32_t *x, int32_t sample);
uint32_t Butterworth_Q27_Filter(butterworth_fixed *coefficients,
                                int32_t           *x,
                                int32_t           *y);
uint32_t Apply_Butterworth_Q27_Filter(butterworth_fixed *coefficients,
                                      uint8_t           sensor);

typedef struct {
	int32_t *taps; // filter coeffiecients
	uint32_t N;     // order of the filter
} bartlett_fixed;

// FIXME: These could be collected together in a data structure with an api to get
// the pointers instead of being global.
extern butterworth_fixed iirTaps_2_Hz;
extern butterworth_fixed iirTaps_5_Hz;
extern butterworth_fixed iirTaps_10_Hz;
extern butterworth_fixed iirTaps_20_Hz;
extern butterworth_fixed iirTaps_25_Hz;
extern butterworth_fixed iirTaps_40_Hz;
extern butterworth_fixed iirTaps_50_Hz;

extern bartlett_fixed firTaps_5_Hz;
extern bartlett_fixed firTaps_10_Hz;
extern bartlett_fixed firTaps_20_Hz;
extern bartlett_fixed firTaps_40_Hz;

void Bartlett_Q27_PushSample( bartlett_fixed *filter, uint32_t *x, uint32_t sample );
uint32_t Bartlett_Q27_Filter( bartlett_fixed *coefficients, uint32_t *x, uint32_t *y );
uint32_t Apply_Bartlett_Q27_Filter( bartlett_fixed *coefficients,
                                    uint8_t        sensor,
                                    uint32_t       *x );

void FilterInit(void); // Fixed point init


/** @brief Rolling avereage "boxcar" filter
  moving average structure including pointer to the circular buffer
  its size, the running sum and current average. The function pushes on a
  new value poping off the oldest in a circular buffer summing the rawData
  subtracting off the last value before it is popped off providing a window
  the size of the circular buffer for the running average.
*/
typedef struct {
	uint32_t size;       // length of the circular buffer
	uint32_t *oldValues; // raw data to be avereraged
	uint32_t index;      // head/tail pointer for new data
	uint32_t startIndex; // index for loading the first time
	uint32_t sum;        // running total of the contents of the buffer
	float    average;    // running average of the contents of the buffer
} movingAverage;

/**
@brief standardized unsigned word buffer
*/
typedef struct {
	uint32_t size;    // how many items are in the buffer
	uint32_t *values; // pointer to the pyload array
} movingBuffer;

void init_average(movingAverage *avg,  movingBuffer *circular);
void calc_average(movingAverage *avg,
				  uint32_t      rawData);
void destruct_average(movingAverage *avg);

/** @brief "debounce" structure including pointer to the circular buffer
  its size and the running sum. The function works like a moving average
  pushing on a new value poping off the oldest in a circular buffer
  summing the number of fails (1) subtracting off the last value before
  it is popped off providing a window of the number of failures the
  size of the circular buffer.
*/
typedef struct {
	uint32_t size;		 // length of the circular buffer
	uint8_t  *oldValues; // binary pass [0], fail[1] value
	uint32_t index;      // head/tail pointer for new data
	uint32_t startIndex; // index for loading the first time
	uint32_t sum;        // running total of the number of failures
} debounce;

/**
@brief standardized unsigned byte buffer
*/
typedef struct {
	uint32_t size;   // number of elements in the buffer
	uint8_t *values; // buffer payload (binary)
} debounceBuffer;

void debounce_init(debounce *dBounce, debounceBuffer *buffer);
void debounce_calc(debounce *dBounce,
				   uint8_t   value);


/** ****************************************************************************
 * @brief smoothingFilter for GPS NAV.speed low pass filtering
 * @author Jung Soon Jang
 * @date   2009-04-10 23:20:59Z
 * @ver 8719
 * @brief smoothing data using a moving average filtering with a robust weight
 *  in order to exclude outliers. The maximum delay occurred in the smoothed
 *  data is 400 msecs. This technique is essentially same as a low pass filter
 *  except it is time-independent.
 * @version
 *	06/26/2006	JJANG	initial creation
 *  07/01/2006  DA      Added "if(j==0) break;"
 *						Added "#define" to facilitate filter tuning
 *						Changed to three points
 *  04.2007 DA  Cleaned up, Doxygenized, and finalized for NAV440 release.
 ******************************************************************************/
#define MEDIAN_FILTER_DATA_SIZE  3
#define MOVING_WINDOW_LENGTH     3
#define TRUST_FACTOR             6

float smoothing_filter(float *in);

// delta struct
typedef struct {
    double oldValues[3]; // moving window of delta values
    double sum;
    double last; // Last whole value
} gpsDeltaStruct;

// takes the change in state each time step and averages the amount of change
// over 3 frames with a 6x threshold for filtering
double avgDeltaSmoother(double in, gpsDeltaStruct* data);
void thresholdSmoother( double dataIn[3], float dataOut[3]);


#endif