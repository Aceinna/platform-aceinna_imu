/** ***************************************************************************
 * @file filter.c iir and fir digital filter implementations
 * @Author
 * @date   September, 2008
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 * 07.11.14 DKH Added Q27 versions of Bartlett fir and Butterworth iir filters
 *****************************************************************************/
#include <stdint.h>
#include <string.h> // For memset()
#include <stdlib.h> // malloc
#include <math.h>   // fabs()

#include "dmu.h"   // For NUM_SENSOR_READINGS, etc
#include "xbowsp_algorithm.h"   // For XACCEL, etc
#include "filter.h"

#include "ucb_packet.h"   // for AHRS_SYS

// Butterworth (IIR) low-pass filter coefficients Q27
// 200 Hz Sampling
static int32_t b_2_Hz_iir_200HzSamp[] = {134217728, -256516528,  122805978};
static int32_t g_2_Hz_iir_200HzSamp   =     126794;

static int32_t b_5_Hz_iir_200HzSamp[] = {134217728,  -238723916, 107481912};
static int32_t g_5_Hz_iir_200HzSamp   =     743931;

static int32_t b_10_Hz_iir_200HzSamp[] = {134217728, -209516300,  86080746};
static int32_t g_10_Hz_iir_200HzSamp   =    2695544;

static int32_t b_20_Hz_iir_200HzSamp[] = {134217728, -153408246,  55405293};
static int32_t g_20_Hz_iir_200HzSamp   =    9053694;

// 25 Hz Filter (200 Hz sampling)
static int32_t b_25_Hz_iir_200HzSamp[] = {134217728, -126541687,   44739243};
static int32_t g_25_Hz_iir_200HzSamp   =    13103821;

// 40 Hz filter (200 Hz sampling)
static int32_t b_40_Hz_iir_200HzSamp[] = {134217728,  -49597125,   26281940};
static int32_t g_40_Hz_iir_200HzSamp   =    27725636;

static int32_t b_50_Hz_iir_200HzSamp[] = {134217728,          0,  23028122};
static int32_t g_50_Hz_iir_200HzSamp   =   39311462;

// 100 Hz Sampling
static int32_t b_2_Hz_iir_100HzSamp[] = {134217728, -244637972,  112364619};
static int32_t g_2_Hz_iir_100HzSamp   =     486094;

static int32_t b_5_Hz_iir_100HzSamp[] = {134217728,  -209516300, 86080746};
static int32_t g_5_Hz_iir_100HzSamp   =     2695544;

static int32_t b_10_Hz_iir_100HzSamp[] = {134217728, -153408246,  55405293};
static int32_t g_10_Hz_iir_100HzSamp   =    9053694;

static int32_t b_20_Hz_iir_100HzSamp[] = {134217728, -49597125,  26281940};
static int32_t g_20_Hz_iir_100HzSamp   =    27725636;

// 25 Hz Filter (100 Hz sampling)
static int32_t b_25_Hz_iir_100HzSamp[] = {134217728,         0,  23028122};
static int32_t g_25_Hz_iir_100HzSamp   =    39311462;

// 40 Hz filter (100 Hz sampling)
static int32_t b_40_Hz_iir_100HzSamp[] = {134217728,   153408246,    55405293};
static int32_t g_40_Hz_iir_100HzSamp   =    85757817;

// 49 Hz LPF (50 Hz is Nyquist and the filter is not defined at this cutoff freq)
static int32_t b_50_Hz_iir_100HzSamp[] = {134217728, 256516528,  122805978};
static int32_t g_50_Hz_iir_100HzSamp   =   128385058;

// Bartlett fir Q27 taps (coefficients)
// 200 Hz Sampling
static int32_t b_5Hz_fir_200HzSamp[]  = { 856611, 1714775, 2574283, 3434922, 4296482, 5158750, 6021514, 6884562, 7747681, 8610659, 9473284, 10335341 };
static int32_t b_10Hz_fir_200HzSamp[] = { 3119269, 6303690, 9534154, 12791165, 16054959, 19305626 };
static int32_t b_20Hz_fir_200HzSamp[] = { 9407719, 22044088, 35657057 };
static int32_t b_40Hz_fir_200HzSamp[] = { 30038942, 74139843 };

// 100 Hz Sampling
static int32_t b_5Hz_fir_100HzSamp[]  = { 3119269, 6303690, 9534154, 12791165, 16054959, 19305626 };
static int32_t b_10Hz_fir_100HzSamp[] = { 9407719, 22044088, 35657057 };
static int32_t b_20Hz_fir_100HzSamp[] = { 30038942, 74139843 };
static int32_t b_40Hz_fir_100HzSamp[] = { 10737418, 112742892 };

	butterworth_fixed iirTaps_2_Hz;
	butterworth_fixed iirTaps_5_Hz;
	butterworth_fixed iirTaps_10_Hz;
	butterworth_fixed iirTaps_20_Hz;
	butterworth_fixed iirTaps_25_Hz;
	butterworth_fixed iirTaps_40_Hz;
	butterworth_fixed iirTaps_50_Hz;

	bartlett_fixed firTaps_5_Hz;
	bartlett_fixed firTaps_10_Hz;
	bartlett_fixed firTaps_20_Hz;
	bartlett_fixed firTaps_40_Hz;

void FilterInit()
{
    // load the coefficients for the iir butterworth filters
    memset(&iirTaps_2_Hz,  0, sizeof(iirTaps_2_Hz));
    memset(&iirTaps_5_Hz,  0, sizeof(iirTaps_5_Hz));
    memset(&iirTaps_10_Hz, 0, sizeof(iirTaps_10_Hz));
    memset(&iirTaps_20_Hz, 0, sizeof(iirTaps_20_Hz));
    memset(&iirTaps_25_Hz, 0, sizeof(iirTaps_25_Hz));
    memset(&iirTaps_40_Hz, 0, sizeof(iirTaps_40_Hz));
    memset(&iirTaps_50_Hz, 0, sizeof(iirTaps_50_Hz));

    // 200 Hz sampling
    if( UcbGetSysType() <= UNAIDED_AHRS_SYS ) {
        iirTaps_2_Hz.b = (int32_t*)b_2_Hz_iir_200HzSamp;
        iirTaps_2_Hz.g = g_2_Hz_iir_200HzSamp;

        iirTaps_5_Hz.b = (int32_t*)b_5_Hz_iir_200HzSamp;
        iirTaps_5_Hz.g = g_5_Hz_iir_200HzSamp;

        iirTaps_10_Hz.b = (int32_t*)b_10_Hz_iir_200HzSamp;
        iirTaps_10_Hz.g = g_10_Hz_iir_200HzSamp;

        iirTaps_20_Hz.b = (int32_t*)b_20_Hz_iir_200HzSamp;
        iirTaps_20_Hz.g = g_20_Hz_iir_200HzSamp;

        iirTaps_25_Hz.b = (int32_t*)b_25_Hz_iir_200HzSamp;
        iirTaps_25_Hz.g = g_25_Hz_iir_200HzSamp;

        iirTaps_40_Hz.b = (int32_t*)b_40_Hz_iir_200HzSamp;
        iirTaps_40_Hz.g = g_40_Hz_iir_200HzSamp;

        iirTaps_50_Hz.b = (int32_t*)b_50_Hz_iir_200HzSamp;
        iirTaps_50_Hz.g = g_50_Hz_iir_200HzSamp;

        // load the coefficients for the fir bartlett filters
        firTaps_5_Hz.taps = b_5Hz_fir_200HzSamp;
        firTaps_5_Hz.N = 2 * (sizeof( b_5Hz_fir_200HzSamp) / sizeof(int32_t));

        firTaps_10_Hz.taps = b_10Hz_fir_200HzSamp;
        firTaps_10_Hz.N = 2 * (sizeof( b_10Hz_fir_200HzSamp) / sizeof(int32_t));

        firTaps_20_Hz.taps = b_20Hz_fir_200HzSamp;
        firTaps_20_Hz.N = 2 * (sizeof( b_20Hz_fir_200HzSamp) / sizeof(int32_t));

        // 40 Hz Bartlett requires three terms and isn't symmetric like the others
        firTaps_40_Hz.taps = b_40Hz_fir_200HzSamp;
        firTaps_40_Hz.N = 3;
    } else {
        // 100 Hz Sampling
        iirTaps_2_Hz.b = (int32_t*)b_2_Hz_iir_100HzSamp;
        iirTaps_2_Hz.g = g_2_Hz_iir_100HzSamp;

        iirTaps_5_Hz.b = (int32_t*)b_5_Hz_iir_100HzSamp;
        iirTaps_5_Hz.g = g_5_Hz_iir_100HzSamp;

        iirTaps_10_Hz.b = (int32_t*)b_10_Hz_iir_100HzSamp;
        iirTaps_10_Hz.g = g_10_Hz_iir_100HzSamp;

        iirTaps_20_Hz.b = (int32_t*)b_20_Hz_iir_100HzSamp;
        iirTaps_20_Hz.g = g_20_Hz_iir_100HzSamp;

        iirTaps_25_Hz.b = (int32_t*)b_25_Hz_iir_100HzSamp;
        iirTaps_25_Hz.g = g_25_Hz_iir_100HzSamp;

        iirTaps_40_Hz.b = (int32_t*)b_40_Hz_iir_100HzSamp;
        iirTaps_40_Hz.g = g_40_Hz_iir_100HzSamp;

        iirTaps_50_Hz.b = (int32_t*)b_50_Hz_iir_100HzSamp;
        iirTaps_50_Hz.g = g_50_Hz_iir_100HzSamp;

        // load the coefficients for the fir bartlett filters
        firTaps_5_Hz.taps = b_5Hz_fir_100HzSamp;
        firTaps_5_Hz.N = 2 * (sizeof( b_5Hz_fir_100HzSamp) / sizeof(int32_t));

        firTaps_10_Hz.taps = b_10Hz_fir_100HzSamp;
        firTaps_10_Hz.N = 2 * (sizeof( b_10Hz_fir_100HzSamp) / sizeof(int32_t));

        firTaps_20_Hz.taps = b_20Hz_fir_100HzSamp;
        firTaps_20_Hz.N = 2 * (sizeof( b_20Hz_fir_100HzSamp) / sizeof(int32_t));

        // 40 Hz Bartlett requires three terms and isn't symmetric like the others
        firTaps_40_Hz.taps = b_40Hz_fir_100HzSamp;
        firTaps_40_Hz.N = 3;
    }
}

/** ****************************************************************************
 * @name: Butterworth_Q27_LoadSample - load the input and delay buffer with the
 *        latest sample and push the older samples down.
 * @brief
 * @param [in] x - 3 sample delay buffer
 * @param [in] sample - new data to push onto the queue
 * @retval N/A
 ******************************************************************************/
void
Butterworth_Q27_PushSample(int32_t *x,
                           int32_t sample)
{
    x[2] = x[1]; // simple minded queue
    x[1] = x[0];
    x[0] = sample;
}

/** ***************************************************************************
 * @name Butterworth_Q27_Filter
 * implementation of a direct form 2nd order Butterworth IIR Filter
 * y = filtered data and delay
 * x = scaledSensors raw sensor data + delayed data
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
 * @param [in]  x - pointer to the start of the section of the raw data
 * @param [out] y - filtered and delayed filter data Q27 y/(2^27) to scale back to
 *                  floating point
 * @retval always 0 - uses a return value to force system to wait for fcn
 *         complete before continuing.
 *****************************************************************************/
uint32_t
Butterworth_Q27_Filter( butterworth_fixed *coefficients,
				        int32_t           *x,
				        int32_t           *y )
{
    int64_t poles = 0;
    int64_t zeros = 0;

    poles = (int64_t)coefficients->g * (int64_t)( (x[0] + x[2]) + 2 * x[1] );
    zeros = ((int64_t)coefficients->b[1] * (int64_t)y[1]) + ( (int64_t)coefficients->b[2] * (int64_t)y[2]);

	// filtered, rounded data delay queue - push new value on
    y[0] = (int32_t)( ((poles - zeros)  + 67108864) >> 27); //  67108864 = 0.5 * 2^27
    y[2] = y[1];
    y[1] = y[0];

	return 0; // force a context switch so function completes before returning
}

/** ****************************************************************************
 * @name: Butterworth_Q27_Filter - load the input and delay buffer with the
 *        latest sample and apply the filter.
 * @brief
 * @param [in] coefficients - iir fitler coefficients
 * @param [in] sensor - index into the raw data array
 * @retval always return 0
 ******************************************************************************/
uint32_t
Apply_Butterworth_Q27_Filter(butterworth_fixed *coefficients,
                             uint8_t           sensor)
{
    static int32_t  iir_x[NUM_SENSOR_READINGS][3]; // [11][3] raw + delay data
    static int32_t  iir_y[NUM_SENSOR_READINGS][3]; // [11][3] filtered + delay data

    Butterworth_Q27_PushSample((int32_t*)iir_x[sensor],
                               gAlgorithm.rawSensors[sensor]);

    Butterworth_Q27_Filter( coefficients,
                           (int32_t*)iir_x[sensor],
                           (int32_t*)iir_y[sensor]);
    gAlgorithm.rawSensors[sensor] = iir_y[sensor][0];

	return 0; // force fcn to finish before returning
}

/** ****************************************************************************
 * @name: Bartlett_Q27_PushSample - load the input and delay buffer with the
 *        latest sample and push the older samples down.
 * @brief
 * @param [in] x - [Number of Taps] sample delay buffer
 * @param [in] sample - new data to push onto the queue
 * @retval N/A
 ******************************************************************************/
void
Bartlett_Q27_PushSample( bartlett_fixed *filter,
                         uint32_t       *x,
                         uint32_t       data )
{   // crude way to do a fifo - with too small a buffer too much arithmatic is
	// required to manage the indexing
	memmove(&x[1], &x[0], (filter->N - 1) * sizeof(uint32_t) );
	x[0] = data;
}

/** ****************************************************************************
 * @name: filter a genric Q27 fixed point Bartlett FIR filter
 * @brief Any order filter can be defined
 * 20 Hz Low-Pass Filter Coefficients (for Bartlett filter)
 *   a0 = a7 = 0.0;
 *   a1 = a6 = 0.070092967075280;
 *   a2 = a5 = 0.164241256811824;
 *   a3 = a4 = 0.265665776112896;
 * @param [in] coefficients - data struct with the order of the filter and the
 *        taps Q27 x *2^27 to scale to Q27
 * @param [in]  x - pointer to the start of the section of the raw data
 * @param [out] y - pointer to acumulated filter data Q27 y/(2^27) to scale
 *                  back to floating point
 * @retval always 0 - uses a return value to force system to wait for fcn
 *         complete before continuing.
 ******************************************************************************/
uint32_t
Bartlett_Q27_Filter( bartlett_fixed *coefficients,
                     uint32_t       *x,
                     uint32_t       *y )
{
	// Can handle up to 256 coefficients
    uint8_t i = 0;

    // Filtering variables
    uint64_t tmp = 0;
    uint64_t multiplier;
    uint64_t sum;

    // Check for 40 Hz Bartlett filter (the difference equations are not symmetric in the same way
    //   as the others)
    if( coefficients->N == 3 ) {
        multiplier = coefficients->taps[0];
        sum = (uint64_t)( x[0] + x[ coefficients->N - 1] );
        tmp = tmp + ( multiplier * sum );

        multiplier = coefficients->taps[1];
        sum = (uint64_t)( x[1] );
        tmp = tmp + ( multiplier * sum );
    } else {
        for (i = 0; i < coefficients->N / 2; i++)
        {   // This takes advantage of the symmetry of a sinc function
            //   *y += coefficients->taps[i] * ( x[i] + x[ coefficients->N - i - 1] );
            multiplier = coefficients->taps[i];
            sum = (uint64_t)( x[i] + x[ coefficients->N - i - 1] );

            tmp = tmp + ( multiplier * sum );
        }
    }
    tmp = tmp >> 27;

    //*y += 67108864; // 0.5 * 2^27 (for rounding)
    *y = tmp;
    return 0; // force fcn to finish before returning
}

/** ****************************************************************************
 * @name: Apply_Bartlett_Q27_Filter - load the input and delay buffer with the
 *        latest sample then run the filter.
 * @brief
 * @param [in] coefficients - fir filter tap structure
 * @param [in] sensor - index into the sensor array
 * @param [in] x - [Number of Taps] sample delay buffer
 * @retval always return 0
 ******************************************************************************/
uint32_t
Apply_Bartlett_Q27_Filter( bartlett_fixed *coefficients,
                           uint8_t        sensor,
                           uint32_t       *x )
{
    uint32_t filteredValue = 0;

    // Filter the sensor data using buffered (raw) sensor readings
    Bartlett_Q27_Filter( coefficients,
                         x,  &filteredValue );

    // Write the (raw) value in rawSensors to the filter buffer
    Bartlett_Q27_PushSample( coefficients,
                             x,
                             gAlgorithm.rawSensors[sensor] );

    // Update the output value with the filtered value
    gAlgorithm.rawSensors[sensor] = (uint32_t)filteredValue; //x[0];

    return 0; // finish before returning
}

/// @brief
/// To cascade a series of butterworth filters (to increase the rolloff from 20
/// dB/dec to 40, 60, etc.), need to make the past and past-past arrays
/// two-dimensional and provide an index as an argument to the function to
/// indicate the filter stage.

/// Second-order low-pass Butterworth filters
/// Difference equation:
///  b1*y(k) = g*( a1*x(k) + a2*x(k-1) + a3*x(k-2) ) - ( b2*y(k-1) + b3*y(k-2) )

/** ****************************************************************************
 * @name: init_average set up a moving average filter, set defaults and pick size
 * of the delay buffer and allocate the memory for the buffer.
 * @brief
 * @param [in] avg - new data in
 * @retval N/A
 ******************************************************************************/
void init_average(movingAverage *avg, movingBuffer *circular)
{
	avg->startIndex =  0;
	avg->index      =  0;
	avg->size       =  circular->size;
	avg->sum        =  0;
	avg->average    =  0.0f;
	avg->oldValues  =  circular->values;
	memset(avg->oldValues, 0, sizeof(uint32_t) * avg->size);
}

/** ****************************************************************************
 * @name: destruct_average memory allocation cleanup
 * @brief
 * @param [in] avg - data structure with the dynmauically allocated buffer
 * @retval N/A
 ******************************************************************************/
void destruct_average(movingAverage *avg)
{
	free(avg->oldValues);
}

/** ****************************************************************************
 * @name: calc_average A moving average filter to smooth the data
 * @brief
 * @param [out] avg - data structure with the sum and delay buffer
 * @param [in] values - new data in
 * @retval N/A
 ******************************************************************************/
void calc_average(movingAverage *avg,
	              uint32_t       rawData)
{
    if (avg->startIndex >= (avg->size - 1) ) {
        avg->sum -= avg->oldValues[avg->index]; ///< subtract the oldest value
    } else {
        avg->startIndex++; ///< increment until the buffer is full at start
    }
    avg->sum += rawData; ///< update with new data
    avg->oldValues[avg->index] = rawData; ///< push the new raw value over the oldest

    avg->index++; /// circular buffer index
//    avg->index %= avg->size; ///< roll over
    // Remove mod-function; replace with if-statement
    if( avg->index >= avg->size ) {
        avg->index = 0;
    }

    avg->average = (float)avg->sum / (float)avg->startIndex;
}

/** ***************************************************************************
 * @name debounce_init() API pass in the pointer to the buffer, set the size
 * @brief debounce is a BIT function to threshold setting a failure for
 *        systems that may have intermittent error that is not fatal. compare
 *        the size with the sum to determine threshold
 *
 * @param [in] dBounce - data structure with a pointer to a cirular buffer
 * @param [in] buffer - the cirucular buffer to point to in the data structure
 * @retval N/A
 ******************************************************************************/
void debounce_init(debounce       *dBounce,
	               debounceBuffer *buffer)
{
	dBounce->startIndex =  0;
	dBounce->index      =  0;
	dBounce->size       =  buffer->size;
	dBounce->sum        =  0;
	dBounce->oldValues  =  buffer->values;
	memset(dBounce->oldValues, 0, sizeof(uint8_t) * dBounce->size);
}

/** ***************************************************************************
 * @name debounce_calc() API pass in value to add to the debounce buffer
 * @brief debounce is a BIT function to threshold setting a failure for
 *        systems that may have intermittent error that is not fatal. Compare
 *        the size with the sum to determine threshold
 *
 * @param [in] dBounce - data structure with a pointer to a cirular buffer
 * @param [in] newValue - binary value to indicate a single fail or pass
 * @retval N/A
 * @brief this could bee the base class for a rolling average
 ******************************************************************************/
void debounce_calc(debounce *dBounce,
                   uint8_t    newValue)
{
	if (dBounce->startIndex >= (dBounce->size - 1) ) {
		dBounce->sum -= dBounce->oldValues[dBounce->index]; ///< subtract the oldest value off
	} else {
		dBounce->startIndex++; ///< Keep track of filling the buffer the first time
	}
	dBounce->sum += newValue; ///< add the newest data
	dBounce->oldValues[dBounce->index] = newValue; ///< new value overwrites the oldest

    dBounce->index++; ///< circular buffer index
//    dBounce->index %= dBounce->size; ///< roll over
    // Remove mod-function; replace with if-statement
    if( dBounce->index >= dBounce->size ) {
        dBounce->index = 0;
    }
}

/** ****************************************************************************
 * @name: _median LOCAL routine for calculating a median value
 * @author JJANG
 * @param [in]  in - new value
 * @retval median value
 ******************************************************************************/
float _median(float *in)
{
   int i;
   int j;
   float temp;
   float out;
   float dta[MEDIAN_FILTER_DATA_SIZE];

   for(i = 0; i < MEDIAN_FILTER_DATA_SIZE; i++)
       dta[i] = in[i];

   for (i = 1; i < MEDIAN_FILTER_DATA_SIZE; i++) {
		temp = dta[i];
        j = i;
		while( dta[j - 1] > temp) { // sort low to high
		  dta[j] = dta[j - 1];
		  j--;
		  if(j == 0)
              break;
		}
		dta[j] = temp;
	  }
  out =  dta[1]; // send out the middle sorted value
  return out;
}

/** ****************************************************************************
 * @name: smoothing_filter API routine for smoothing data using a moving average
 *        with a robust weight. Filters out data spikes over 6 times the median
 * @author JJANG
 * @param [in]  in - new value
 * @retval filtered value
 * ccalled in processNMEAGPS.cpp
 ******************************************************************************/
float smoothing_filter(float *in)
{
	static int    weight[MOVING_WINDOW_LENGTH];
	static float  dtapoint[MOVING_WINDOW_LENGTH];
    static float  r[MOVING_WINDOW_LENGTH];
	static unsigned char initialFlag = 0;
	int			 i;
    int          j;
    int          sum;
	float		 out;
    float        MAD;

	if (initialFlag == 0) {
		initialFlag = 1;

	   for(i = 0; i < MOVING_WINDOW_LENGTH; i++) {
			weight[i] = 1;
			dtapoint[i] = 0;
			r[i] = 0;
	   }
	}
	dtapoint[MOVING_WINDOW_LENGTH - 1] = *in;

	///compute moving average
	out = 0;
    sum = 0;
	for(j = 0;j < MOVING_WINDOW_LENGTH; j++) {
		   out += weight[j] * dtapoint[j];
		   sum += weight[j];
	}

	if (sum == 0)
        sum = 1;
	out /= (float)sum;

	/// absolute value of residual of ith data
	r[MEDIAN_FILTER_DATA_SIZE - 1] = (float)fabs(dtapoint[MOVING_WINDOW_LENGTH - 1] - out);
	/// median value of the residuals over window (sorts r[])
	MAD = _median(r);

	/// if the residual is larger than a given threshold (6x the median)
	/// exclude data
    // only weighting the largest value
	if (r[MOVING_WINDOW_LENGTH - 1] > TRUST_FACTOR * MAD)
        weight[MOVING_WINDOW_LENGTH - 1] = 0;
	else
        weight[MOVING_WINDOW_LENGTH - 1] = 1;

	///do moving average with new weight
	out = 0;
    sum = 0;
	for(j = 0;j < MOVING_WINDOW_LENGTH; j++) {
		   out += weight[j] * dtapoint[j];
	       sum += weight[j];
	}
	if (sum == 0)
        sum = 1;
	out /= (float)sum;

	// << everything one element
	for (i = 0; i < (MOVING_WINDOW_LENGTH - 1); i++) {
	   dtapoint[i] = dtapoint[i + 1];
	   r[i] = r[i + 1];
	   weight[i] = weight[i + 1];
	}
	weight[MOVING_WINDOW_LENGTH - 1] = 1;

	return out;
}

/** ****************************************************************************
 * @name: avgDeltaSmoother API routine using data structure to hold instance data
 *        for smoothing gps using a 3 item moving average of the change in values
  *       (deltas) that removes data "spikes" over 6 times the average
 * @author
 * @param [in]  in - new value to add to the average
 * @param [in]  data - filter data for lat, lon or alt
 * @retval last value if > than 6x avg or pass the value back out
 ******************************************************************************/
double avgDeltaSmoother( double          rawData,
                         gpsDeltaStruct *delta)
{
    double thisDelta;
    double returnVal;

    thisDelta = delta->last - rawData;

    delta->sum -= delta->oldValues[2]; // pop the old value off of the sum
    delta->sum += thisDelta;           // push the new value in the sum

    // push the data down the hold stack
    delta->oldValues[2] = delta->oldValues[1];
	delta->oldValues[1] = delta->oldValues[0];
	delta->oldValues[0] = thisDelta;
    if ( fabs(thisDelta) > 6.0 * fabs( delta->sum / 3.0 ) ) {
        returnVal = delta->last; // filter (omit) the value
    } else
        returnVal = rawData; // send the input value back out

    delta->last = rawData;             // hold input value for next entry
    return returnVal;
}

/** ****************************************************************************
 * @name: thresholdSmoother API threshold filter that uses total speed from Vned
 *        for smoothing gps "glitches" (in delta speed) "spikes" over 15m/s
 * @author
 * @param [in]  in - new value to compare to threshold
 * @param [in]  data - return last good data or current good data
 * @retval N/A
 ******************************************************************************/
void thresholdSmoother( double         vNedIn[3],
                        float          vNedOut[3])
{
    double        speedSqCurr;
    double        SPEED_SQ_LIMIT = 225;  // limit the change to 15 m/s
    static double speedSqPast;
    static double vNedPast[3];

    // "glitch" filter
    // Compute the speed
    speedSqCurr = vNedIn[0]*vNedIn[0] + vNedIn[1]*vNedIn[1] + vNedIn[2]*vNedIn[2];

    // "Filter" the velocity signal if the delta is greater than 15 [ m/s ]
    //   (This is an 8% margin over expected system dynamics)
    if( fabs( speedSqCurr - speedSqPast ) > SPEED_SQ_LIMIT ) {
        // If a glitch is encountered, set output to last "good" value.
        //  Do not update past with current.
        vNedOut[0] = (float)vNedPast[0];
        vNedOut[1] = (float)vNedPast[1];
        vNedOut[2] = (float)vNedPast[2];
    } else {
        // Do not change GPS velocity, update past values with current values
        speedSqPast = speedSqCurr;
        vNedPast[0] = vNedIn[0];
        vNedPast[1] = vNedIn[1];
        vNedPast[2] = vNedIn[2];

        vNedOut[0] = (float)vNedIn[0];
        vNedOut[1] = (float)vNedIn[1];
        vNedOut[2] = (float)vNedIn[2];
    }
}
