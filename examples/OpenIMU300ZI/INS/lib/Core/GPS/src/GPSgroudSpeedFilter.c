/** ***************************************************************************
 * @file GPSgroudSpeedFilter.c GPS Driver for Intertial/GPS NAV.speed smoother
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * @briefsmoothing data using a moving average filtering with a robust weight
 *  in order to exclude outliers. The maximum delay occurred in the smoothed
 *  data is 400 msecs. This technique is essentially same as a low pass filter
 *  except it is time-independent.
 *
 *	This module provides high level GPS management,  based on product type and
 *  GPS receiver. This module is interfaced with NAV processing and other
 *  specific GPS process files. The functions are provided in this files are
 *  common for all GPS protocol. Protocol-specific process is provided in other
 *  GPS files
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
#ifdef GPS

#include <stdio.h>
#include <math.h>
#include "driverGPS.h"

#define MEDIAN_FILTER_DATA_SIZE  3
#define MOVING_WINDOW_LENGTH     3
#define TRUST_FACTOR             6

/** ****************************************************************************
 * @name: _median LOCAL  routine for calculating a median value
 * @author JJANG
 * @param [in]  in - new value
 * @retval actual baud rate value
 ******************************************************************************/
float _median1(float *in)
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
		while( dta[j - 1] > temp) {
		  dta[j] = dta[j - 1];
		  j--;
		  if(j == 0)
              break;
		}
		dta[j] = temp;
	  }
  out = ( !(MEDIAN_FILTER_DATA_SIZE % 2) ) ?
        (dta[MEDIAN_FILTER_DATA_SIZE / 2 - 1] + dta[MEDIAN_FILTER_DATA_SIZE / 2]) / 2 :
         dta[(MEDIAN_FILTER_DATA_SIZE - 1) / 2];
  return out;
}

/** ****************************************************************************
 * @name: lowpass_filter LOCAL routine for smoothing data using a moving average
 *        with a robust weight
 * @author JJANG
 * @param [in]  in - new value
 * @retval actual baud rate value
 * ccalled in processNMEAGPS.cpp
 ******************************************************************************/
float lowpass_filter(float *in)
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

	///absolute value of residual of ith data
	r[MEDIAN_FILTER_DATA_SIZE - 1] = (float)fabs(dtapoint[MOVING_WINDOW_LENGTH - 1] - out);
	///median value of the residuals over window
	MAD = _median1(r);

	///if the residual is larger than a given threshold
	///exclude data (6 * the median)
	if (r[MOVING_WINDOW_LENGTH - 1] > TRUST_FACTOR * MAD)
        weight[MOVING_WINDOW_LENGTH - 1] = 0;
	else
        weight[MOVING_WINDOW_LENGTH - 1] = 1;

	///do moving avergae with new weight
	out = 0;
    sum = 0;
	for(j = 0;j < MOVING_WINDOW_LENGTH; j++) {
		   out += weight[j] * dtapoint[j];
	       sum += weight[j];
	}
	if (sum == 0)
        sum = 1;
	out /= (float)sum;

	for (i = 0; i < (MOVING_WINDOW_LENGTH - 1); i++) {
	   dtapoint[i] = dtapoint[i + 1];
	   r[i] = r[i + 1];
	   weight[i] = weight[i + 1];
	}
	weight[MOVING_WINDOW_LENGTH - 1] = 1;

	return out;
}

#endif // GPS