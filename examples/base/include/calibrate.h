/** ***************************************************************************
 * @file calibrate.h DMU380 sensor calibration algorithms.
 * @Author
 * @date   September, 2008
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
#include "xbowsp_algorithm.h"

void CalibrateInit(void);
void CalibrateFilter(void);
void CalibrateApply( void );

void orientSensors( double *scaledSensors );
void orientSensors_q27( int32_t *scaledSensors_q27 );
void LimitSensorValues( double *scaledSensors );