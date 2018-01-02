/** ***************************************************************************
 * @file   taskDataAcquisition.h
 * @Author
 * @date   September, 2008
 * @brief  Copyright (c) 2013, 2014 All Rights Reserved.
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 * sensor data acquisition task runs at 100Hz, gets the data for each sensor
 * and applies available calibration
 ******************************************************************************/
#ifndef _TASK_DATA_ACQUISITION_H_
#define _TASK_DATA_ACQUISITION_H_
// Specify the limit used in the int16-limiter
#define INT16_LIMIT 32765
#define INT12_LIMIT 2045

extern void TaskDataAcquisition(void);
void InitExternalSync( FunctionalState NewState );
void InitSensors( void );

void _DisplayDebugOutput( int16_t* reading, uint32_t startTime );
void _ConvertToXBowScaling( int16_t* reading );
// Limiter for int16_t (limits at +/- 32765)
int16_t LimitInt16Value( int16_t value, int16_t limit );

#endif