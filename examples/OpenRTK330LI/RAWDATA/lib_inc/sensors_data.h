/** ***************************************************************************
 * @file   sensors_data.h
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

#ifndef SENSORS_DATA_H
#define SENSORS_DATA_H

#include <stdint.h>
#include "Indices.h"

/* Global Sensor Data structure  */
typedef struct {
    int32_t           rawSensors[NUM_SENSOR_CHIPS][N_RAW_SENS];
    int32_t           rawSensorsCombined[N_RAW_SENS];
    int32_t           rawTempSensors[NUM_SENSOR_CHIPS];
    int32_t           rawTempSensorsCombined;
    float             scaledSensors[NUM_SENSOR_CHIPS][N_RAW_SENS]; 	/// g's, rad/s, G, deg C, (body frame)
    float             scaledSensorsCombined[N_RAW_SENS]; 			/// g's, rad/s, G, deg C, (body frame)
    float             rateAlarm;
    float             accelAlarm;
    uint32_t          samplingTstamp;
}sensors_data_t;

extern sensors_data_t gSensorsData;
extern void     SampleSensorsData();


#endif // SENSORS_DATA 

