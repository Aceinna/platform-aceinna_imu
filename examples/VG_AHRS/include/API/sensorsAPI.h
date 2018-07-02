/** ***************************************************************************
 * @file magAPI.h API functions for Magnitometer functionality
 *
 * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
 * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
 * PARTICULAR PURPOSE.
 *
 *****************************************************************************/
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

#ifndef _SENSORS_API_H
#define _SENSORS_API_H

#include <stdint.h>

/** ****************************************************************************
 * @name AccelerometerGetGain
 * @brief Get gain of accelerometer in LSB/G
 * @param N/A
 * @retval 0 if error, gain if succesful
 ******************************************************************************/
uint16_t AccelerometerGetGain();

/** ****************************************************************************
 * @name MagnetometerGetGain
 * @brief Get gain of magnetometer in LSB/gauss
 *
 * @param N/A
 * @retval 0 if success, error otherwise
 ******************************************************************************/
uint16_t MagnetometerGetGain();

/** ****************************************************************************
 * @name GyroGetGain return the gain for the device
 * @param N/A
 * @retval gain
 ******************************************************************************/
uint16_t GyroGetGain();

/** ****************************************************************************
 * @name AccelerometerStartReading
 * Description:  This doesn't kick off a conversion cycle as the sampling
 *   asynchronous to the system. It just makes it so the data ready line
 *   goes (but it goes whenever it was ready).
 * @param N/A
 * @retval 0 if success, error otherwise
 ******************************************************************************/
uint8_t AccelerometerStartReading();

/** ****************************************************************************
 * @name MagnetometerStartReading
 * @brief This kicks off conversion of magnetometers
 * @param N/A
 * @retval always 0
 ******************************************************************************/
uint8_t MagnetometerStartReading();

/** ****************************************************************************
 * @name IsGyroDoneReading get the read complete status for the gyro
 * @param N/A
 * @retval N/A
 ******************************************************************************/
uint8_t IsGyroDoneReading();

/** ****************************************************************************
 * @name GyroGetLastReading get the data saturate it and transform it to
 * DMU380 coordinate frame
 * @param N/A
 * @retval N/A
 ******************************************************************************/
uint8_t GyroGetLastReading(int16_t *readings);

/** ****************************************************************************
 * @name IsAccelerometerDoneReading
 * @brief Checks to see if  transaction is complete
 * @param N/A
 * @retval 0 if success, error otherwise
 ******************************************************************************/
uint8_t IsAccelerometerDoneReading();

/** ****************************************************************************
 * @name IsMagnetometerDoneReading
 * @brief Checks to see if magnetometer transaction is complete
 * @param N/A
 * @retval 0 if success, error otherwise
 ******************************************************************************/
uint8_t IsMagnetometerDoneReading();

/** ****************************************************************************
 * @name: AccelerometerGetLastReading
 * @brief Gets the data that has been accumulated, filters it, and returns it to
 *        the calling routine
 * @param [out] readings - array of read data (NUM_AXIS)
 * @retval 0 if success, error otherwise
 ******************************************************************************/
uint8_t AccelerometerGetLastReading(int16_t *readings);

/** ****************************************************************************
 * @name MagnetometerGetLastReading
 * @brief Gets the data that was last read by I2C process saturating and
 * transforming to the DMU380 coordinate system
 *
 * @param array of reading (NUM_AXIS)
 * @retval 0 if success, error otherwise
 * @brief The tempX/Y/Z have to be there for the data to come out right in
 * readings[]
 ******************************************************************************/
uint8_t MagnetometerGetLastReading(int16_t *readings);

/** ****************************************************************************
 * @name AccelerometerDataReadyIRQ pin 9 intrrupt aserted
 * @brief This interrupt indicates the data is ready on the accelerometer. Kicks
 *        off reading data.
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void AccelerometerDataReadyIRQ(void);

/** ****************************************************************************
 * @name MagnetomterDataReadyIRQ pin 5 interrupt asserted
 * @brief This interrupt indicates the data is ready on the magenetomter. Kicks
 *        off reading data.
 * @param N/A
 * @retval N/A
 ******************************************************************************/
void MagnetomterDataReadyIRQ(void);


#endif
